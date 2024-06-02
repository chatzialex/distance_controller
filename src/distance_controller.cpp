#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>

using Pose = geometry_msgs::msg::Pose;
using PoseArray = geometry_msgs::msg::PoseArray;
using Odometry = nav_msgs::msg::Odometry;
using Twist = geometry_msgs::msg::Twist;

using namespace std::chrono_literals;

namespace DistanceController {

class DistanceController : public rclcpp::Node {
public:
  DistanceController(const std::string &node_name = kNodeName,
                     const rclcpp::NodeOptions &options = rclcpp::NodeOptions{})
      : Node{node_name, options}, odom_sub_{this->create_subscription<Odometry>(
                                      kOdometryTopicName, 1,
                                      std::bind(&DistanceController::odomSubCb,
                                                this, std::placeholders::_1))},
        twist_pub_{
            this->create_publisher<Twist>(kCommandVelocityTopicName, 1)} {

    RCLCPP_INFO(this->get_logger(), "%s node started.", node_name.c_str());
  }

  bool followTrajectory(const PoseArray &goals);

private:
  struct State {
    double x;
    double y;
    double dx;
    double dy;
  };

  constexpr static char kCommandVelocityTopicName[]{"cmd_vel"};
  constexpr static char kNodeName[]{"distance_controller"};
  constexpr static char kOdometryTopicName[]{"/odometry/filtered"};

  constexpr static double kPositionTolerance{0.01}; // [m]
  constexpr static auto kControlCycle{100ms};
  constexpr static double kDGain{0.7};
  constexpr static double kPGain{1.7};
  constexpr static double kIGain{0.05};

  void odomSubCb(const std::shared_ptr<const Odometry> msg);
  bool goToPoint(const Pose &goal);

  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_{};
  rclcpp::Publisher<Twist>::SharedPtr twist_pub_{};

  std::optional<struct State> state_cur_{std::nullopt};
};

void DistanceController::odomSubCb(const std::shared_ptr<const Odometry> msg) {
  state_cur_ = {msg->pose.pose.position.x, msg->pose.pose.position.y,
                msg->twist.twist.linear.x, msg->twist.twist.linear.y};
}

bool DistanceController::goToPoint(const Pose &goal) {
  const auto x_goal{goal.position.x};
  const auto y_goal{goal.position.y};

  RCLCPP_INFO(this->get_logger(), "Received new goal: x=%f y=%f", x_goal,
              y_goal);

  const auto pid_step{[this](auto e, auto de, auto ie) {
    return kPGain * e + kDGain * de + kIGain * ie;
  }};

  while (!state_cur_) {
    RCLCPP_WARN(this->get_logger(), "Odometry not received yet, waiting...");
    rclcpp::sleep_for(1s);
  }

  double e_x{}, e_y{}, de_x{}, de_y{}, ie_x{0.0}, ie_y{0.0}, ds{};
  Twist v_d{};
  while (true) {
    e_x = x_goal - state_cur_.value().x;
    e_y = y_goal - state_cur_.value().y;
    de_x = -state_cur_.value().dx; // dx_goal = 0
    de_y = -state_cur_.value().dy; // dy_goal = 0
    ds = std::sqrt(e_x * e_x + e_y * e_y);

    if (ds <= kPositionTolerance) {
      twist_pub_->publish(Twist{});
      RCLCPP_INFO(this->get_logger(), "Reached waypoint.");
      return true;
    }

    v_d.linear.x = pid_step(e_x, de_x, ie_x);
    v_d.linear.y = pid_step(e_y, de_y, ie_y);
    ie_x += std::chrono::duration<double>{kControlCycle}.count() * e_x;
    ie_y += std::chrono::duration<double>{kControlCycle}.count() * e_y;
    twist_pub_->publish(v_d);

    rclcpp::sleep_for(kControlCycle);
  }
}

bool DistanceController::followTrajectory(const PoseArray &goals) {
  RCLCPP_INFO(this->get_logger(), "Received new goal trajectory.");

  for (const auto &goal : goals.poses) {
    if (!goToPoint(goal)) {
      RCLCPP_WARN(this->get_logger(), "Failed to follow goal trajectory.");
      return false;
    }
  }
  RCLCPP_INFO(this->get_logger(), "Reached goal.");
  return true;
}

} // namespace DistanceController

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node{std::make_shared<DistanceController::DistanceController>()};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto executor_thread{std::thread([&executor]() { executor.spin(); })};

  const auto goals{[] {
    PoseArray goal_poses;
    Pose goal_pose{};

    goal_pose.position.x = 1.0;
    goal_poses.poses.push_back(goal_pose);

    goal_pose.position.x = 2.0;
    goal_poses.poses.push_back(goal_pose);

    goal_pose.position.x = 3.0;
    goal_poses.poses.push_back(goal_pose);

    return goal_poses;
  }()};
  node->followTrajectory(goals);

  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  rclcpp::shutdown();
}