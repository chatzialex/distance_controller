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
  constexpr static char kCommandVelocityTopicName[]{"cmd_vel"};
  constexpr static char kNodeName[]{"distance_controller"};
  constexpr static char kOdometryTopicName[]{"/odometry/filtered"};

  constexpr static double kPositionTolerance{0.1}; // [m]
  constexpr static auto kControlCycle{10ms};
  constexpr static double kPGain{/* TODO */};
  constexpr static double kIGain{/* TODO */};
  constexpr static double kDGain{/* TODO */};

  void odomSubCb(const std::shared_ptr<const Odometry> msg);
  bool goToPoint(const Pose &goal);

  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_{};
  rclcpp::Publisher<Twist>::SharedPtr twist_pub_{};

  std::optional<double> x_cur_{std::nullopt};
  std::optional<double> y_cur_{std::nullopt};
};

void DistanceController::odomSubCb(const std::shared_ptr<const Odometry> msg) {
  x_cur_ = msg->pose.pose.position.x;
  y_cur_ = msg->pose.pose.position.y;
}

bool DistanceController::goToPoint(const Pose &goal) {
  const auto x_goal{goal.position.x};
  const auto y_goal{goal.position.y};

  RCLCPP_INFO(this->get_logger(), "Received new goal: x=%f y=%f", x_goal,
              y_goal);

  while (!x_cur_ || !y_cur_) {
    RCLCPP_WARN(this->get_logger(), "Odometry not received yet, waiting...");
    rclcpp::sleep_for(1s);
  }

  double dx{}, dy{}, ds{};
  while (true) {
    dx = x_goal - x_cur_.value();
    dy = y_goal - y_cur_.value();
    ds = std::sqrt(dx * dx + dy * dy);

    if (ds <= kPositionTolerance) {
      twist_pub_->publish(Twist{});
      RCLCPP_INFO(this->get_logger(), "Reached waypoint.");
      return true;
    }

    /* TODO: implement control cycle*/

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