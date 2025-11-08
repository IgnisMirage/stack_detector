#include "stack_detector/stack_detector_node.hpp"
#include <cmath>

namespace stack_detector
{

StackDetectorNode::StackDetectorNode(const rclcpp::NodeOptions & options)
: Node("stack_detector_node", options),
  current_state_(RobotState::NORMAL),
  stuck_start_time_(this->now()),
  recovery_start_time_(this->now()),
  first_stuck(false)
{
  // パラメータの宣言
  this->declare_parameter<double>("velocity_threshold", 0.05);
  this->declare_parameter<double>("stuck_time", 5.0);
  this->declare_parameter<double>("recovery_time", 10.0);
  this->declare_parameter<double>("check_rate", 10.0);

  // パラメータの取得
  velocity_threshold_ = this->get_parameter("velocity_threshold").as_double();
  stuck_time_ = this->get_parameter("stuck_time").as_double();
  recovery_time_ = this->get_parameter("recovery_time").as_double();
  double check_rate = this->get_parameter("check_rate").as_double();

  // サブスクライバーの作成
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10,
    std::bind(&StackDetectorNode::cmdVelCommandCallback, this, std::placeholders::_1));

  cmd_vel_safe_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel/safty_limiter",
    10,
    std::bind(&StackDetectorNode::cmdVelSafeCallback, this, std::placeholders::_1));

  // パブリッシャーの作成
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/nav_status", 10);

  // タイマーの作成
  auto timer_period = std::chrono::duration<double>(1.0 / check_rate);
  timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&StackDetectorNode::checkStuckCondition, this));

  RCLCPP_INFO(this->get_logger(), "StackDetectorNode initialized");
}

void StackDetectorNode::cmdVelCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_ = *msg;
}

void StackDetectorNode::cmdVelSafeCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_safe_ = *msg;
}

bool StackDetectorNode::isStuckCondition() const
{
  // cmd_velに速度指令があるが、safe_cmd_velが0の場合はスタックとみなす
  if (std::abs(cmd_vel_.linear.x) > velocity_threshold_ || 
      std::abs(cmd_vel_.angular.z) > velocity_threshold_) {
    if (std::abs(cmd_vel_safe_.linear.x) < velocity_threshold_ && 
        std::abs(cmd_vel_safe_.angular.z) < velocity_threshold_) {
      return true;
    }
  }
  return false;
}

void StackDetectorNode::checkStuckCondition()
{
  rclcpp::Time current_time = this->now();
  bool is_stuck = isStuckCondition();

  switch (current_state_) {
    case RobotState::NORMAL:
      if (is_stuck) {
        if (!first_stuck) {
          stuck_start_time_ = current_time;
          first_stuck = true;
        } else {
          double elapsed = (current_time - stuck_start_time_).seconds();
          if (elapsed >= stuck_time_) {
            current_state_ = RobotState::RECOVERY;
            recovery_start_time_ = current_time;
            first_stuck = false;
          }
        }
      } else {
        first_stuck = false;
      }
      break;

    case RobotState::RECOVERY:
      if (!is_stuck) {
        double elapsed = (current_time - recovery_start_time_).seconds();
        if (elapsed >= recovery_time_) {
          current_state_ = RobotState::NORMAL;
        }
      } else {
        recovery_start_time_ = current_time;
      }
      break;
  }
  // スタック状態のパブリッシュ
  auto status_msg = std_msgs::msg::String();
  switch (current_state_) {
    case RobotState::NORMAL:
      status_msg.data = "NORMAL";
      break;
    case RobotState::RECOVERY:
      status_msg.data = "RECOVERY";
      break;  
  }
  status_pub_->publish(status_msg);
}

}  // namespace stack_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stack_detector::StackDetectorNode)
