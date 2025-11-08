#ifndef STACK_DETECTOR__STACK_DETECTOR_NODE_HPP_
#define STACK_DETECTOR__STACK_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>

namespace stack_detector
{
enum class RobotState
{
  NORMAL = 0,   // 通常状態
  RECOVERY = 1, // リカバリー状態
};

class StackDetectorNode : public rclcpp::Node
{
public:
  explicit StackDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StackDetectorNode() = default;

private:
  void cmdVelCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdVelSafeCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void checkStuckCondition();
  bool isStuckCondition() const;

  // パラメータ
  double velocity_threshold_;
  double stuck_time_;
  double recovery_time_;

  // サブスクライバー
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_safe_sub_;

  // パブリッシャー
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // タイマー
  rclcpp::TimerBase::SharedPtr timer_;

  // 内部状態
  geometry_msgs::msg::Twist cmd_vel_;
  geometry_msgs::msg::Twist cmd_vel_safe_;
  RobotState current_state_;
  rclcpp::Time stuck_start_time_;
  rclcpp::Time recovery_start_time_;
  rclcpp::Time normal_start_time_;

  bool first_stuck = false;
};

}  // namespace stack_detector

#endif  // STACK_DETECTOR__STACK_DETECTOR_NODE_HPP_
