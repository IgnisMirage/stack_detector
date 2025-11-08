#include "stack_detector/stack_detector_node.hpp"
#include <cmath>

namespace stack_detector
{

StackDetectorNode::StackDetectorNode(const rclcpp::NodeOptions & options)
: Node("stack_detector_node", options),
  current_state_(RobotState::NORMAL),
  stuck_start_time_(this->now()),
  recovery_start_time_(this->now()),
  normal_start_time_(this->now())
{
  // パラメータの宣言
  this->declare_parameter<std::string>("cmd_vel_command_topic", "/cmd_vel_command");
  this->declare_parameter<std::string>("cmd_vel_safe_topic", "/cmd_vel_safe");
  this->declare_parameter<std::string>("stuck_state_topic", "/stuck_state");
  this->declare_parameter<std::string>("recovery_status_topic", "/recovery_status");
  this->declare_parameter<double>("velocity_threshold", 0.01);
  this->declare_parameter<double>("stuck_timeout", 5.0);
  this->declare_parameter<double>("recovery_timeout", 10.0);
  this->declare_parameter<double>("normal_timeout", 3.0);
  this->declare_parameter<double>("check_rate", 10.0);

  // パラメータの取得
  std::string cmd_vel_command_topic = this->get_parameter("cmd_vel_command_topic").as_string();
  std::string cmd_vel_safe_topic = this->get_parameter("cmd_vel_safe_topic").as_string();
  std::string stuck_state_topic = this->get_parameter("stuck_state_topic").as_string();
  std::string recovery_status_topic = this->get_parameter("recovery_status_topic").as_string();
  velocity_threshold_ = this->get_parameter("velocity_threshold").as_double();
  stuck_timeout_ = this->get_parameter("stuck_timeout").as_double();
  recovery_timeout_ = this->get_parameter("recovery_timeout").as_double();
  normal_timeout_ = this->get_parameter("normal_timeout").as_double();
  double check_rate = this->get_parameter("check_rate").as_double();

  // サブスクライバーの作成
  cmd_vel_command_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_command_topic,
    10,
    std::bind(&StackDetectorNode::cmdVelCommandCallback, this, std::placeholders::_1));

  cmd_vel_safe_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_safe_topic,
    10,
    std::bind(&StackDetectorNode::cmdVelSafeCallback, this, std::placeholders::_1));

  // パブリッシャーの作成
  stuck_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(stuck_state_topic, 10);
  recovery_status_pub_ = this->create_publisher<std_msgs::msg::String>(recovery_status_topic, 10);

  // タイマーの作成
  auto timer_period = std::chrono::duration<double>(1.0 / check_rate);
  timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&StackDetectorNode::checkStuckCondition, this));

  RCLCPP_INFO(this->get_logger(), "Stack Detector Node started");
  RCLCPP_INFO(this->get_logger(), "Monitoring: %s and %s",
              cmd_vel_command_topic.c_str(), cmd_vel_safe_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing stuck state to: %s", stuck_state_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing recovery status to: %s",
              recovery_status_topic.c_str());
}

void StackDetectorNode::cmdVelCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_command_ = *msg;
}

void StackDetectorNode::cmdVelSafeCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_safe_ = *msg;
}

bool StackDetectorNode::isVelocityZero(const geometry_msgs::msg::Twist & twist_msg) const
{
  double linear_speed = std::sqrt(
    twist_msg.linear.x * twist_msg.linear.x +
    twist_msg.linear.y * twist_msg.linear.y +
    twist_msg.linear.z * twist_msg.linear.z);

  double angular_speed = std::sqrt(
    twist_msg.angular.x * twist_msg.angular.x +
    twist_msg.angular.y * twist_msg.angular.y +
    twist_msg.angular.z * twist_msg.angular.z);

  return (linear_speed < velocity_threshold_) && (angular_speed < velocity_threshold_);
}

bool StackDetectorNode::isVelocityNonzero(const geometry_msgs::msg::Twist & twist_msg) const
{
  return !isVelocityZero(twist_msg);
}

bool StackDetectorNode::isStuckCondition() const
{
  // 速度司令が0より大きく、かつ安全機能後の速度が0
  bool command_moving = isVelocityNonzero(cmd_vel_command_);
  bool safe_stopped = isVelocityZero(cmd_vel_safe_);

  return command_moving && safe_stopped;
}

void StackDetectorNode::checkStuckCondition()
{
  rclcpp::Time current_time = this->now();
  bool is_stuck = isStuckCondition();

  // 状態マシン
  switch (current_state_) {
    case RobotState::NORMAL:
      if (is_stuck) {
        if (stuck_start_time_.nanoseconds() == 0 ||
            stuck_start_time_ == this->now()) {
          stuck_start_time_ = current_time;
          RCLCPP_INFO(this->get_logger(), "Potential stuck condition detected, monitoring...");
        } else {
          // スタック条件が継続している時間をチェック
          double elapsed = (current_time - stuck_start_time_).seconds();
          if (elapsed >= stuck_timeout_) {
            transitionToStuck();
          }
        }
      } else {
        // スタック条件が解除されたらタイマーをリセット
        stuck_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
      break;

    case RobotState::STUCK:
      // スタック状態からリカバリーモードへ自動遷移
      transitionToRecovery();
      break;

    case RobotState::RECOVERY:
      if (is_stuck) {
        // リカバリー中に再度スタックが検出された場合
        RCLCPP_WARN(this->get_logger(), "Stuck condition detected again during recovery!");
        normal_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        // リカバリータイマーをリセット
        recovery_start_time_ = current_time;
      } else {
        // スタック条件が解除されている
        if (normal_start_time_.nanoseconds() == 0) {
          normal_start_time_ = current_time;
          RCLCPP_INFO(this->get_logger(), "No stuck condition, starting normal state timer...");
        } else {
          // スタックしていない時間をチェック
          double elapsed_normal = (current_time - normal_start_time_).seconds();
          if (elapsed_normal >= normal_timeout_) {
            transitionToNormal();
          }
        }
      }

      // リカバリーモードのタイムアウトチェック
      if (recovery_start_time_.nanoseconds() != 0) {
        double elapsed_recovery = (current_time - recovery_start_time_).seconds();
        if (elapsed_recovery >= recovery_timeout_) {
          RCLCPP_INFO(this->get_logger(), "Recovery timeout (%.1fs) reached",
                      recovery_timeout_);
          // タイムアウト後も継続的にリカバリーを試みる場合はここに処理を追加
        }
      }
      break;
  }
}

void StackDetectorNode::transitionToStuck()
{
  current_state_ = RobotState::STUCK;
  RCLCPP_WARN(this->get_logger(), "=== STUCK DETECTED ===");

  // スタック状態をパブリッシュ
  auto stuck_msg = std_msgs::msg::Bool();
  stuck_msg.data = true;
  stuck_state_pub_->publish(stuck_msg);

  auto status_msg = std_msgs::msg::String();
  status_msg.data = "STUCK";
  recovery_status_pub_->publish(status_msg);

  stuck_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void StackDetectorNode::transitionToRecovery()
{
  current_state_ = RobotState::RECOVERY;
  recovery_start_time_ = this->now();
  normal_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  RCLCPP_INFO(this->get_logger(), "=== ENTERING RECOVERY MODE ===");

  // リカバリー状態をパブリッシュ
  auto status_msg = std_msgs::msg::String();
  status_msg.data = "RECOVERY";
  recovery_status_pub_->publish(status_msg);

  // スタック状態をfalseに設定
  auto stuck_msg = std_msgs::msg::Bool();
  stuck_msg.data = false;
  stuck_state_pub_->publish(stuck_msg);
}

void StackDetectorNode::transitionToNormal()
{
  current_state_ = RobotState::NORMAL;
  recovery_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  normal_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  RCLCPP_INFO(this->get_logger(), "=== RETURNING TO NORMAL STATE ===");

  // 平常状態をパブリッシュ
  auto status_msg = std_msgs::msg::String();
  status_msg.data = "NORMAL";
  recovery_status_pub_->publish(status_msg);

  // スタック状態をfalseに設定
  auto stuck_msg = std_msgs::msg::Bool();
  stuck_msg.data = false;
  stuck_state_pub_->publish(stuck_msg);
}

}  // namespace stack_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stack_detector::StackDetectorNode)
