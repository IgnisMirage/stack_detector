#ifndef STACK_DETECTOR__STACK_DETECTOR_NODE_HPP_
#define STACK_DETECTOR__STACK_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>

namespace stack_detector
{

/**
 * @brief ロボットの状態を表す列挙型
 */
enum class RobotState
{
  NORMAL = 0,   // 平常状態
  STUCK = 1,    // スタック検出状態
  RECOVERY = 2  // リカバリーモード
};

/**
 * @brief スタック検出ノード
 *
 * 2つのcmd_velトピックを監視し、スタック状態を検出します:
 * - cmd_vel_command: 元の速度司令
 * - cmd_vel_safe: 安全機能が適用された後の速度司令
 *
 * スタック条件:
 * - 速度司令が0より大きい（ロボットが動こうとしている）
 * - かつ、安全機能後のcmd_velが0（安全機能により停止）
 *
 * スタック検出時:
 * 1. スタック状態をパブリッシュ
 * 2. リカバリーモードに入る
 * 3. 一定時間スタックが検出されなければ、平常状態に戻る
 */
class StackDetectorNode : public rclcpp::Node
{
public:
  /**
   * @brief コンストラクタ
   */
  explicit StackDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief デストラクタ
   */
  ~StackDetectorNode() = default;

private:
  /**
   * @brief 速度司令のコールバック
   * @param msg 速度司令メッセージ
   */
  void cmdVelCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief 安全機能後の速度司令のコールバック
   * @param msg 安全機能後の速度司令メッセージ
   */
  void cmdVelSafeCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief 速度が0かどうかを判定
   * @param twist_msg Twistメッセージ
   * @return true: 速度が0, false: 速度が0より大きい
   */
  bool isVelocityZero(const geometry_msgs::msg::Twist & twist_msg) const;

  /**
   * @brief 速度が0より大きいかを判定
   * @param twist_msg Twistメッセージ
   * @return true: 速度が0より大きい, false: 速度が0
   */
  bool isVelocityNonzero(const geometry_msgs::msg::Twist & twist_msg) const;

  /**
   * @brief スタック条件を満たしているかチェック
   * @return true: スタック条件を満たす, false: スタック条件を満たさない
   */
  bool isStuckCondition() const;

  /**
   * @brief 定期的にスタック状態をチェック
   */
  void checkStuckCondition();

  /**
   * @brief スタック状態へ遷移
   */
  void transitionToStuck();

  /**
   * @brief リカバリーモードへ遷移
   */
  void transitionToRecovery();

  /**
   * @brief 平常状態へ遷移
   */
  void transitionToNormal();

  // パラメータ
  double velocity_threshold_;
  double stuck_timeout_;
  double recovery_timeout_;
  double normal_timeout_;

  // サブスクライバー
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_safe_sub_;

  // パブリッシャー
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stuck_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_status_pub_;

  // タイマー
  rclcpp::TimerBase::SharedPtr timer_;

  // 内部状態
  geometry_msgs::msg::Twist cmd_vel_command_;
  geometry_msgs::msg::Twist cmd_vel_safe_;
  RobotState current_state_;
  rclcpp::Time stuck_start_time_;
  rclcpp::Time recovery_start_time_;
  rclcpp::Time normal_start_time_;
};

}  // namespace stack_detector

#endif  // STACK_DETECTOR__STACK_DETECTOR_NODE_HPP_
