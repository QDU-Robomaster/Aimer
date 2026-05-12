#pragma once

/**
 * @file Aimer.hpp
 * @brief Aimer 模块的公开接口和 topic 载荷声明。
 */

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: ballistic aimer with TinyMPC gimbal plan output
constructor_args:
  cfg:
    yaw_offset: -1.0
    pitch_offset: -1.4
    yaw_rate_threshold: 2.0
    default_bullet_speed: 23.0
    min_valid_bullet_speed: 14.0
    auto_fire: true
    image_to_now_s: 0.0
    vision_to_command_delay_s: 0.0
    command_transport_delay_s: 0.0
    gimbal_response_delay_s: 0.0
    fire_delay_s: 0.0
    low_speed_extra_predict_s: 0.015
    high_speed_extra_predict_s: 0.03
    armor_width_m: 0.135
    armor_height_m: 0.055
    bullet_spread_m: 0.015
    min_fire_threshold: 0.003
    max_fire_threshold: 0.05
    enable_pitch_fire_gate: false
    enable_mpc_plan: true
    mpc_fire_thresh: 0.05
    max_yaw_acc: 100.0
    q_yaw_pos: 50.0
    q_yaw_vel: 1.0
    r_yaw_acc: 1.0
    max_pitch_acc: 100.0
    q_pitch_pos: 50.0
    q_pitch_vel: 1.0
    r_pitch_acc: 1.0
    mpc_max_iter: 10
template_args: []
required_hardware: []
depends:
  - qdu-future/ArmorTracker
=== END MANIFEST === */
// clang-format on

#include <Eigen/Dense>
#include <atomic>
#include <cstdint>

#include "ArmorTracker.hpp"
#include "ArmorTrackerTarget.hpp"
#include "GimbalPlan.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "mutex.hpp"
#include "tinympc/tiny_api.hpp"

/**
 * @brief 发布到 tracker/send 的云台命令载荷。
 */
struct AimerSend
{
  /// 当前命令是否请求开火。
  bool is_fire{};
  /// tracker 相机坐标系下的选中瞄点。
  LibXR::Position<double> position{};
  /// 目标 yaw 角速度，单位 rad/s。
  double v_yaw{};
  /// 命令 pitch，单位 rad。
  double pitch{};
  /// 命令 yaw，单位 rad。
  double yaw{};
  /// 预留线速度命令，当前保持为 0。
  Eigen::Matrix<double, 3, 1> cmd_vel_linear = Eigen::Matrix<double, 3, 1>::Zero();
  /// 预留角速度命令，当前保持为 0。
  Eigen::Matrix<double, 3, 1> cmd_vel_angular = Eigen::Matrix<double, 3, 1>::Zero();
};

/**
 * @brief 选择目标装甲板、解算弹道 yaw/pitch，并发布云台命令。
 */
class Aimer : public LibXR::Application
{
 public:
  /**
   * @brief 由 xrobot YAML 生成的运行时配置。
   */
  struct Config
  {
    /// 施加到弹道命令上的固定 yaw 偏置，单位 deg。
    double yaw_offset{-1.0};
    /// 施加到弹道命令上的固定 pitch 偏置，单位 deg。
    double pitch_offset{-1.4};
    /// 低速与旋转策略的 yaw 角速度阈值，单位 rad/s。
    double yaw_rate_threshold{2.0};
    /// 弹速异常时使用的默认弹速，单位 m/s。
    double default_bullet_speed{23.0};
    /// 可接受的最小裁判系统弹速，单位 m/s。
    double min_valid_bullet_speed{14.0};
    /// 是否启用基于实测云台姿态的自动开火门控。
    bool auto_fire{true};
    /// 从图像曝光到当前处理时刻的估计延迟。
    double image_to_now_s{0.0};
    /// 从视觉输出到命令生成的估计延迟。
    double vision_to_command_delay_s{0.0};
    /// 命令传输估计延迟。
    double command_transport_delay_s{0.0};
    /// 云台响应估计延迟。
    double gimbal_response_delay_s{0.0};
    /// 从开火命令到弹丸出膛的估计延迟。
    double fire_delay_s{0.0};
    /// 低速目标的额外预测延迟。
    double low_speed_extra_predict_s{0.015};
    /// yaw 角速度超过阈值时的额外预测延迟。
    double high_speed_extra_predict_s{0.03};
    /// 动态开火阈值使用的装甲板宽度，单位 m。
    double armor_width_m{0.135};
    /// 动态开火阈值使用的装甲板高度，单位 m。
    double armor_height_m{0.055};
    /// 弹道散布裕量，单位 m。
    double bullet_spread_m{0.015};
    /// 最小角度开火阈值，单位 rad。
    double min_fire_threshold{0.003};
    /// 最大角度开火阈值，单位 rad。
    double max_fire_threshold{0.05};
    /// 是否把 pitch 对准纳入自动开火门控。
    bool enable_pitch_fire_gate{false};
    /// 是否启用 TinyMPC 云台计划。
    bool enable_mpc_plan{true};
    /// 允许开火的最大 MPC 跟踪误差，单位 rad。
    double mpc_fire_thresh{0.05};
    /// TinyMPC yaw 加速度约束，单位 rad/s^2。
    double max_yaw_acc{100.0};
    /// TinyMPC yaw 位置代价。
    double q_yaw_pos{50.0};
    /// TinyMPC yaw 速度代价。
    double q_yaw_vel{1.0};
    /// TinyMPC yaw 加速度代价。
    double r_yaw_acc{1.0};
    /// TinyMPC pitch 加速度约束，单位 rad/s^2。
    double max_pitch_acc{100.0};
    /// TinyMPC pitch 位置代价。
    double q_pitch_pos{50.0};
    /// TinyMPC pitch 速度代价。
    double q_pitch_vel{1.0};
    /// TinyMPC pitch 加速度代价。
    double r_pitch_acc{1.0};
    /// TinyMPC ADMM 最大迭代次数。
    int mpc_max_iter{10};
  };

  /**
   * @brief 创建 Aimer 模块并注册 topic 回调。
   */
  Aimer(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, Config cfg);

  /**
   * @brief LibXR 应用要求的周期监控钩子。
   */
  void OnMonitor() override {}

 private:
  /**
   * @brief 根据裁判系统 topic 更新最新弹速。
   */
  void BulletSpeedCallback(float bullet_speed_msg);
  /**
   * @brief 更新最新实测云台旋转。
   */
  void GimbalRotationCallback(LibXR::Quaternion<float> gimbal_rotation_msg);
  /**
   * @brief 处理一帧 tracker 目标消息并发布所有 Aimer 输出。
   */
  void TargetCallback(const ArmorTrackerTarget& target_msg);
  /**
   * @brief 根据命令稳定性和云台对准情况评估自动开火门控。
   */
  bool ShouldAutoFire(const Eigen::Vector3d& target_xyz, double selected_view_angle,
                      bool shootable, double yaw,
                      double pitch);
  /**
   * @brief 初始化 yaw 和 pitch TinyMPC 求解器。
   */
  void SetupGimbalPlanSolvers();
  /**
   * @brief 尝试为当前目标构建 TinyMPC 云台计划。
   */
  bool BuildMpcGimbalPlan(const ArmorTrackerTarget& target_msg,
                          double delay_time, double bullet_speed, bool fire);
  /**
   * @brief 在 TinyMPC 关闭或不可用时构建直接 yaw/pitch 计划。
   */
  void BuildFiniteDifferenceGimbalPlan(const ArmorTrackerTarget& target_msg,
                                       bool control, bool fire, double yaw,
                                       double pitch);
  /**
   * @brief 在 TinyMPC 和直接云台计划之间选择。
   */
  void BuildGimbalPlan(const ArmorTrackerTarget& target_msg, double delay_time,
                       bool control, bool fire, double yaw, double pitch,
                       double bullet_speed);
  /**
   * @brief 清理与上一目标相关的规划器状态。
   */
  void ResetGimbalPlanHistory();

 private:
  Config cfg_{};
  std::atomic<double> bullet_speed_{23.0};
  int lock_id_{-1};
  ArmorNumber last_target_id_{ArmorNumber::INVALID};
  bool has_last_command_{false};
  double last_command_yaw_{0.0};
  double last_command_pitch_{0.0};
  bool has_gimbal_rotation_{false};
  LibXR::Quaternion<double> gimbal_rotation_{1.0, 0.0, 0.0, 0.0};
  bool planner_ready_{false};
  bool last_plan_mpc_{false};
  TinySolver* yaw_solver_{nullptr};
  TinySolver* pitch_solver_{nullptr};
  mutable LibXR::Mutex gimbal_rotation_lock_{};

  AimerSend send_msg_{};
  GimbalPlan gimbal_plan_msg_{};

  LibXR::Topic::Domain tracker_domain_ = LibXR::Topic::Domain("tracker");
  LibXR::Topic fire_notify_topic_ =
      LibXR::Topic("fire_notify", sizeof(uint8_t), &tracker_domain_);
  LibXR::Topic send_topic_ =
      LibXR::Topic("send", sizeof(AimerSend), &tracker_domain_);
  LibXR::Topic gimbal_plan_topic_ =
      LibXR::Topic("gimbal_plan", sizeof(GimbalPlan), &tracker_domain_);
};

#include "AimerImpl.hpp"
