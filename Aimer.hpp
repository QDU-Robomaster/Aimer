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
#include <array>
#include <atomic>
#include <cstdint>
#include <fstream>
#include <string>

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
   * @brief 用于延迟选择和审计输出的目标运动粗分类。
   */
  enum class Strategy : uint8_t
  {
    /// tracker 当前没有有效目标。
    LOST = 0,
    /// 目标 yaw 角速度未超过低速阈值。
    LOW_SPEED = 1,
    /// 目标 yaw 角速度超过低速阈值。
    MEDIUM_SPIN = 2,
    /// 目标为前哨站。
    OUTPOST = 3,
  };

  /**
   * @brief 当前装甲板被选中的原因。
   */
  enum class SelectReason : uint8_t
  {
    /// 没有选中装甲板。
    NONE = 0,
    /// 在当前策略下选择水平距离最近的装甲板。
    NEAREST_FRONT = 1,
  };

  /**
   * @brief 选中装甲板锁定发生变化的原因。
   */
  enum class SwitchReason : uint8_t
  {
    /// 没有发生切换。
    NONE = 0,
    /// tracker 目标身份发生变化。
    NEW_TARGET = 1,
    /// 当前预测状态下最近装甲板发生变化。
    NEAREST_CHANGED = 2,
  };

  /**
   * @brief 当前帧自动开火门控的最终结果。
   */
  enum class FireReason : uint8_t
  {
    /// 配置中关闭了自动开火。
    DISABLED = 0,
    /// 所有开火门控通过。
    OK = 1,
    /// tracker 目标无效。
    NO_TRACK = 2,
    /// 尚未收到当前云台姿态。
    NO_GIMBAL = 3,
    /// 连续两帧命令变化超过稳定性门限。
    COMMAND_UNSTABLE = 4,
    /// 实测云台角度未对准命令角。
    GIMBAL_NOT_ALIGNED = 5,
    /// 当前策略认为选中装甲板不可打。
    NOT_SHOOTABLE = 6,
    /// 弹道 pitch 方程无有效解。
    BALLISTIC_UNSOLVABLE = 7,
  };

  /**
   * @brief 预览和离线检查使用的调试弹道载荷。
   */
  struct AimerTrajectory
  {
    /// 固定弹道采样点数量上限。
    static constexpr uint8_t MAX_POINTS = 32;

    /// 来源 tracker 帧时间戳，单位 us。
    uint64_t image_timestamp_us{0};
    /// 弹道载荷是否包含有效解。
    bool valid{false};
    /// 对应命令是否请求开火。
    bool fire{false};
    /// 弹道解算是否收敛。
    bool converged{false};
    /// points 中实际使用的条目数。
    uint8_t point_count{0};
    /// 选中的装甲板面索引。
    uint8_t selected_armor_index{0};
    /// 选中的 tracker 目标 id。
    ArmorNumber target_id{ArmorNumber::INVALID};
    /// 解算使用的弹速，单位 m/s。
    double bullet_speed{0.0};
    /// 固定视觉到命令预测延迟，单位 s。
    double delay_time_s{0.0};
    /// 估计弹丸飞行时间，单位 s。
    double fly_time_s{0.0};
    /// 命令 yaw，单位 rad。
    double yaw{0.0};
    /// 命令 pitch，单位 rad。
    double pitch{0.0};
    /// tracker 相机坐标系下的最终瞄点。
    LibXR::Position<double> aim_point{};
    /// 本地发射坐标系下的弹道采样点。
    std::array<LibXR::Position<double>, MAX_POINTS> points{};
  };

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
   * @brief 在线监控使用的紧凑逐帧状态载荷。
   */
  struct AimerMetrics
  {
    /// Aimer 回调的单调递增帧序号。
    uint64_t frame_index{0};
    /// tracker 是否报告有效目标。
    bool target_tracking{false};
    /// 当前帧是否有有效命令。
    bool valid{false};
    /// 弹道解算是否收敛。
    bool converged{false};
    /// 弹道细化迭代次数。
    uint32_t iteration_count{0};
    /// 选中的装甲板面索引。
    uint32_t selected_armor_index{0};
    /// 选中的 tracker 目标 id。
    ArmorNumber target_id{ArmorNumber::INVALID};
    /// Aimer 回调处理耗时，单位 ms。
    double latency_ms{0.0};
    /// 当前帧使用的弹速，单位 m/s。
    double bullet_speed{0.0};
    /// 固定预测延迟，单位 s。
    double delay_time_s{0.0};
    /// 估计弹丸飞行时间，单位 s。
    double fly_time_s{0.0};
    /// 固定延迟、开火延迟和飞行时间之和，单位 s。
    double total_hit_delay_s{0.0};
    /// 命令 yaw，单位 rad。
    double yaw{0.0};
    /// 命令 pitch，单位 rad。
    double pitch{0.0};
    /// 动态 yaw 开火阈值，单位 rad。
    double fire_thres_yaw{0.0};
    /// 动态 pitch 开火阈值，单位 rad。
    double fire_thres_pitch{0.0};
    /// 稳定性门控使用的连续命令 yaw 差值。
    double command_error_yaw{0.0};
    /// 稳定性门控使用的连续命令 pitch 差值。
    double command_error_pitch{0.0};
    /// 实测云台 yaw 与命令的误差，单位 rad。
    double gimbal_error_yaw{0.0};
    /// 实测云台 pitch 与命令的误差，单位 rad。
    double gimbal_error_pitch{0.0};
    /// 当前目标策略。
    Strategy strategy{Strategy::LOST};
    /// 装甲板选择原因。
    SelectReason selected_reason{SelectReason::NONE};
    /// 锁定切换原因。
    SwitchReason switch_reason{SwitchReason::NONE};
    /// 自动开火门控结果。
    FireReason fire_reason{FireReason::NO_TRACK};
    /// 当前云台计划是否由 TinyMPC 生成。
    bool planner_mpc{false};
    /// 当前帧是否请求开火。
    bool is_fire{false};
  };

  /**
   * @brief 离线回放和审计使用的完整逐帧决策载荷。
   */
  struct AimerDecision
  {
    /// Aimer 回调的单调递增帧序号。
    uint64_t frame_id{0};
    /// 来源 tracker 帧时间戳，单位 us。
    uint64_t image_timestamp_us{0};
    /// Aimer 回调接收时刻，steady clock 单位 us。
    uint64_t aimer_receive_time_us{0};
    /// 预测目标时刻，单位 us。
    uint64_t predict_time_us{0};
    /// 预计命中时刻，单位 us。
    uint64_t expected_hit_time_us{0};
    /// tracker 是否报告有效目标。
    bool target_tracking{false};
    /// 当前帧是否有有效命令。
    bool valid{false};
    /// 弹道解算是否收敛。
    bool converged{false};
    /// 参与选择的候选装甲板数量。
    uint8_t candidate_count{0};
    /// 选中的装甲板面索引。
    uint8_t selected_armor_index{0};
    /// 选中的 tracker 目标 id。
    ArmorNumber target_id{ArmorNumber::INVALID};
    /// 当前目标策略。
    Strategy strategy{Strategy::LOST};
    /// 装甲板选择原因。
    SelectReason selected_reason{SelectReason::NONE};
    /// 锁定切换原因。
    SwitchReason switch_reason{SwitchReason::NONE};
    /// 自动开火门控结果。
    FireReason fire_reason{FireReason::NO_TRACK};
    /// 固定视觉到命令预测延迟，单位 s。
    double fixed_delay_s{0.0};
    /// 从开火命令到弹丸出膛的估计延迟，单位 s。
    double fire_delay_s{0.0};
    /// 估计弹丸飞行时间，单位 s。
    double fly_time_s{0.0};
    /// 固定延迟、开火延迟和飞行时间之和，单位 s。
    double total_hit_delay_s{0.0};
    /// tracker 相机坐标系下选中瞄点的 x。
    double selected_x{0.0};
    /// tracker 相机坐标系下选中瞄点的 y。
    double selected_y{0.0};
    /// tracker 相机坐标系下选中瞄点的 z。
    double selected_z{0.0};
    /// 选中装甲板面的 yaw，单位 rad。
    double selected_yaw{0.0};
    /// 选中装甲板相对整车中心方位的视角。
    double selected_view_angle{0.0};
    /// 选中装甲板是否满足当前策略的正面门控。
    bool selected_front_facing{false};
    /// 选中装甲板是否允许开火。
    bool shootable{false};
    /// 原始弹道命令 yaw，单位 rad。
    double command_yaw{0.0};
    /// 原始弹道命令 pitch，单位 rad。
    double command_pitch{0.0};
    /// 规划器输出点的目标 yaw，单位 rad。
    double target_yaw{0.0};
    /// 规划器输出点的目标 pitch，单位 rad。
    double target_pitch{0.0};
    /// 规划后的 yaw 命令，单位 rad。
    double planned_yaw{0.0};
    /// 规划后的 pitch 命令，单位 rad。
    double planned_pitch{0.0};
    /// 规划后的 yaw 速度前馈，单位 rad/s。
    double planned_yaw_vel{0.0};
    /// 规划后的 pitch 速度前馈，单位 rad/s。
    double planned_pitch_vel{0.0};
    /// 规划后的 yaw 加速度前馈，单位 rad/s^2。
    double planned_yaw_acc{0.0};
    /// 规划后的 pitch 加速度前馈，单位 rad/s^2。
    double planned_pitch_acc{0.0};
    /// 发布的计划是否由 TinyMPC 生成。
    bool mpc_used{false};
    /// 最终发布命令是否允许开火。
    bool fire_allowed{false};
    /// 动态 yaw 开火阈值，单位 rad。
    double fire_thres_yaw{0.0};
    /// 动态 pitch 开火阈值，单位 rad。
    double fire_thres_pitch{0.0};
    /// 连续命令 yaw 差值，单位 rad。
    double command_error_yaw{0.0};
    /// 连续命令 pitch 差值，单位 rad。
    double command_error_pitch{0.0};
    /// 实测云台 yaw 与命令的误差，单位 rad。
    double actual_gimbal_error_yaw{0.0};
    /// 实测云台 pitch 与命令的误差，单位 rad。
    double actual_gimbal_error_pitch{0.0};
  };

  /**
   * @brief 每次接受开火命令时发布的事件载荷。
   */
  struct AimerShotEvent
  {
    /// 单调递增的 shot 事件序号。
    uint64_t shot_id{0};
    /// 产生该 shot 事件的 Aimer 帧序号。
    uint64_t frame_id{0};
    /// 来源 tracker 帧时间戳，单位 us。
    uint64_t image_timestamp_us{0};
    /// Aimer 命令时刻，steady clock 单位 us。
    uint64_t command_time_us{0};
    /// 预计命中时刻，单位 us。
    uint64_t expected_hit_time_us{0};
    /// 选中的装甲板面索引。
    uint8_t selected_armor_index{0};
    /// 选中的 tracker 目标 id。
    ArmorNumber target_id{ArmorNumber::INVALID};
    /// 开火命令 yaw，单位 rad。
    double command_yaw{0.0};
    /// 开火命令 pitch，单位 rad。
    double command_pitch{0.0};
    /// 最近一次实测云台 yaw，单位 rad。
    double actual_gimbal_yaw{0.0};
    /// 最近一次实测云台 pitch，单位 rad。
    double actual_gimbal_pitch{0.0};
    /// 本次 shot 使用的弹速，单位 m/s。
    double bullet_speed{0.0};
    /// 从开火命令到弹丸出膛的估计延迟，单位 s。
    double fire_delay_s{0.0};
    /// 估计弹丸飞行时间，单位 s。
    double fly_time_est_s{0.0};
    /// 产生本次 shot 的自动开火门控结果。
    FireReason fire_reason{FireReason::NO_TRACK};
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
   * @brief 构建当前帧的调试弹道消息。
   */
  void BuildTrajectoryMessage(const ArmorTrackerTarget& target_msg,
                              const Eigen::Vector3d& aim_point, double fly_time,
                              double launch_pitch, double bullet_speed, double yaw,
                              double pitch);
  /**
   * @brief 发布决策消息，并在开火时发布 shot 事件。
   */
  void PublishDecisionAndMaybeShot();
  /**
   * @brief 将当前决策追加到可选 decision TSV 审计文件。
   */
  void WriteDecisionAudit();
  /**
   * @brief 将 shot 事件追加到可选 shot TSV 审计文件。
   */
  void WriteShotAudit(const AimerShotEvent& shot);
  /**
   * @brief 初始化 yaw 和 pitch TinyMPC 求解器。
   */
  void SetupGimbalPlanSolvers();
  /**
   * @brief 尝试为当前目标构建 TinyMPC 云台计划。
   */
  bool BuildMpcGimbalPlan(const ArmorTrackerTarget& target_msg,
                          double bullet_speed, bool fire);
  /**
   * @brief 在 TinyMPC 关闭或不可用时构建直接 yaw/pitch 计划。
   */
  void BuildFiniteDifferenceGimbalPlan(const ArmorTrackerTarget& target_msg,
                                       bool control, bool fire, double yaw,
                                       double pitch);
  /**
   * @brief 在 TinyMPC 和直接云台计划之间选择。
   */
  void BuildGimbalPlan(const ArmorTrackerTarget& target_msg, bool control,
                       bool fire, double yaw, double pitch, double bullet_speed);
  /**
   * @brief 清理与上一目标相关的规划器状态。
   */
  void ResetGimbalPlanHistory();

 private:
  Config cfg_{};
  std::atomic<double> bullet_speed_{23.0};
  int lock_id_{-1};
  ArmorNumber last_target_id_{ArmorNumber::INVALID};
  uint64_t frame_index_{0};
  uint64_t shot_index_{0};
  bool has_last_command_{false};
  double last_command_yaw_{0.0};
  double last_command_pitch_{0.0};
  bool has_gimbal_rotation_{false};
  double last_fire_tolerance_rad_{0.0};
  double last_fire_command_error_rad_{0.0};
  double last_fire_command_pitch_error_rad_{0.0};
  double last_fire_gimbal_error_rad_{0.0};
  double last_fire_gimbal_pitch_error_rad_{0.0};
  double last_fire_gimbal_yaw_rad_{0.0};
  double last_fire_gimbal_pitch_rad_{0.0};
  LibXR::Quaternion<double> gimbal_rotation_{1.0, 0.0, 0.0, 0.0};
  bool planner_ready_{false};
  bool last_plan_mpc_{false};
  TinySolver* yaw_solver_{nullptr};
  TinySolver* pitch_solver_{nullptr};
  mutable LibXR::Mutex gimbal_rotation_lock_{};

  AimerMetrics metrics_msg_{};
  AimerTrajectory trajectory_msg_{};
  AimerDecision decision_msg_{};
  AimerSend send_msg_{};
  GimbalPlan gimbal_plan_msg_{};
  /**
   * @brief 延迟打开的审计文件状态。
   */
  struct AuditFile
  {
    /// 环境变量配置的输出路径。
    std::string path{};
    /// TSV 文件输出流。
    std::ofstream file{};
    /// 避免重复打印打开失败日志。
    bool open_failed{false};
  };
  AuditFile decision_audit_{};
  AuditFile shot_audit_{};

  LibXR::Topic::Domain aimer_domain_ = LibXR::Topic::Domain("aimer");
  LibXR::Topic metrics_topic_ =
      LibXR::Topic("metrics", sizeof(AimerMetrics), &aimer_domain_);
  LibXR::Topic trajectory_topic_ = LibXR::Topic(
      LibXR::Topic::FindOrCreate<AimerTrajectory>("trajectory", &aimer_domain_));
  LibXR::Topic decision_topic_ =
      LibXR::Topic("decision", sizeof(AimerDecision), &aimer_domain_);
  LibXR::Topic shot_event_topic_ =
      LibXR::Topic("shot_event", sizeof(AimerShotEvent), &aimer_domain_);

  LibXR::Topic::Domain tracker_domain_ = LibXR::Topic::Domain("tracker");
  LibXR::Topic fire_notify_topic_ =
      LibXR::Topic("fire_notify", sizeof(uint8_t), &tracker_domain_);
  LibXR::Topic send_topic_ =
      LibXR::Topic("send", sizeof(AimerSend), &tracker_domain_);
  LibXR::Topic gimbal_plan_topic_ =
      LibXR::Topic("gimbal_plan", sizeof(GimbalPlan), &tracker_domain_);
};

#include "AimerImpl.hpp"
