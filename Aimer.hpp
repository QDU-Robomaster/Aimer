#pragma once

/**
 * @file Aimer.hpp
 * @brief Aimer 模块的公开接口和 Topic 数据声明。
 *        Public interface and Topic data declarations of the Aimer Module.
 */

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 弹道瞄准模块：选择装甲板、预测目标运动并解算云台目标与发射许可 / Ballistic aiming Module that selects the armor plate, predicts the target motion and solves the gimbal target and fire permission
depends:
- id: QDU-Robomaster/ArmorTracker
  ref: same-or-dev
- id: QDU-Robomaster/CameraBase
  ref: same-or-dev
- id: QDU-Robomaster/VisionPreview
  ref: same-or-dev
- id: xrobot-org/DurationStatistics
  ref: same-or-dev
- id: QDU-Robomaster/Referee
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <Eigen/Dense>
#include <atomic>
#include <cstdint>
#include <optional>
#include <string_view>
#include <utility>

#include "ArmorTrackerTarget.hpp"
#include "CameraBase.hpp"
#include "DurationStatistics.hpp"
#include "GimbalPlan.hpp"
#include "RefereeTypes.hpp"
#include "VisionPreview.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "libxr_string.hpp"
#include "logger.hpp"
#include "mutex.hpp"
#include "tinympc/tiny_api.hpp"

/**
 * @brief 裁判输入使用 RefereeTypes 的公共数据类型。
 *        The referee input uses the public data types of RefereeTypes.
 */
using AimerRefereeRobotStatus = RefereeTypes::RobotStatus;
using AimerRefereeGameStatus = RefereeTypes::GameStatus;
using AimerRefereeSummary = RefereeTypes::RobotGameRefereePack;

/**
 * @brief DevC HostData 接收的云台目标数据。
 *        Gimbal target data received by DevC HostData.
 */
struct AimerHostGimbalTarget
{
  /// 机械俯仰轴 roll 命令，单位 rad
  /// Mechanical pitch-axis roll command, in rad
  float rol{0.0f};
  /// 与 rol 相同的机械俯仰轴 roll 命令，单位 rad
  /// Mechanical pitch-axis roll command equal to rol, in rad
  float pit{0.0f};
  /// yaw 命令，单位 rad
  /// Yaw command, in rad
  float yaw{0.0f};
  /// 机械俯仰轴 roll 速度前馈，单位 rad/s
  /// Mechanical pitch-axis roll velocity feedforward, in rad/s
  float rol_dot{0.0f};
  /// 与 rol_dot 相同的 roll 速度前馈，单位 rad/s
  /// Roll velocity feedforward equal to rol_dot, in rad/s
  float pit_dot{0.0f};
  /// yaw 速度前馈，单位 rad/s
  /// Yaw velocity feedforward, in rad/s
  float yaw_dot{0.0f};
  /// 机械俯仰轴 roll 加速度前馈，单位 rad/s^2
  /// Mechanical pitch-axis roll acceleration feedforward, in rad/s^2
  float rol_ddot{0.0f};
  /// 与 rol_ddot 相同的 roll 加速度前馈，单位 rad/s^2
  /// Roll acceleration feedforward equal to rol_ddot, in rad/s^2
  float pit_ddot{0.0f};
  /// yaw 加速度前馈，单位 rad/s^2
  /// Yaw acceleration feedforward, in rad/s^2
  float yaw_ddot{0.0f};
};

static_assert(sizeof(AimerHostGimbalTarget) == sizeof(float) * 9);

/**
 * @brief DevC LauncherCMD 接收的发射许可数据。
 *        Fire permission data received by DevC LauncherCMD.
 */
struct AimerHostFireNotify
{
  /// 是否允许发射
  /// Whether firing is permitted
  bool isfire{false};
};

static_assert(sizeof(AimerHostFireNotify) == 1);

/**
 * @brief Aimer 内置预览绘制所需的同帧状态。
 *        Same-frame state required by the built-in preview of Aimer.
 */
struct AimerPreviewFrame
{
  /// 匹配触发沿的 MCU 陀螺仪时间戳，单位 us
  /// MCU gyroscope timestamp matching the trigger edge, in us
  uint64_t image_timestamp_us{};
  /// 当前帧是否带有 tracker 目标消息
  /// Whether the frame carries a tracker target message
  bool have_target{false};
  /// tracker 原始目标消息
  /// Raw tracker target message
  ArmorTrackerTarget target{};
  /// 是否存在 Aimer 选中的预测瞄点
  /// Whether Aimer selected a predicted aim point
  bool aim_point_valid{false};
  /// 预测后的瞄点中心，tracker 输出坐标系
  /// Predicted aim-point center in the tracker output frame
  Eigen::Vector3d aim_point{Eigen::Vector3d::Zero()};
  /// 预测选中的装甲板索引
  /// Index of the predicted selected armor plate
  int aim_armor_index{-1};
  /// 预测选中的装甲板中心 x、y、z 和 yaw
  /// Center x, y, z and yaw of the predicted selected armor plate
  Eigen::Vector4d aim_xyza{Eigen::Vector4d::Zero()};
  /// 是否已生成 host/fire_notify 输出
  /// Whether the host/fire_notify output has been generated
  bool have_host_fire{false};
  /// 本帧发射许可输出
  /// Fire permission output of this frame
  AimerHostFireNotify host_fire{};
};

/**
 * @brief 单发弹丸对应的未来命中候选。
 *        Future hit candidate of a single projectile.
 */
struct AimerShotCandidate
{
  /// 候选是否有效
  /// Whether the candidate is valid
  bool valid{false};
  /// 命中时刻该装甲板姿态是否允许开火
  /// Whether the armor plate pose allows firing at the hit time
  bool face_shootable_at_hit{false};
  /// 命中时刻对应的物理装甲板索引
  /// Physical armor plate index at the hit time
  int hit_face{-1};
  /// 命中时刻装甲板相对整车中心方位的视角，单位 rad
  /// View angle of the armor plate relative to the vehicle center bearing at the hit
  /// time, in rad
  double view_angle{0.0};
  /// 命中时刻装甲板中心 x、y、z 和 yaw
  /// Armor plate center x, y, z and yaw at the hit time
  Eigen::Vector4d hit_xyza{Eigen::Vector4d::Zero()};
  /// 指向该命中候选的 yaw，单位 rad
  /// Yaw pointing at the hit candidate, in rad
  double yaw{0.0};
  /// 指向该命中候选的机械 roll 轴命令，单位 rad
  /// Mechanical roll-axis command pointing at the hit candidate, in rad
  double roll{0.0};
  /// 命中候选对应的弹丸飞行时间，单位 s
  /// Projectile flight time of the hit candidate, in s
  double fly_time{0.0};
};

/**
 * @brief Aimer 运行时配置，由 xrobot YAML 生成。
 *        Aimer runtime configuration, generated from the xrobot YAML.
 */
struct AimerConfig
{
  /// 施加到 yaw 命令的固定偏置，单位 deg
  /// Fixed bias applied to the yaw command, in deg
  double yaw_offset{-1.0};
  /// 施加到机械 roll 轴命令的固定偏置，单位 deg
  /// Fixed bias applied to the mechanical roll-axis command, in deg
  double roll_offset{-1.4};
  /// 高/低速预测补偿的 yaw 角速度阈值，单位 rad/s
  /// Yaw angular-velocity threshold of the high/low-speed prediction compensation, in
  /// rad/s
  double yaw_rate_threshold{2.0};
  /// 弹速，单位 m/s
  /// Bullet speed, in m/s
  double default_bullet_speed{21.0};
  /// 弹速低于该值时改用 default_bullet_speed，单位 m/s
  /// default_bullet_speed is used when the bullet speed is below this value, in m/s
  double min_valid_bullet_speed{14.0};
  /// 二次阻力加速度系数，a_drag = -k * |v| * v
  /// Quadratic drag acceleration coefficient, a_drag = -k * |v| * v
  double ballistic_drag_k{0.02};
  /// RK4 弹道积分步长，单位 s，限制在 0.0001 到 0.02
  /// RK4 ballistic integration step, in s, limited to 0.0001 to 0.02
  double ballistic_integration_dt_s{0.001};
  /// 仰角括区求根最大迭代次数，限制在 4 到 64
  /// Maximum iterations of the bracketed elevation root search, limited to 4 to 64
  int ballistic_max_iterations{16};
  /// 弹道仰角搜索下限，单位 deg
  /// Lower bound of the ballistic elevation search, in deg
  double ballistic_min_elevation_deg{-20.0};
  /// 弹道仰角搜索上限，单位 deg
  /// Upper bound of the ballistic elevation search, in deg
  double ballistic_max_elevation_deg{35.0};
  /// 是否启用基于实测云台姿态的自动开火门控
  /// Whether the automatic fire gating based on the measured gimbal attitude is enabled
  bool auto_fire{true};
  /// 从图像曝光到当前处理时刻的预测延迟，单位 s
  /// Prediction delay from the image exposure to the current processing time, in s
  double image_to_now_s{0.0};
  /// 从视觉输出到命令生成的预测延迟，单位 s
  /// Prediction delay from the vision output to the command generation, in s
  double vision_to_command_delay_s{0.0};
  /// 命令传输的预测延迟，单位 s
  /// Prediction delay of the command transport, in s
  double command_transport_delay_s{0.0};
  /// 云台响应的预测延迟，单位 s
  /// Prediction delay of the gimbal response, in s
  double gimbal_response_delay_s{0.0};
  /// 从开火命令到弹丸出膛的延迟，决定开火采样点，单位 s
  /// Delay from the fire command to the projectile leaving the barrel, determines the
  /// fire sampling point, in s
  double fire_delay_s{0.0};
  /// 低速目标的额外预测时间，单位 s
  /// Extra prediction time for low-speed targets, in s
  double low_speed_extra_predict_s{0.015};
  /// yaw 角速度超过阈值时的额外预测时间，单位 s
  /// Extra prediction time when the yaw angular velocity exceeds the threshold, in s
  double high_speed_extra_predict_s{0.03};
  /// 动态开火角度阈值的下限，单位 rad
  /// Lower bound of the dynamic fire angle threshold, in rad
  double min_fire_threshold{0.003};
  /// 动态开火角度阈值的上限，单位 rad
  /// Upper bound of the dynamic fire angle threshold, in rad
  double max_fire_threshold{0.05};
  /// 是否启用 TinyMPC 云台计划
  /// Whether the TinyMPC gimbal plan is enabled
  bool enable_mpc_plan{true};
  /// 允许使用 MPC 输出和开火的最大计划偏离，单位 rad
  /// Maximum plan deviation that allows using the MPC output and firing, in rad
  double mpc_fire_thresh{0.05};
  /// TinyMPC yaw 加速度约束，单位 rad/s^2
  /// TinyMPC yaw acceleration constraint, in rad/s^2
  double max_yaw_acc{50.0};
  /// TinyMPC yaw 位置代价
  /// TinyMPC yaw position cost
  double q_yaw_pos{9000000.0};
  /// TinyMPC yaw 速度代价
  /// TinyMPC yaw velocity cost
  double q_yaw_vel{0.0};
  /// TinyMPC yaw 加速度代价
  /// TinyMPC yaw acceleration cost
  double r_yaw_acc{1.0};
  /// TinyMPC roll 轴加速度约束，单位 rad/s^2
  /// TinyMPC roll-axis acceleration constraint, in rad/s^2
  double max_roll_acc{100.0};
  /// TinyMPC roll 轴位置代价
  /// TinyMPC roll-axis position cost
  double q_roll_pos{9000000.0};
  /// TinyMPC roll 轴速度代价
  /// TinyMPC roll-axis velocity cost
  double q_roll_vel{0.0};
  /// TinyMPC roll 轴加速度代价
  /// TinyMPC roll-axis acceleration cost
  double r_roll_acc{1.0};
  /// Aimer 内置实时预览的运行参数
  /// Runtime parameters of the built-in live preview of Aimer
  VisionPreview::RuntimeParam preview{};
  /// 是否输出运行期统计日志
  /// Whether the runtime statistics log is output
  bool enable_runtime_log{true};
  /// 弹速变化超过该阈值时输出日志，单位 m/s
  /// The bullet speed is logged when it changes by more than this value, in m/s
  double bullet_speed_log_delta{0.05};
  /// 热量、热量上限或冷却值变化超过该阈值时输出日志
  /// The heat, heat limit or cooling value is logged when it changes by more than this
  /// value
  double heat_log_delta{1.0};
  /// 是否把原始 x 前、y 左、z 上的 ahrs_quaternion 转到 body 轴
  /// Whether the raw x-forward, y-left, z-up ahrs_quaternion is converted to the body
  /// axes
  bool convert_raw_gimbal_quat_to_body{false};
  /// host 域内裁判 Topic 名称，须非空；构造时复制并订阅
  /// Name of the referee Topic in the host domain, non-empty; copied and subscribed at
  /// construction
  std::string_view referee_topic{"robot_game_ref"};
};

/**
 * @brief 选择目标装甲板、解算 yaw 与 roll 轴命令，并发布云台目标与发射许可。
 *        Select the target armor plate, solve the yaw and roll-axis commands, and publish
 *        the gimbal target and the fire permission.
 */
class AimerCore
{
 public:
  using Config = AimerConfig;
  using PreviewSink = void (*)(void*, const AimerPreviewFrame&);

  /**
   * @brief 创建 Aimer 运行核心，初始化 TinyMPC 求解器并订阅 host 域的裁判 Topic。
   *        Create the Aimer core, initialize the TinyMPC solvers and subscribe to the
   *        referee Topic of the host domain.
   *
   * @param cfg 运行时配置。
   *            Runtime configuration.
   */
  AimerCore(Config cfg);

  /**
   * @brief 输出 target_frame 回调的累计耗时统计。
   *        Output the accumulated duration statistics of the target_frame callback.
   */
  void OnMonitor();

 protected:
  /**
   * @brief 设置接收每帧预览状态的回调。
   *        Set the callback that receives the preview state of each frame.
   *
   * @param sink 预览状态回调。
   *             Preview state callback.
   * @param context 传给回调的第一个参数。
   *                First argument passed to the callback.
   */
  void SetPreviewSink(PreviewSink sink, void* context);
  /**
   * @brief 使用 tracker 同帧 IMU 更新当前云台姿态。
   *        Update the current gimbal attitude from the same-frame IMU of the tracker.
   *
   * @param rotation_wxyz 云台姿态四元数，顺序为 w、x、y、z。
   *                      Gimbal attitude quaternion in the order w, x, y, z.
   */
  void UpdateGimbalRotationFromSyncedImu(const std::array<float, 4>& rotation_wxyz);
  /**
   * @brief 清空当前云台姿态，自动开火门控在下一次姿态更新前不通过。
   *        Clear the current gimbal attitude; the automatic fire gating does not pass
   *        until the next attitude update.
   */
  void ClearGimbalRotation();
  /**
   * @brief 处理一帧 tracker 目标并发布 host 输出。
   *        Process one tracker target and publish the host outputs.
   *
   * @param target_msg 当前 tracker 目标。
   *                   Current tracker target.
   */
  void TargetCallback(const ArmorTrackerTarget& target_msg);

 private:
  /**
   * @brief 注册裁判系统输入回调。
   *        Register the referee input callback.
   */
  void RegisterHostInputCallbacks();
  /**
   * @brief 订阅 host 域的 ahrs_quaternion 云台姿态 Topic。
   *        Subscribe to the ahrs_quaternion gimbal attitude Topic of the host domain.
   */
  void RegisterGimbalQuatInput();
  /**
   * @brief 订阅 host 域的裁判 Topic。
   *        Subscribe to the referee Topic of the host domain.
   */
  void RegisterRefereeSummaryInput();
  /**
   * @brief 更新弹速缓存，变化量超过阈值时输出日志。
   *        Update the bullet-speed cache and log it when the change exceeds the
   * threshold.
   *
   * @param bullet_speed_msg 弹速，单位 m/s。
   *                         Bullet speed, in m/s.
   * @param source 日志中的来源名称。
   *               Source name shown in the log.
   */
  void UpdateBulletSpeed(float bullet_speed_msg, const char* source);
  /**
   * @brief 处理裁判数据。
   *        Handle the referee data.
   *
   * @param summary 裁判数据。
   *                Referee data.
   */
  void RefereeSummaryCallback(const AimerRefereeSummary& summary);
  /**
   * @brief 变化量超过阈值时记录热量、热量上限和冷却值。
   *        Log the heat, heat limit and cooling value when a change exceeds the
   * threshold.
   *
   * @param current_heat 当前热量，非有限值表示未知。
   *                     Current heat; a non-finite value means unknown.
   * @param heat_limit 热量上限。
   *                   Heat limit.
   * @param cooling 冷却值。
   *                Cooling value.
   * @param source 日志中的来源名称。
   *               Source name shown in the log.
   * @param force 为 true 时无条件记录。
   *              Log unconditionally when true.
   */
  void LogHeatStatus(double current_heat, double heat_limit, double cooling,
                     const char* source, bool force);
  /**
   * @brief 自动开火状态翻转时输出日志。
   *        Log when the automatic fire state flips.
   *
   * @param target_msg 当前 tracker 目标。
   *                   Current tracker target.
   * @param fire 当前开火状态。
   *             Current fire state.
   * @param bullet_speed 弹速，单位 m/s。
   *                     Bullet speed, in m/s.
   */
  void LogFireState(const ArmorTrackerTarget& target_msg, bool fire, double bullet_speed);
  /**
   * @brief 用 ahrs_quaternion 消息更新云台姿态。
   *        Update the gimbal attitude from an ahrs_quaternion message.
   *
   * @param gimbal_rotation_msg 云台姿态四元数。
   *                            Gimbal attitude quaternion.
   */
  void GimbalRotationCallback(LibXR::Quaternion<float> gimbal_rotation_msg);
  /**
   * @brief 把本帧 Aimer 状态交给内置预览。
   *        Hand the Aimer state of this frame to the built-in preview.
   *
   * @param state 本帧预览状态。
   *              Preview state of this frame.
   */
  void PublishPreviewState(const AimerPreviewFrame& state);
  /**
   * @brief 根据命令稳定性和云台两轴对准情况评估自动开火门控。
   *        Evaluate the automatic fire gating from the command stability and the
   *        alignment of the two gimbal axes.
   *
   * @param shot_candidate 当前发射对应的未来命中候选。
   *                       Future hit candidate of the current shot.
   * @param plan_fire_enabled 命中面和云台计划是否允许开火。
   *                          Whether the hit face and the gimbal plan allow firing.
   * @param yaw 命令 yaw，单位 rad。
   *            Command yaw, in rad.
   * @param roll 命令机械 roll 轴，单位 rad。
   *             Command mechanical roll axis, in rad.
   * @return 开火门控全部通过时为 true。
   *         True when all fire gates pass.
   */
  bool ShouldAutoFire(const AimerShotCandidate& shot_candidate, bool plan_fire_enabled,
                      double yaw, double roll);
  /**
   * @brief 初始化 yaw 和 roll 轴 TinyMPC 求解器。
   *        Initialize the yaw and roll-axis TinyMPC solvers.
   */
  void SetupGimbalPlanSolvers();
  /**
   * @brief 尝试为当前目标构建 TinyMPC 云台计划。
   *        Try to build the TinyMPC gimbal plan for the current target.
   *
   * @param target_msg 当前 tracker 目标。
   *                   Current tracker target.
   * @param delay_time 固定预测延迟，单位 s。
   *                   Fixed prediction delay, in s.
   * @param bullet_speed 弹速，单位 m/s。
   *                     Bullet speed, in m/s.
   * @param fire_shot_candidate 输出开火采样点的命中候选。
   *                            Output hit candidate at the fire sampling point.
   * @return 得到有效 MPC 计划时为 true。
   *         True when a valid MPC plan is produced.
   */
  bool BuildMpcGimbalPlan(const ArmorTrackerTarget& target_msg, double delay_time,
                          double bullet_speed, AimerShotCandidate& fire_shot_candidate);
  /**
   * @brief 构建直接的 yaw 与 roll 计划，速度与加速度前馈为 0。
   *        Build the direct yaw and roll plan with zero velocity and acceleration
   *        feedforward.
   *
   * @param target_msg 当前 tracker 目标。
   *                   Current tracker target.
   * @param control 下位机是否使用该命令。
   *                Whether the lower controller uses the command.
   * @param fire_enabled 该计划是否允许开火。
   *                     Whether the plan allows firing.
   * @param yaw 命令 yaw，单位 rad。
   *            Command yaw, in rad.
   * @param roll 命令 roll 轴，单位 rad。
   *             Command roll axis, in rad.
   */
  void BuildFiniteDifferenceGimbalPlan(const ArmorTrackerTarget& target_msg, bool control,
                                       bool fire_enabled, double yaw, double roll);
  /**
   * @brief 优先使用 TinyMPC 计划，其不可用时使用直接计划。
   *        Use the TinyMPC plan, falling back to the direct plan when it is unavailable.
   *
   * @param target_msg 当前 tracker 目标。
   *                   Current tracker target.
   * @param delay_time 固定预测延迟，单位 s。
   *                   Fixed prediction delay, in s.
   * @param control 下位机是否使用该命令。
   *                Whether the lower controller uses the command.
   * @param yaw 命令 yaw，单位 rad。
   *            Command yaw, in rad.
   * @param roll 命令 roll 轴，单位 rad。
   *             Command roll axis, in rad.
   * @param bullet_speed 弹速，单位 m/s。
   *                     Bullet speed, in m/s.
   * @param direct_shot_candidate 直接命令对应的命中候选。
   *                              Hit candidate of the direct command.
   * @param fire_shot_candidate 输出开火门控使用的命中候选。
   *                            Output hit candidate used by the fire gating.
   */
  void BuildGimbalPlan(const ArmorTrackerTarget& target_msg, double delay_time,
                       bool control, double yaw, double roll, double bullet_speed,
                       const AimerShotCandidate& direct_shot_candidate,
                       AimerShotCandidate& fire_shot_candidate);
  /**
   * @brief 清除上一帧的规划器状态。
   *        Clear the planner state of the previous frame.
   */
  void ResetGimbalPlanHistory();

 private:
  Config cfg_{};
  /// 裁判 Topic 名称的副本
  /// Copy of the referee Topic name
  LibXR::RuntimeStringView<> referee_topic_name_;
  XRobot::DurationStatistics target_callback_duration_{};
  std::atomic<double> bullet_speed_{23.0};
  int lock_id_{-1};
  ArmorNumber last_target_id_{ArmorNumber::INVALID};
  bool has_last_command_{false};
  double last_command_yaw_{0.0};
  double last_command_roll_{0.0};
  bool has_gimbal_rotation_{false};
  LibXR::Quaternion<double> gimbal_rotation_{1.0, 0.0, 0.0, 0.0};
  bool planner_ready_{false};
  bool last_plan_mpc_{false};
  bool have_logged_fire_state_{false};
  bool last_logged_fire_state_{false};
  bool have_logged_bullet_speed_{false};
  double last_logged_bullet_speed_{0.0};
  bool have_logged_heat_status_{false};
  bool have_logged_current_heat_{false};
  double last_logged_heat_{0.0};
  double last_logged_heat_limit_{0.0};
  double last_logged_cooling_{0.0};
  TinySolver* yaw_solver_{nullptr};
  TinySolver* roll_solver_{nullptr};
  mutable LibXR::Mutex gimbal_rotation_lock_{};
  mutable LibXR::Mutex runtime_log_lock_{};
  PreviewSink preview_sink_{nullptr};
  void* preview_context_{nullptr};

  GimbalPlan gimbal_plan_msg_{};

  LibXR::Topic::Domain host_domain_ = LibXR::Topic::Domain("host");
  LibXR::Topic host_gimbal_topic_ =
      LibXR::Topic::CreateTopic<AimerHostGimbalTarget>("target_euler", &host_domain_);
  LibXR::Topic host_fire_topic_ =
      LibXR::Topic::CreateTopic<AimerHostFireNotify>("fire_notify", &host_domain_);
};

#include "AimerImpl.hpp"
#include "AimerPreview.hpp"

/**
 * @brief Aimer 模块：订阅 tracker 的 target_frame，发布云台目标与发射许可，内置预览。
 *        Aimer Module: subscribes to the tracker target_frame, publishes the gimbal
 *        target and the fire permission, and provides a built-in preview.
 *
 * @tparam FrameLayoutV 帧布局，与上游相机和 ArmorTracker 相同。
 *                      Frame layout, identical to the upstream camera and ArmorTracker.
 */
template <CameraTypes::FrameLayout FrameLayoutV>
class Aimer : public AimerCore
{
 public:
  using Config = AimerConfig;
  using CameraCalibration = CameraTypes::CameraCalibration;
  using TargetFrame = TrackedFrame<FrameLayoutV>;
  using TargetFrameMessage = TrackedFrameMessage<FrameLayoutV>;

  /**
   * @brief 返回全部取默认值的配置。
   *        Return the configuration holding all defaults.
   *
   * @return 默认配置。
   *         Default configuration.
   */
  static Config DefaultConfig() { return {}; }

  /**
   * @brief 返回默认相机标定：1280x720，fx = fy = 800，主点 (640, 360)，零畸变。
   *        Return the default camera calibration: 1280x720, fx = fy = 800, principal
   *        point (640, 360), zero distortion.
   *
   * @return 默认相机标定。
   *         Default camera calibration.
   */
  static CameraCalibration DefaultCalibration()
  {
    return {.native_width = 1280,
            .native_height = 720,
            .camera_matrix = {800.0, 0.0, 640.0, 0.0, 800.0, 360.0, 0.0, 0.0, 1.0},
            .distortion_model = CameraTypes::DistortionModel::PLUMB_BOB,
            .distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0},
            .rectification_matrix = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0},
            .projection_matrix = {800.0, 0.0, 640.0, 0.0, 0.0, 800.0, 360.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0}};
  }

  /**
   * @brief 构造 Aimer，持有原生相机标定，并订阅 tracker 域的 target_frame。
   *        Construct Aimer, hold the native camera calibration and subscribe to the
   *        target_frame of the tracker domain.
   *
   * @param cfg 运行时配置。
   *            Runtime configuration.
   * @param calibration 原生传感器坐标系下的相机标定，用于预览投影，按值持有。
   *                    Camera calibration in the native sensor frame used for the preview
   *                    projection, held by value.
   */
  Aimer(Config cfg = DefaultConfig(),
        CameraCalibration calibration = DefaultCalibration())
      : AimerCore(cfg), calibration_(std::move(calibration))
  {
    ASSERT(CameraBaseIntrinsicSanity::CameraCalibrationReasonable(calibration_));
    if (cfg.preview.enabled)
    {
      preview_.emplace(AimerDetail::MakeAimerPreviewConfig(cfg), calibration_);
      SetPreviewSink([](void* context, const AimerPreviewFrame& frame)
                     { static_cast<Aimer*>(context)->SubmitPreviewFrame(frame); }, this);
    }
    RegisterTargetFrameCallback();
  }

 private:
  /**
   * @brief 订阅 tracker/target_frame。
   *        Subscribe to tracker/target_frame.
   */
  void RegisterTargetFrameCallback()
  {
    LibXR::Topic::Domain tracker_domain("tracker");
    target_frame_topic_ =
        LibXR::Topic::FindOrCreate<TargetFrameMessage>("target_frame", &tracker_domain);
    auto callback = LibXR::Topic::Callback::Create(
        [](bool, Aimer* self, const TargetFrameMessage& message)
        {
          if (message == nullptr || !message->Valid())
          {
            return;
          }
          self->TargetFrameCallback(*message);
        },
        this);
    target_frame_topic_.RegisterCallback(callback);
  }

  /**
   * @brief 处理 tracker 同帧的目标与源图像。
   *        Handle the target and the source image of the same tracker frame.
   *
   * @param frame 目标帧。
   *              Target frame.
   */
  void TargetFrameCallback(const TargetFrame& frame)
  {
    current_target_frame_ = &frame;
    UpdateGimbalRotationFromSyncedImu(frame.imu.rotation_wxyz);
    TargetCallback(frame.target);
    current_target_frame_ = nullptr;
  }

  /**
   * @brief 把 AimerCore 生成的预览状态交给内置预览。
   *        Hand the preview state generated by AimerCore to the built-in preview.
   *
   * @param frame 预览状态。
   *              Preview state.
   */
  void SubmitPreviewFrame(const AimerPreviewFrame& frame)
  {
    if (preview_.has_value())
    {
      preview_->OnAimerFrame(frame, current_target_frame_);
    }
  }

  LibXR::Topic target_frame_topic_ = LibXR::Topic();
  const TargetFrame* current_target_frame_{nullptr};
  const CameraCalibration calibration_;
  std::optional<AimerPreview<FrameLayoutV>> preview_;
};
