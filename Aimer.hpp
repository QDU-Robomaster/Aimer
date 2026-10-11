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
- id: QDU-Robomaster/AutoAimTypes
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
#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "AimerBulletSpeed.hpp"
#include "AimerHeatFire.hpp"
#include "AimerLeadCalibration.hpp"
#include "AutoAimTypes.hpp"
#include "DurationStatistics.hpp"
#include "GimbalPlan.hpp"
#include "RefereeTypes.hpp"
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

/// 0x0207 中 17 mm 与 42 mm 发射机构的发射机构 ID
/// Launcher IDs of the 17 mm and 42 mm launchers in 0x0207
inline constexpr uint8_t REFEREE_LAUNCHER_ID_17MM = 1;
inline constexpr uint8_t REFEREE_LAUNCHER_ID_42MM = 3;

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
 * @brief Aimer 处理一帧后的状态，由模块转成 `AutoAim::AimResult` 发布。
 *        State of Aimer after one frame, published by the Module as `AutoAim::AimResult`.
 */
struct AimerFrameState
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
  /// 本帧是否控制云台
  /// Whether the gimbal is commanded in this frame
  bool control{false};
  /// 本帧云台目标输出
  /// Gimbal target output of this frame
  AimerHostGimbalTarget host_gimbal{};
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
  /// 是否使用裁判系统 0x0207 的实测弹速；有效读数不足 bullet_speed_min_samples 发时
  /// 使用 default_bullet_speed
  /// Whether the measured bullet speed of referee 0x0207 is used; default_bullet_speed is
  /// used until bullet_speed_min_samples valid readings have arrived
  bool referee_bullet_speed{true};
  /// 实测弹速平滑窗口的发数，限制在 1 到 32
  /// Number of shots in the measured-speed smoothing window, limited to 1 to 32
  int bullet_speed_window{9};
  /// 使用实测弹速所需的最少有效读数
  /// Minimum number of valid readings before the measured speed is used
  int bullet_speed_min_samples{3};
  /// 实测弹速的有效上限，单位 m/s；有效下限为 min_valid_bullet_speed
  /// Upper bound of a valid measured speed, in m/s; the lower bound is
  /// min_valid_bullet_speed
  double max_valid_bullet_speed{35.0};
  /// 实测弹速离群门限的下限，单位 m/s
  /// Floor of the outlier gate of the measured speed, in m/s
  double bullet_speed_outlier_m_s{0.5};
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
  double fire_delay_s{0.02};
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
  /// 是否按热量分配开火：估计每次开火的命中概率，只在概率高于随热量升高的门槛时开火；
  /// 需要裁判系统给出热量上限和冷却值，未收到时不生效
  /// Whether heat-aware firing is enabled: the hit probability of each fire opportunity
  /// is estimated and the shot is fired only above a threshold that rises with heat;
  /// requires the heat limit and cooling value from the referee and is inactive without
  /// them
  bool heat_aware_fire{false};
  /// 热量为 0 时开火所需的命中概率
  /// Hit probability required to fire at zero heat
  double heat_fire_p_low{0.55};
  /// 热量达到上限时开火所需的命中概率
  /// Hit probability required to fire at the heat limit
  double heat_fire_p_high{0.85};
  /// 热量低于上限一半且长时间未开火时，门槛放宽到的下限
  /// Lower bound the threshold is relaxed to when the heat is below half the limit and
  /// no shot was fired for a while
  double heat_fire_p_floor{0.3};
  /// 开始放宽门槛前允许的不开火时间，单位 s
  /// Time without a shot before the threshold starts to relax, in s
  double heat_fire_relax_s{0.5};
  /// 命中概率模型的基础横向标准差，单位 m
  /// Base lateral standard deviation of the hit-probability model, in m
  double heat_fire_sigma_m{0.01};
  /// 装甲板相位预测标准差随预测时域的增长率，单位 rad/s
  /// Growth rate of the plate-phase prediction standard deviation with the horizon, in
  /// rad/s
  double heat_fire_phase_sigma_rad_s{0.7};
  /// 加在弹丸飞行时间上的预测时域，覆盖开火指令到出膛的延迟，单位 s
  /// Horizon added to the flight time, covering the delay from the fire command to the
  /// muzzle, in s
  double heat_fire_horizon_extra_s{0.05};
  /// 单发热量，17 mm 弹丸为 10
  /// Heat per shot, 10 for 17 mm projectiles
  double heat_fire_shot_heat{10.0};
  /// 发射机构相邻两发的最小间隔，单位 s
  /// Minimum interval between two shots of the launcher, in s
  double heat_fire_min_interval_s{0.05};
  /// 出膛时仍保留的请求时刻云台误差比例；1 表示按请求时刻误差估计
  /// Share of the request-time gimbal error that remains at the muzzle exit; 1 uses the
  /// request-time error as it is
  double heat_fire_gimbal_error_gain{0.65};
  /// 横向偏差中与指令 yaw 角速度成正比的时间，单位 s
  /// Time multiplied by the command yaw rate in the lateral offset, in s
  double heat_fire_rate_bias_s{0.01};
  /// 横向标准差中与指令 yaw 角速度成正比的时间，单位 s
  /// Time multiplied by the command yaw rate in the lateral standard deviation, in s
  double heat_fire_rate_spread_s{0.01};
  /// 是否用裁判系统 0x0202 的实测枪管热量修正本地热量估计；最近一包裁判摘要早于
  /// referee_heat_timeout_s 时只用本地估计
  /// Whether the local heat estimate is corrected with the barrel heat measured by referee
  /// 0x0202; only the local estimate is used when the latest referee summary is older than
  /// referee_heat_timeout_s
  bool referee_heat{true};
  /// 实测热量有效的最长时间，单位 s
  /// Longest time a measured heat stays valid, in s
  double referee_heat_timeout_s{0.5};
  /// 实测热量可能尚未计入的出弹时间窗，单位 s，覆盖 0x0202 的发送周期和出弹延迟
  /// Time window of shots the measured heat may not include yet, in s, covering the
  /// 0x0202 period and the launch delay
  double referee_heat_window_s{0.15};
  /// 使用 42 mm 发射机构的热量；否则使用 17 mm 发射机构的热量
  /// Use the heat of the 42 mm launcher; otherwise that of the 17 mm launcher
  bool referee_heat_42mm{false};
  /// 是否把 0x0204 的冷却增益加到 0x0201 的每秒冷却值上
  /// Whether the cooling buff of 0x0204 is added to the cooling per second of 0x0201
  bool referee_cooling_add_buff{false};
  /// 是否在线标定指向超前量：用之后的帧检查云台指向，修正预测延迟，收敛后固定
  /// Whether the pointing lead is calibrated online: the gimbal pointing is checked
  /// against later frames, the prediction delay is corrected and frozen once calibrated
  bool lead_calibration{false};
  /// 固定前的修正批数
  /// Number of correcting batches before the correction is frozen
  int lead_calibration_batches{5};
  /// 修正量绝对值上限，单位 s
  /// Limit of the absolute correction, in s
  double lead_calibration_max_adjust_s{0.05};
  /// 固定后触发重新标定的超前时间，单位 s
  /// Lead time that triggers a recalibration once frozen, in s
  double lead_calibration_monitor_threshold_s{0.003};
  /// 触发重新标定所需的同向连续批数
  /// Consecutive same-sign batches that trigger a recalibration
  int lead_calibration_monitor_batches{3};
  /// 参与超前量标定的最大目标水平距离，单位 m；更远时飞行时间内的目标机动主导残差
  /// Largest horizontal target distance used for the lead calibration, in m; farther away
  /// the target maneuver during the flight dominates the residual
  double lead_calibration_max_distance_m{4.0};
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
  using FrameSink = void (*)(void*, const AimerFrameState&);

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
   * @brief 设置接收每帧状态的回调。
   *        Set the callback that receives the state of each frame.
   *
   * @param sink 每帧状态回调。
   *             Per-frame state callback.
   * @param context 传给回调的第一个参数。
   *                First argument passed to the callback.
   */
  void SetFrameSink(FrameSink sink, void* context);
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
   * @brief 把本帧 Aimer 状态交给每帧状态回调。
   *        Hand the Aimer state of this frame to the per-frame state callback.
   *
   * @param state 本帧状态。
   *              State of this frame.
   */
  void PublishFrameState(const AimerFrameState& state);
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
   * @brief 在已有开火门控之后应用按热量分配的开火判定，并维护本地热量估计。
   *        Apply the heat-aware fire decision after the existing fire gates and keep the
   *        local heat estimate.
   *
   * @param shot_candidate 当前发射对应的未来命中候选。
   *                       Future hit candidate of the current shot.
   * @param gimbal_error_yaw 实测云台 yaw 减命令 yaw，单位 rad。
   *                         Measured gimbal yaw minus the command yaw, in rad.
   * @param command_yaw_rate 命令 yaw 角速度，单位 rad/s。
   *                         Command yaw rate, in rad/s.
   * @param gates_passed 已有开火门控是否全部通过。
   *                     Whether all existing fire gates pass.
   * @return 最终是否开火。
   *         Whether the shot is finally fired.
   */
  bool HeatAwareFire(const AimerShotCandidate& shot_candidate, double gimbal_error_yaw,
                     double command_yaw_rate, bool gates_passed);
  /**
   * @brief 用最近一包裁判摘要的实测热量修正本地热量估计；实测值超过
   *        referee_heat_timeout_s 时不修正。
   *        Correct the local heat estimate with the measured heat of the latest referee
   *        summary; no correction when the measurement is older than
   *        referee_heat_timeout_s.
   *
   * @param now_us 当前图像时刻，单位 us。
   *               Current image time, in us.
   * @param cooling 每秒冷却值。
   *                Cooling per second.
   */
  void FuseRefereeHeat(uint64_t now_us, double cooling);
  /**
   * @brief 在线标定指向超前量：记录本帧云台指向，用本帧观测检查此前各帧的指向，按批
   *        修正预测延迟。
   *        Online calibration of the pointing lead: record the gimbal pointing of this
   *        frame, check the pointing of earlier frames against this observation and
   *        correct the prediction delay in batches.
   *
   * @param target_msg 当前 tracker 目标。
   *                   Current tracker target.
   * @param command_in_force 上一帧是否发出了云台指令。
   *                         Whether a gimbal command was issued for the previous frame.
   * @param command_yaw_rate 当前生效指令的 yaw 角速度，单位 rad/s。
   *                         Yaw rate of the command in force, in rad/s.
   */
  void UpdateLeadCalibration(const ArmorTrackerTarget& target_msg, bool command_in_force,
                             double command_yaw_rate);
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
  /// 上一帧命令的 yaw、roll 角速度和图像时间，用于判断命令是否稳定
  /// Yaw and roll rates and image time of the previous command, for the command stability
  /// check
  double last_command_yaw_vel_{0.0};
  double last_command_roll_vel_{0.0};
  uint64_t last_command_image_us_{0};
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
  /// 裁判系统给出的热量上限，未收到时为 0
  /// Heat limit from the referee, 0 until received
  std::atomic<double> referee_heat_limit_{0.0};
  /// 裁判系统给出的每秒冷却值
  /// Cooling per second from the referee
  std::atomic<double> referee_cooling_{0.0};
  /// 按热量分配开火使用的本地热量估计
  /// Local heat estimate of the heat-aware firing
  AimerDetail::HeatFireState heat_fire_state_{};
  /// 最近一包裁判摘要中的实测热量和该包的接收时刻（host 时基，单位 us）
  /// Measured heat of the latest referee summary and its arrival time (host timebase, in
  /// us)
  struct RefereeHeatSample
  {
    bool valid{false};
    double heat{0.0};
    uint64_t receive_us{0};
  };
  LibXR::Mutex referee_heat_lock_;
  RefereeHeatSample referee_heat_sample_{};
  /// 实测弹速平滑，只在裁判回调中访问
  /// Measured-speed smoothing, accessed only in the referee callback
  AimerDetail::BulletSpeedFilter bullet_speed_filter_;
  /// 是否已收到过裁判摘要，及其中最近的 0x0207 计数
  /// Whether a referee summary has been received, and its latest 0x0207 count
  bool have_shot_seq_{false};
  uint16_t last_shot_seq_{0};
  /// 当前处理帧的图像时间戳，单位 us
  /// Image timestamp of the frame being processed, in us
  uint64_t current_image_us_{0};
  /// 当前目标的装甲半径，单位 m
  /// Armor radius of the current target, in m
  double current_target_radius_{0.2};
  /// 上一帧是否在跟踪目标，用于判断开始跟踪的时刻
  /// Whether a target was tracked in the previous frame, to detect the start of tracking
  bool heat_fire_tracking_{false};
  /// 等待到达时刻观测的指向样本
  /// Pointing sample waiting for the observation at its arrival time
  struct LeadSample
  {
    uint64_t arrival_us;
    double gimbal_yaw;
    double command_yaw_rate;
  };
  /// 指向超前量的在线标定
  /// Online calibration of the pointing lead
  AimerDetail::LeadCalibrator lead_calibrator_;
  /// 等待到达时刻观测的指向样本，按到达时刻排序
  /// Pointing samples waiting for the observation at their arrival time, in arrival order
  std::deque<LeadSample> lead_pending_{};
  TinySolver* yaw_solver_{nullptr};
  TinySolver* roll_solver_{nullptr};
  mutable LibXR::Mutex gimbal_rotation_lock_{};
  mutable LibXR::Mutex runtime_log_lock_{};
  FrameSink frame_sink_{nullptr};
  void* frame_context_{nullptr};

  GimbalPlan gimbal_plan_msg_{};

  LibXR::Topic::Domain host_domain_ = LibXR::Topic::Domain("host");
  LibXR::Topic host_gimbal_topic_ =
      LibXR::Topic::CreateTopic<AimerHostGimbalTarget>("target_euler", &host_domain_);
  LibXR::Topic host_fire_topic_ =
      LibXR::Topic::CreateTopic<AimerHostFireNotify>("fire_notify", &host_domain_);
};

#include "AimerImpl.hpp"

/**
 * @brief Aimer 模块：订阅 `<相机名>_tracked`，发布云台目标、发射许可与 `<相机名>_aimed`。
 *        Aimer Module: subscribes to `<camera>_tracked`, publishes the gimbal target, the
 *        fire permission and `<camera>_aimed`.
 *
 * 在跟踪器的发布线程里同步处理，每收一帧发一帧。
 * Runs synchronously in the tracker's publishing thread, one frame out per frame in.
 */
class Aimer : public AimerCore
{
 public:
  using Config = AimerConfig;

  /// 全部取默认值的配置 / Configuration holding all defaults.
  static Config DefaultConfig() { return {}; }

  /**
   * @param camera_name 相机名，决定订阅与发布的 Topic / Camera name, selects the Topics
   * @param cfg 运行时配置 / Runtime configuration
   */
  explicit Aimer(std::string camera_name, Config cfg = {})
      : AimerCore(cfg),
        camera_name_(std::move(camera_name)),
        aimed_topic_(LibXR::Topic::CreateTopic<const AutoAim::AimedFrame*>(
            StageTopicName(camera_name_, AutoAim::STAGE_AIMED).c_str()))
  {
    SetFrameSink([](void* context, const AimerFrameState& state)
                 { static_cast<Aimer*>(context)->state_ = state; }, this);
    auto on_tracked = LibXR::Topic::Callback::Create(
        [](bool, Aimer* self, const AutoAim::TrackedFrame* frame)
        { self->OnTracked(*frame); }, this);
    AutoAim::RequireTopic<const AutoAim::TrackedFrame*>(
        StageTopicName(camera_name_, AutoAim::STAGE_TRACKED))
        .RegisterCallback(on_tracked);
  }

 private:
  void OnTracked(const AutoAim::TrackedFrame& frame)
  {
    state_ = {};
    UpdateGimbalRotationFromSyncedImu(frame.detected.synced.imu.rotation_wxyz);
    TargetCallback(frame.target);
    AutoAim::AimedFrame aimed{frame, {}};
    AutoAim::AimResult& aim = aimed.aim;
    aim.control = state_.control;
    aim.fire = state_.host_fire.isfire;
    aim.yaw = state_.host_gimbal.yaw;
    aim.pitch = state_.host_gimbal.pit;
    aim.aim_point = state_.aim_point_valid ? state_.aim_point : Eigen::Vector3d::Zero();
    aim.plate = state_.aim_point_valid ? state_.aim_armor_index : -1;
    const AutoAim::AimedFrame* payload = &aimed;
    aimed_topic_.Publish(payload);
  }

  std::string camera_name_;
  LibXR::Topic aimed_topic_;
  AimerFrameState state_{};
};
