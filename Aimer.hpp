#pragma once

/**
 * @file Aimer.hpp
 * @brief Aimer 模块的公开接口和 topic 数据声明。
 */

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: ballistic aimer with DevC host target output
constructor_args:
  cfg:
    yaw_offset: -1.0
    roll_offset: -1.4
    yaw_rate_threshold: 2.0
    default_bullet_speed: 21.0
    min_valid_bullet_speed: 14.0
    ballistic_drag_k: 0.02
    ballistic_integration_dt_s: 0.001
    ballistic_max_iterations: 16
    ballistic_min_elevation_deg: -20.0
    ballistic_max_elevation_deg: 35.0
    auto_fire: true
    image_to_now_s: 0.0
    vision_to_command_delay_s: 0.0
    command_transport_delay_s: 0.0
    gimbal_response_delay_s: 0.0
    fire_delay_s: 0.0
    low_speed_extra_predict_s: 0.015
    high_speed_extra_predict_s: 0.03
    min_fire_threshold: 0.003
    max_fire_threshold: 0.05
    enable_mpc_plan: true
    mpc_fire_thresh: 0.05
    max_yaw_acc: 50.0
    q_yaw_pos: 9000000.0
    q_yaw_vel: 0.0
    r_yaw_acc: 1.0
    max_roll_acc: 100.0
    q_roll_pos: 9000000.0
    q_roll_vel: 0.0
    r_roll_acc: 1.0
    preview:
      enabled: false
      preview_window_name: "aimer_preview"
      preview_scale: 0.5
      preview_wait_key_ms: 1
      queue_capacity: 1
      output_mode: "window"
      web_bind_address: "0.0.0.0"
      web_port: 8080
      web_stream_name: "aimer_preview"
      max_fps: 30.0
    enable_runtime_log: true
    bullet_speed_log_delta: 0.05
    heat_log_delta: 1.0
    convert_raw_gimbal_quat_to_body: false
  calibration:
    native_width: 1280
    native_height: 720
    camera_matrix: [800.0, 0.0, 640.0, 0.0, 800.0, 360.0, 0.0, 0.0, 1.0]
    distortion_model: CameraTypes::DistortionModel::PLUMB_BOB
    distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]
    rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    projection_matrix: [800.0, 0.0, 640.0, 0.0, 0.0, 800.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]
template_args:
  - Layout:
      width: 1280
      height: 720
      step: 3840
      encoding: CameraTypes::Encoding::BGR8
required_hardware: []
depends:
  - qdu-future/ArmorTracker
  - qdu-future/CameraBase
  - qdu-future/VisionPreview
  - xrobot-org/DurationStatistics
=== END MANIFEST === */
// clang-format on

#include <Eigen/Dense>
#include <atomic>
#include <cstdint>
#include <optional>
#include <utility>

#include "ArmorTrackerTarget.hpp"
#include "CameraBase.hpp"
#include "DurationStatistics.hpp"
#include "GimbalPlan.hpp"
#include "VisionPreview.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "mutex.hpp"
#include "tinympc/tiny_api.hpp"

/**
 * @brief 裁判系统机器人状态摘要。
 */
struct [[gnu::packed]] AimerRefereeRobotStatus
{
  uint8_t robot_id{};
  uint8_t robot_level{};
  uint16_t remain_hp{};
  uint16_t max_hp{};
  uint16_t shooter_cooling_value{};
  uint16_t shooter_heat_limit{};
  uint16_t chassis_power_limit{};
  uint8_t power_gimbal_output : 1 {};
  uint8_t power_chassis_output : 1 {};
  uint8_t power_launcher_output : 1 {};
};

/**
 * @brief 裁判系统比赛状态摘要。
 */
struct [[gnu::packed]] AimerRefereeGameStatus
{
  uint8_t game_type : 4 {};
  uint8_t game_progress : 4 {};
  uint16_t stage_remain_time{};
  uint64_t sync_time_stamp{};
};

/**
 * @brief host/robot_game_ref 的比赛旧 BSP 92 字节裁判系统摘要数据。
 *
 * 该 robot_game_ref 实际对应旧比赛 BSP 的 RobotGameRefereePack。Aimer 当前只显式消费
 * RobotStatus/GameStatus 前缀，其余字段保留为不透明尾部，只用于对齐 ABI。
 */
struct [[gnu::packed]] AimerRefereeSummary
{
  AimerRefereeRobotStatus robot_status{};
  AimerRefereeGameStatus game_status{};
  uint8_t reserved_tail[68]{};
};

static_assert(sizeof(AimerRefereeRobotStatus) == 13);
static_assert(sizeof(AimerRefereeGameStatus) == 11);
static_assert(sizeof(AimerRefereeSummary) == 92);

/**
 * @brief DevC HostData 接收的云台目标数据。
 */
struct AimerHostGimbalTarget
{
  /// 机械俯仰轴 roll 命令，单位 rad。
  float rol{0.0f};
  /// 兼容旧 C 板接口，镜像当前机械俯仰轴 roll 命令。
  float pit{0.0f};
  /// yaw 命令，单位 rad。
  float yaw{0.0f};
  /// 机械俯仰轴 roll 速度前馈，单位 rad/s。
  float rol_dot{0.0f};
  /// 兼容旧 C 板接口，镜像当前机械俯仰轴 roll 速度前馈。
  float pit_dot{0.0f};
  /// yaw 速度前馈，单位 rad/s。
  float yaw_dot{0.0f};
  /// 机械俯仰轴 roll 加速度前馈，单位 rad/s^2。
  float rol_ddot{0.0f};
  /// 兼容旧 C 板接口，镜像当前机械俯仰轴 roll 加速度前馈。
  float pit_ddot{0.0f};
  /// yaw 加速度前馈，单位 rad/s^2。
  float yaw_ddot{0.0f};
};

static_assert(sizeof(AimerHostGimbalTarget) == sizeof(float) * 9);

/**
 * @brief DevC LauncherCMD 接收的发射许可数据。
 */
struct AimerHostFireNotify
{
  /// 是否允许发射。
  bool isfire{false};
};

static_assert(sizeof(AimerHostFireNotify) == 1);

/**
 * @brief Aimer 内置预览绘制所需的同帧状态。
 */
struct AimerPreviewFrame
{
  /// 匹配触发沿的 MCU 陀螺仪时间戳，单位 us。
  uint64_t image_timestamp_us{};
  /// 当前帧是否带有 tracker 目标消息。
  bool have_target{false};
  /// tracker 原始目标消息。
  ArmorTrackerTarget target{};
  /// 是否存在 Aimer 选中的预测瞄点。
  bool aim_point_valid{false};
  /// 预测后的瞄点中心，使用 tracker 输出坐标系。
  Eigen::Vector3d aim_point{Eigen::Vector3d::Zero()};
  /// 预测选中的装甲板索引。
  int aim_armor_index{-1};
  /// 预测选中的装甲板中心和 yaw。
  Eigen::Vector4d aim_xyza{Eigen::Vector4d::Zero()};
  /// 是否已经生成 host/fire_notify 输出。
  bool have_host_fire{false};
  /// 本帧发射许可输出。
  AimerHostFireNotify host_fire{};
};

/**
 * @brief 单发弹丸对应的未来命中候选。
 */
struct AimerShotCandidate
{
  /// 候选是否有效。
  bool valid{false};
  /// 命中时刻该装甲板姿态是否允许开火。
  bool face_shootable_at_hit{false};
  /// 命中时刻对应的物理装甲板索引。
  int hit_face{-1};
  /// 命中时刻装甲板相对整车中心方位的视角，单位 rad。
  double view_angle{0.0};
  /// 命中时刻装甲板中心 x、y、z 和装甲板 yaw。
  Eigen::Vector4d hit_xyza{Eigen::Vector4d::Zero()};
  /// 指向该命中候选的 yaw，单位 rad。
  double yaw{0.0};
  /// 指向该命中候选的机械 roll 轴命令，单位 rad。
  double roll{0.0};
  /// 命中候选对应的弹丸飞行时间，单位 s。
  double fly_time{0.0};
};

/**
 * @brief 由 xrobot YAML 生成的 Aimer 运行时配置。
 */
struct AimerConfig
{
  /// 施加到弹道命令上的固定 yaw 偏置，单位 deg。
  double yaw_offset{-1.0};
  /// 施加到机械 roll 轴命令上的固定偏置，单位 deg。
  double roll_offset{-1.4};
  /// 低速与旋转策略的 yaw 角速度阈值，单位 rad/s。
  double yaw_rate_threshold{2.0};
  /// 弹速异常时使用的默认弹速，单位 m/s。
  double default_bullet_speed{21.0};
  /// 可接受的最小裁判系统弹速，单位 m/s。
  double min_valid_bullet_speed{14.0};
  /// 二次阻力加速度系数，a_drag = -k * |v| * v。
  double ballistic_drag_k{0.02};
  /// RK4 弹道积分步长，单位 s。
  double ballistic_integration_dt_s{0.001};
  /// 一维括区求根最大迭代次数。
  int ballistic_max_iterations{16};
  /// 允许搜索的最小弹道仰角，单位 deg。
  double ballistic_min_elevation_deg{-20.0};
  /// 允许搜索的最大弹道仰角，单位 deg。
  double ballistic_max_elevation_deg{35.0};
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
  /// 低速目标的预测延迟补偿。
  double low_speed_extra_predict_s{0.015};
  /// yaw 角速度超过阈值时的预测延迟补偿。
  double high_speed_extra_predict_s{0.03};
  /// 最小角度开火阈值，单位 rad。
  double min_fire_threshold{0.003};
  /// 最大角度开火阈值，单位 rad。
  double max_fire_threshold{0.05};
  /// 是否启用 TinyMPC 云台计划。
  bool enable_mpc_plan{true};
  /// 允许使用 MPC 输出和开火的最大计划偏离，单位 rad。
  double mpc_fire_thresh{0.05};
  /// TinyMPC yaw 加速度约束，单位 rad/s^2。
  double max_yaw_acc{50.0};
  /// TinyMPC yaw 位置代价。
  double q_yaw_pos{9000000.0};
  /// TinyMPC yaw 速度代价。
  double q_yaw_vel{0.0};
  /// TinyMPC yaw 加速度代价。
  double r_yaw_acc{1.0};
  /// TinyMPC roll 轴加速度约束，单位 rad/s^2。
  double max_roll_acc{100.0};
  /// TinyMPC roll 轴位置代价。
  double q_roll_pos{9000000.0};
  /// TinyMPC roll 轴速度代价。
  double q_roll_vel{0.0};
  /// TinyMPC roll 轴加速度代价。
  double r_roll_acc{1.0};
  /// Aimer 内置实时预览运行参数。
  VisionPreview::RuntimeParam preview{};
  /// 是否输出运行期统计日志。
  bool enable_runtime_log{true};
  /// 弹速变化超过该阈值时打印反馈日志，单位 m/s。
  double bullet_speed_log_delta{0.05};
  /// 热量变化超过该阈值时打印反馈日志。
  double heat_log_delta{1.0};
  /// 是否把原始 x 前、y 左、z 上的 ahrs_quaternion 转到公开 body 轴。
  bool convert_raw_gimbal_quat_to_body{false};
};

/**
 * @brief 选择目标装甲板、解算 yaw/roll 轴命令，并发布云台命令。
 */
class AimerCore : public LibXR::Application
{
 public:
  using Config = AimerConfig;
  using PreviewSink = void (*)(void*, const AimerPreviewFrame&);

  /**
   * @brief 创建 Aimer 运行核心并注册裁判系统和云台反馈回调。
   */
  AimerCore(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, Config cfg);

  /**
   * @brief 输出 tracker 目标回调的累计耗时统计。
   */
  void OnMonitor() override;

 protected:
  /**
   * @brief 设置由模板 Aimer 持有的 preview 状态接收器。
   */
  void SetPreviewSink(PreviewSink sink, void* context);
  /**
   * @brief 使用 tracker 同帧同步 IMU 更新当前云台姿态。
   */
  void UpdateGimbalRotationFromSyncedImu(const std::array<float, 4>& rotation_wxyz);
  /**
   * @brief 清空当前可用的云台姿态，禁止自动开火门控继续使用旧姿态。
   */
  void ClearGimbalRotation();
  /**
   * @brief 处理一帧 tracker 目标消息并发布 host 输出。
   */
  void TargetCallback(const ArmorTrackerTarget& target_msg);

 private:
  /**
   * @brief 注册裁判系统与云台姿态输入回调。
   */
  void RegisterHostInputCallbacks();
  /**
   * @brief 注册 C 板回传的云台姿态四元数输入 topic。
   */
  void RegisterGimbalQuatInput();
  /**
   * @brief 注册裁判系统摘要输入 topic。
   */
  void RegisterRefereeSummaryInput();
  /**
   * @brief 统一更新弹速缓存并按变化量输出日志。
   */
  void UpdateBulletSpeed(float bullet_speed_msg, const char* source);
  /**
   * @brief 处理裁判系统摘要反馈。
   */
  void RefereeSummaryCallback(const AimerRefereeSummary& summary);
  /**
   * @brief 按变化量记录热量、热量上限和冷却值。
   */
  void LogHeatStatus(double current_heat, double heat_limit, double cooling,
                     const char* source, bool force);
  /**
   * @brief 在自动开火状态翻转时输出统计日志。
   */
  void LogFireState(const ArmorTrackerTarget& target_msg, bool fire, double bullet_speed);
  /**
   * @brief 更新最新实测云台旋转。
   */
  void GimbalRotationCallback(LibXR::Quaternion<float> gimbal_rotation_msg);
  /**
   * @brief 提交本帧 Aimer 状态给内置 preview。
   */
  void PublishPreviewState(const AimerPreviewFrame& state);
  /**
   * @brief 根据计划命令稳定性和云台两轴对准情况评估自动开火门控。
   */
  bool ShouldAutoFire(const AimerShotCandidate& shot_candidate, bool plan_fire_enabled,
                      double yaw, double roll);
  /**
   * @brief 初始化 yaw 和 roll 轴 TinyMPC 求解器。
   */
  void SetupGimbalPlanSolvers();
  /**
   * @brief 尝试为当前目标构建 TinyMPC 云台计划。
   */
  bool BuildMpcGimbalPlan(const ArmorTrackerTarget& target_msg, double delay_time,
                          double bullet_speed, AimerShotCandidate& fire_shot_candidate);
  /**
   * @brief 在 TinyMPC 关闭或不可用时构建直接 yaw/roll 计划。
   */
  void BuildFiniteDifferenceGimbalPlan(const ArmorTrackerTarget& target_msg, bool control,
                                       bool fire_enabled, double yaw, double roll);
  /**
   * @brief 在 TinyMPC 和直接云台计划之间选择。
   */
  void BuildGimbalPlan(const ArmorTrackerTarget& target_msg, double delay_time,
                       bool control, double yaw, double roll, double bullet_speed,
                       const AimerShotCandidate& direct_shot_candidate,
                       AimerShotCandidate& fire_shot_candidate);
  /**
   * @brief 清理与上一目标相关的规划器状态。
   */
  void ResetGimbalPlanHistory();

 private:
  Config cfg_{};
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
 * @brief Aimer 对外 xrobot 模块，消费 tracker 同源目标帧并内联持有实时预览。
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
   * @brief 构造瞄准模块，并固定预览使用的原生相机标定。
   *
   * @param calibration 原生传感器坐标系下的不可变相机标定，按值持有。
   */
  Aimer(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, Config cfg,
        CameraCalibration calibration)
      : AimerCore(hw, app, cfg), calibration_(std::move(calibration))
  {
    ASSERT(CameraBaseIntrinsicSanity::CameraCalibrationReasonable(calibration_));
    if (cfg.preview.enabled)
    {
      preview_.emplace(hw, app, AimerDetail::MakeAimerPreviewConfig(cfg), calibration_);
      SetPreviewSink([](void* context, const AimerPreviewFrame& frame)
                     { static_cast<Aimer*>(context)->SubmitPreviewFrame(frame); }, this);
    }
    RegisterTargetFrameCallback();
  }

 private:
  /**
   * @brief 注册带同源图像帧的 tracker/target_frame topic。
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
   * @brief 处理 tracker 同帧目标和源图像。
   */
  void TargetFrameCallback(const TargetFrame& frame)
  {
    current_target_frame_ = &frame;
    UpdateGimbalRotationFromSyncedImu(frame.imu.rotation_wxyz);
    TargetCallback(frame.target);
    current_target_frame_ = nullptr;
  }

  /**
   * @brief 将 Core 生成的预览状态交给内置 preview。
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
