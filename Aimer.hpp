#pragma once

/**
 * @file Aimer.hpp
 * @brief Aimer 模块的公开接口和 topic 载荷声明。
 */

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: ballistic aimer with DevC host target output
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
    min_fire_threshold: 0.003
    max_fire_threshold: 0.05
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
template_args:
  - Info:
      width: 1280
      height: 720
      step: 3840
      encoding: CameraTypes::Encoding::BGR8
      camera_matrix: [800.0, 0.0, 640.0, 0.0, 800.0, 360.0, 0.0, 0.0, 1.0]
      distortion_model: CameraTypes::DistortionModel::PLUMB_BOB
      distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]
      rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      projection_matrix: [800.0, 0.0, 640.0, 0.0, 0.0, 800.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]
required_hardware: []
depends:
  - qdu-future/ArmorTracker
  - qdu-future/VisionPreview
=== END MANIFEST === */
// clang-format on

#include <Eigen/Dense>
#include <atomic>
#include <cstdint>
#include <optional>

#include "ArmorTracker.hpp"
#include "ArmorTrackerTarget.hpp"
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
 * @brief 裁判系统发射机构反馈摘要。
 */
struct [[gnu::packed]] AimerRefereeLauncherData
{
  uint8_t bullet_type{};
  uint8_t launcher_id{};
  uint8_t bullet_freq{};
  float bullet_speed{};
};

/**
 * @brief host/robot_game_ref 的完整 31 字节裁判系统摘要载荷。
 */
struct [[gnu::packed]] AimerRefereeSummary
{
  AimerRefereeRobotStatus robot_status{};
  AimerRefereeGameStatus game_status{};
  AimerRefereeLauncherData launcher_data{};
};

static_assert(sizeof(AimerRefereeRobotStatus) == 13);
static_assert(sizeof(AimerRefereeGameStatus) == 11);
static_assert(sizeof(AimerRefereeLauncherData) == 7);
static_assert(sizeof(AimerRefereeSummary) == 31);

/**
 * @brief DevC HostData 接收的云台目标载荷。
 */
struct AimerHostGimbalTarget
{
  /// roll 命令，单位 rad。
  float rol{0.0f};
  /// pitch 命令，单位 rad。
  float pit{0.0f};
  /// yaw 命令，单位 rad。
  float yaw{0.0f};
  /// roll 速度前馈，单位 rad/s。
  float rol_dot{0.0f};
  /// pitch 速度前馈，单位 rad/s。
  float pit_dot{0.0f};
  /// yaw 速度前馈，单位 rad/s。
  float yaw_dot{0.0f};
  /// roll 加速度前馈，单位 rad/s^2。
  float rol_ddot{0.0f};
  /// pitch 加速度前馈，单位 rad/s^2。
  float pit_ddot{0.0f};
  /// yaw 加速度前馈，单位 rad/s^2。
  float yaw_ddot{0.0f};
};

static_assert(sizeof(AimerHostGimbalTarget) == sizeof(float) * 9);

/**
 * @brief DevC LauncherCMD 接收的发射许可载荷。
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
  /// 图像传感器时间戳，单位 us。
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
 * @brief 由 xrobot YAML 生成的 Aimer 运行时配置。
 */
struct AimerConfig
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
    /// 最小角度开火阈值，单位 rad。
    double min_fire_threshold{0.003};
    /// 最大角度开火阈值，单位 rad。
    double max_fire_threshold{0.05};
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
    /// Aimer 内置实时预览运行参数。
    VisionPreview::RuntimeParam preview{};
    /// 是否输出运行期统计日志。
    bool enable_runtime_log{true};
    /// 弹速变化超过该阈值时打印反馈日志，单位 m/s。
    double bullet_speed_log_delta{0.05};
    /// 热量变化超过该阈值时打印反馈日志。
    double heat_log_delta{1.0};
};

/**
 * @brief 选择目标装甲板、解算弹道 yaw/pitch，并发布云台命令。
 */
class AimerCore : public LibXR::Application
{
 public:
  using Config = AimerConfig;
  using PreviewSink = void (*)(void*, const AimerPreviewFrame&);

  /**
   * @brief 创建 Aimer 运行核心并注册裁判系统和云台反馈回调。
   */
  AimerCore(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
            Config cfg);

  /**
   * @brief LibXR 应用要求的周期监控钩子。
   */
  void OnMonitor() override {}

 protected:
  /**
   * @brief 设置由模板 Aimer 持有的 preview 状态接收器。
   */
  void SetPreviewSink(PreviewSink sink, void* context);
  /**
   * @brief 处理一帧 tracker 目标消息并发布 host 输出。
   */
  void TargetCallback(const ArmorTrackerTarget& target_msg);

 private:
  /**
   * @brief 注册裁判系统与发射机构运行期日志回调。
   */
  void RegisterRuntimeLogCallbacks();
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
  void LogFireState(const ArmorTrackerTarget& target_msg, bool fire,
                    double bullet_speed);
  /**
   * @brief 更新最新实测云台旋转。
   */
  void GimbalRotationCallback(LibXR::Quaternion<float> gimbal_rotation_msg);
  /**
   * @brief 提交本帧 Aimer 状态给内置 preview。
   */
  void PublishPreviewState(const AimerPreviewFrame& state);
  /**
   * @brief 根据命令稳定性和云台对准情况评估自动开火门控。
   */
  bool ShouldAutoFire(const Eigen::Vector3d& target_xyz, double selected_view_angle,
                      bool shootable, double yaw);
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
  bool has_gimbal_rotation_{false};
  LibXR::Quaternion<double> gimbal_rotation_{1.0, 0.0, 0.0, 0.0};
  bool planner_ready_{false};
  bool last_plan_mpc_{false};
  bool have_logged_fire_state_{false};
  bool last_logged_fire_state_{false};
  bool have_logged_bullet_speed_{false};
  double last_logged_bullet_speed_{0.0};
  bool have_logged_heat_status_{false};
  double last_logged_heat_{0.0};
  double last_logged_heat_limit_{0.0};
  double last_logged_cooling_{0.0};
  TinySolver* yaw_solver_{nullptr};
  TinySolver* pitch_solver_{nullptr};
  mutable LibXR::Mutex gimbal_rotation_lock_{};
  mutable LibXR::Mutex runtime_log_lock_{};
  PreviewSink preview_sink_{nullptr};
  void* preview_context_{nullptr};

  GimbalPlan gimbal_plan_msg_{};

  LibXR::Topic::Domain host_domain_ = LibXR::Topic::Domain("host");
  LibXR::Topic host_gimbal_topic_ =
      LibXR::Topic("target_euler", sizeof(AimerHostGimbalTarget), &host_domain_);
  LibXR::Topic host_fire_topic_ =
      LibXR::Topic("fire_notify", sizeof(AimerHostFireNotify), &host_domain_);
};

#include "AimerImpl.hpp"
#include "AimerPreview.hpp"

/**
 * @brief Aimer 对外 xrobot 模块，消费 tracker 同源目标帧并内联持有实时预览。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class Aimer : public AimerCore
{
 public:
  using Config = AimerConfig;
  using TargetFramePacket = ArmorTrackerTargetFramePacket<CameraInfoV>;
  using TargetFrameMessage = ArmorTrackerTargetFrameMessage<CameraInfoV>;
  using SourceFrame = ArmorDetectionsSourceFrame<CameraInfoV>;

  Aimer(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, Config cfg)
      : AimerCore(hw, app, cfg)
  {
    if (cfg.preview.enabled)
    {
      preview_.emplace(hw, app, AimerDetail::MakeAimerPreviewConfig(cfg));
      SetPreviewSink(
          [](void* context, const AimerPreviewFrame& frame)
          {
            static_cast<Aimer*>(context)->SubmitPreviewFrame(frame);
          },
          this);
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
        LibXR::Topic::FindOrCreate<TargetFrameMessage>("target_frame",
                                                       &tracker_domain);
    auto callback = LibXR::Topic::Callback::Create(
        [](bool, Aimer* self, LibXR::RawData& data)
        {
          auto* message = reinterpret_cast<TargetFrameMessage*>(data.addr_);
          if (message == nullptr || data.size_ != sizeof(TargetFrameMessage) ||
              *message == nullptr || (*message)->target == nullptr)
          {
            return;
          }
          self->TargetFrameCallback(**message);
        },
        this);
    target_frame_topic_.RegisterCallback(callback);
  }

  /**
   * @brief 处理 tracker 同帧目标和源图像。
   */
  void TargetFrameCallback(const TargetFramePacket& frame)
  {
    current_source_frame_ = &frame.source_frame;
    TargetCallback(*frame.target);
    current_source_frame_ = nullptr;
  }

  /**
   * @brief 将 Core 生成的预览状态交给内置 preview。
   */
  void SubmitPreviewFrame(const AimerPreviewFrame& frame)
  {
    if (preview_.has_value())
    {
      preview_->OnAimerFrame(frame, current_source_frame_);
    }
  }

  LibXR::Topic target_frame_topic_ = LibXR::Topic();
  const SourceFrame* current_source_frame_{nullptr};
  std::optional<AimerPreview<CameraInfoV>> preview_;
};
