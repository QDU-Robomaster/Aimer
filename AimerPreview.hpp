#pragma once

/**
 * @file AimerPreview.hpp
 * @brief Aimer 内部实时预览实现。
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "AimerTargetModel.hpp"
#include "CameraFrameSync.hpp"
#include "VisionPreview.hpp"

/**
 * @brief Aimer 实时预览模块配置。
 */
struct AimerPreviewConfig
{
  /// 预览子系统运行参数。
  VisionPreview::RuntimeParam preview{};
  /// 等待匹配 Aimer 输出的最大时间，单位 ms。
  uint32_t sync_wait_ms{AimerDetail::PREVIEW_SYNC_WAIT_MS};
};

namespace AimerDetail
{
/**
 * @brief 从 Aimer 主配置中提取内部预览配置。
 */
inline AimerPreviewConfig MakeAimerPreviewConfig(const AimerConfig& cfg)
{
  AimerPreviewConfig preview_cfg{};
  preview_cfg.preview = cfg.preview;
  return preview_cfg;
}
}  // namespace AimerDetail

/**
 * @brief Aimer 预览：从相机同步帧和 tracker/Aimer topic 绘制实时结果。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class AimerPreview : public LibXR::Application
{
 public:
  using FrameSync = CameraFrameSync<CameraInfoV>;
  using ImageFrame = typename FrameSync::ImageFrame;
  using SyncedFrame = typename FrameSync::SyncedFrame;

  /**
   * @brief 构造 Aimer preview 并注册 topic 回调。
   */
  AimerPreview(LibXR::HardwareContainer&, LibXR::ApplicationManager& app,
               AimerPreviewConfig cfg, FrameSync& sync)
      : cfg_(std::move(cfg)),
        subscriber_(sync),
        preview_(cfg_.preview)
  {
    RegisterTopics();
    if (preview_.Running())
    {
      running_.store(true, std::memory_order_release);
      worker_thread_ = std::thread(&AimerPreview::WorkerMain, this);
    }
    app.Register(*this);
  }

  ~AimerPreview() { Stop(); }

  /**
   * @brief Aimer preview 不需要周期监控。
   */
  void OnMonitor() override {}

 private:
  struct Snapshot
  {
    uint64_t image_timestamp_us{0};
    bool have_target{false};
    bool have_host_target{false};
    bool have_host_fire{false};
    ArmorTrackerTarget target{};
    AimerHostGimbalTarget host_target{};
    AimerHostFireNotify host_fire{};
  };

  /**
   * @brief 注册 tracker 与 host topic 回调。
   */
  void RegisterTopics()
  {
    LibXR::Topic::Domain tracker_domain("tracker");
    LibXR::Topic::Domain host_domain("host");

    LibXR::Topic target_topic =
        LibXR::Topic::FindOrCreate<ArmorTrackerTarget>("target", &tracker_domain);
    auto target_callback = LibXR::Topic::Callback::Create(
        [](bool, AimerPreview* self, LibXR::RawData& data)
        {
          auto* target = reinterpret_cast<ArmorTrackerTarget*>(data.addr_);
          if (target != nullptr && data.size_ == sizeof(ArmorTrackerTarget))
          {
            self->OnTarget(*target);
          }
        },
        this);
    target_topic.RegisterCallback(target_callback);

    LibXR::Topic host_target_topic =
        LibXR::Topic::FindOrCreate<AimerHostGimbalTarget>("target_euler",
                                                          &host_domain);
    auto host_target_callback = LibXR::Topic::Callback::Create(
        [](bool, AimerPreview* self, LibXR::RawData& data)
        {
          auto* target = reinterpret_cast<AimerHostGimbalTarget*>(data.addr_);
          if (target != nullptr && data.size_ == sizeof(AimerHostGimbalTarget))
          {
            self->OnHostTarget(*target);
          }
        },
        this);
    host_target_topic.RegisterCallback(host_target_callback);

    LibXR::Topic host_fire_topic =
        LibXR::Topic::FindOrCreate<AimerHostFireNotify>("fire_notify", &host_domain);
    auto host_fire_callback = LibXR::Topic::Callback::Create(
        [](bool, AimerPreview* self, LibXR::RawData& data)
        {
          auto* fire = reinterpret_cast<AimerHostFireNotify*>(data.addr_);
          if (fire != nullptr && data.size_ == sizeof(AimerHostFireNotify))
          {
            self->OnHostFire(*fire);
          }
        },
        this);
    host_fire_topic.RegisterCallback(host_fire_callback);
  }

  /**
   * @brief 更新最新 tracker 目标缓存。
   */
  void OnTarget(const ArmorTrackerTarget& target)
  {
    LibXR::Mutex::LockGuard lock(snapshot_lock_);
    target_snapshot_ = target;
    have_target_ = true;
    Snapshot& snapshot = MutableSnapshot(target.image_timestamp_us);
    snapshot.have_target = true;
    snapshot.target = target;
  }

  /**
   * @brief 更新最新 host 云台目标缓存。
   */
  void OnHostTarget(const AimerHostGimbalTarget& target)
  {
    LibXR::Mutex::LockGuard lock(snapshot_lock_);
    host_target_snapshot_ = target;
    have_host_target_ = true;
  }

  /**
   * @brief 更新最新 host 发射许可缓存。
   */
  void OnHostFire(const AimerHostFireNotify& fire)
  {
    LibXR::Mutex::LockGuard lock(snapshot_lock_);
    host_fire_snapshot_ = fire;
    have_host_fire_ = true;
  }

  /**
   * @brief 取得或创建指定图像时间戳的短历史快照。
   */
  Snapshot& MutableSnapshot(uint64_t image_timestamp_us)
  {
    for (auto& snapshot : snapshot_history_)
    {
      if (snapshot.image_timestamp_us == image_timestamp_us)
      {
        return snapshot;
      }
    }
    snapshot_history_.push_back({});
    Snapshot& snapshot = snapshot_history_.back();
    snapshot.image_timestamp_us = image_timestamp_us;
    while (snapshot_history_.size() > kMaxSnapshotHistory)
    {
      snapshot_history_.pop_front();
    }
    return snapshot;
  }

  /**
   * @brief 提取与当前图像时间戳匹配的 tracker/Aimer 快照。
   */
  bool CollectSnapshot(uint64_t image_timestamp_us, Snapshot& snapshot)
  {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(cfg_.sync_wait_ms);
    do
    {
      {
        LibXR::Mutex::LockGuard lock(snapshot_lock_);
        for (const auto& cached : snapshot_history_)
        {
          if (cached.image_timestamp_us == image_timestamp_us)
          {
            snapshot = cached;
            break;
          }
        }
        if (!snapshot.have_host_target && have_host_target_)
        {
          snapshot.have_host_target = true;
          snapshot.host_target = host_target_snapshot_;
        }
        if (!snapshot.have_host_fire && have_host_fire_)
        {
          snapshot.have_host_fire = true;
          snapshot.host_fire = host_fire_snapshot_;
        }
        const bool has_any = snapshot.have_target || snapshot.have_host_target ||
                             snapshot.have_host_fire;
        const bool complete =
            (snapshot.have_host_target && snapshot.have_host_fire) ||
            (snapshot.have_target && !snapshot.target.tracking);
        if (complete || (has_any && std::chrono::steady_clock::now() >= deadline))
        {
          return true;
        }
      }
      if (std::chrono::steady_clock::now() >= deadline)
      {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (true);
    return false;
  }

  /**
   * @brief 将 tracker output frame 点转到 world frame。
   */
  static Eigen::Vector3d OutputFrameToWorld(const Eigen::Vector3d& point)
  {
    return {point.z(), -point.x(), -point.y()};
  }

  /**
   * @brief 将 tracker output frame 装甲板 yaw 转到 world frame。
   */
  static double OutputYawToWorld(double yaw)
  {
    return AimerDetail::LimitRad(yaw + AimerDetail::PI * 0.5);
  }

  /**
   * @brief 相机光轴坐标转像素。
   */
  static bool ProjectOpticalPoint(const Eigen::Vector3d& point, cv::Point& uv)
  {
    const double x = point.x();
    const double y = point.y();
    const double depth = point.z();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(depth) ||
        depth <= 1e-6)
    {
      return false;
    }
    const double u = static_cast<double>(CameraInfoV.camera_matrix[0]) * x / depth +
                     static_cast<double>(CameraInfoV.camera_matrix[2]);
    const double v = static_cast<double>(CameraInfoV.camera_matrix[4]) * y / depth +
                     static_cast<double>(CameraInfoV.camera_matrix[5]);
    if (!std::isfinite(u) || !std::isfinite(v))
    {
      return false;
    }
    uv = cv::Point(static_cast<int>(std::lround(u)),
                   static_cast<int>(std::lround(v)));
    return true;
  }

  /**
   * @brief world frame 点转到当前图像的相机光轴坐标。
   */
  static Eigen::Vector3d WorldToOptical(const Eigen::Vector3d& point_world,
                                        const Eigen::Quaterniond& q_gimbal_to_world)
  {
    static const Eigen::Matrix3d kCameraToGimbal =
        (Eigen::Matrix3d() << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0)
            .finished();
    return kCameraToGimbal.transpose() *
           q_gimbal_to_world.toRotationMatrix().transpose() * point_world;
  }

  /**
   * @brief 将 tracker output frame 点按当前 IMU 姿态投影到图像像素。
   */
  static bool ProjectOutputPoint(const Eigen::Vector3d& point,
                                 const Eigen::Quaterniond& q_gimbal_to_world,
                                 cv::Point& uv)
  {
    return ProjectOpticalPoint(
        WorldToOptical(OutputFrameToWorld(point), q_gimbal_to_world), uv);
  }

  /**
   * @brief 将 host 角度命令转换成 tracker output frame 中的预览点。
   */
  static std::optional<Eigen::Vector3d> HostTargetPreviewPoint(
      const Snapshot& snapshot)
  {
    if (!snapshot.have_host_target)
    {
      return std::nullopt;
    }

    const auto& target = snapshot.host_target;
    if (!std::isfinite(target.pit) || !std::isfinite(target.yaw))
    {
      return std::nullopt;
    }

    if (std::abs(target.pit) < 1e-6f && std::abs(target.yaw) < 1e-6f)
    {
      return std::nullopt;
    }

    double horizontal_distance = 1.0;
    if (snapshot.have_target && snapshot.target.tracking)
    {
      horizontal_distance = std::max(
          0.2, AimerDetail::HorizontalDistance(snapshot.target.position));
    }

    return Eigen::Vector3d(horizontal_distance * std::cos(target.yaw),
                           horizontal_distance * std::tan(-target.pit),
                           horizontal_distance * std::sin(target.yaw));
  }

  /**
   * @brief 依据 target 的几何状态构造单个装甲面的四边形顶点。
   */
  std::array<Eigen::Vector3d, 4> BuildArmorQuadWorld(
      const Eigen::Vector4d& xyza, const ArmorTrackerTarget& target) const
  {
    const Eigen::Vector3d center = OutputFrameToWorld(xyza.head<3>());
    const double yaw = OutputYawToWorld(xyza[3]);
    const Eigen::Vector3d width_dir(-std::sin(yaw), std::cos(yaw), 0.0);
    const double pitch = AimerDetail::ARMOR_PITCH_DEG * AimerDetail::DEG2RAD;
    const double pitch_sign =
        target.id == ArmorNumber::OUTPOST ? -1.0 : 1.0;
    const Eigen::Vector3d height_dir =
        Eigen::Vector3d(std::cos(yaw) * pitch_sign * std::sin(pitch),
                        std::sin(yaw) * pitch_sign * std::sin(pitch),
                        std::cos(pitch));
    const Eigen::Vector3d half_w =
        0.5 * AimerDetail::SMALL_ARMOR_WIDTH_M * width_dir;
    const Eigen::Vector3d half_h = 0.5 * AimerDetail::ARMOR_HEIGHT_M * height_dir;
    return {center - half_w + half_h, center + half_w + half_h,
            center + half_w - half_h, center - half_w - half_h};
  }

  /**
   * @brief 从 tracker target 和 host 输出画出预览。
   */
  void DrawPreview(cv::Mat& canvas, const Snapshot& snapshot,
                   const Eigen::Quaterniond& q_gimbal_to_world)
  {
    if (snapshot.have_target && snapshot.target.tracking)
    {
      AimerDetail::PredictedTarget predicted{snapshot.target};
      const auto armor_xyza_list = predicted.GetArmorXYZAList();
      for (std::size_t i = 0; i < armor_xyza_list.size(); ++i)
      {
        const bool selected =
            static_cast<int>(i) == snapshot.target.tracked_face_index;
        const auto quad = BuildArmorQuadWorld(armor_xyza_list[i], snapshot.target);
        std::array<cv::Point, 4> corners{};
        bool valid = true;
        for (std::size_t corner = 0; corner < quad.size(); ++corner)
        {
          valid = valid &&
                  ProjectOpticalPoint(WorldToOptical(quad[corner], q_gimbal_to_world),
                                      corners[corner]);
        }
        if (!valid)
        {
          continue;
        }
        const cv::Scalar body_color =
            selected ? cv::Scalar(80, 255, 80) : cv::Scalar(255, 180, 40);
        const cv::Scalar face_color =
            selected ? cv::Scalar(255, 120, 40) : cv::Scalar(190, 220, 255);
        for (int edge = 0; edge < 4; ++edge)
        {
          cv::line(canvas, corners[static_cast<std::size_t>(edge)],
                   corners[static_cast<std::size_t>((edge + 1) % 4)], body_color,
                   selected ? 2 : 1, cv::LINE_AA);
        }
        cv::Point center_uv;
        if (ProjectOutputPoint(armor_xyza_list[i].head<3>(), q_gimbal_to_world,
                               center_uv))
        {
          cv::circle(canvas, center_uv, selected ? 5 : 4, face_color, -1,
                     cv::LINE_AA);
          cv::putText(canvas, "E" + std::to_string(i), center_uv + cv::Point(6, 14),
                      cv::FONT_HERSHEY_SIMPLEX, 0.45, face_color, 1, cv::LINE_AA);
        }
      }

      cv::Point target_center_uv;
      if (ProjectOutputPoint(snapshot.target.position, q_gimbal_to_world,
                             target_center_uv))
      {
        cv::drawMarker(canvas, target_center_uv, cv::Scalar(255, 255, 255),
                       cv::MARKER_CROSS, 20, 2, cv::LINE_AA);
      }
    }

    cv::Point aim_uv;
    const bool host_fire = snapshot.have_host_fire && snapshot.host_fire.isfire;
    const auto host_target_point = HostTargetPreviewPoint(snapshot);
    if (host_target_point.has_value() &&
        ProjectOutputPoint(*host_target_point, q_gimbal_to_world, aim_uv))
    {
      cv::drawMarker(canvas, aim_uv, cv::Scalar(0, 0, 255), cv::MARKER_TILTED_CROSS,
                     host_fire ? 22 : 18, 2, cv::LINE_AA);
      if (host_fire)
      {
        cv::circle(canvas, aim_uv, 10, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
      }
    }

    const std::string id_name =
        snapshot.have_target &&
                static_cast<std::size_t>(snapshot.target.id) < ARMOR_NUMBER_NAMES.size()
            ? std::string(ARMOR_NUMBER_NAMES[static_cast<std::size_t>(snapshot.target.id)])
            : std::string("invalid");
    const std::string header =
        std::string("aimer ") +
        (snapshot.have_target && snapshot.target.tracking ? "TRACK" : "NO_TARGET") +
        " id=" + id_name +
        " fire=" + std::to_string(host_fire ? 1 : 0);
    cv::putText(canvas, header, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.75,
                cv::Scalar(40, 240, 40), 2, cv::LINE_AA);
  }

  /**
   * @brief worker 线程主循环。
   */
  void WorkerMain()
  {
    while (running_.load(std::memory_order_acquire))
    {
      SyncedFrame frame;
      const auto wait_ans = subscriber_.Wait(frame, 100);
      if (!running_.load(std::memory_order_acquire))
      {
        break;
      }
      if (wait_ans != LibXR::ErrorCode::OK)
      {
        continue;
      }
      const auto* image_frame = frame.GetImageFrame();
      if (image_frame == nullptr)
      {
        continue;
      }
      SubmitPreview(frame);
    }
  }

  /**
   * @brief 从同步帧提取 gimbal-to-world 姿态。
   */
  static Eigen::Quaterniond FrameQuaternion(const SyncedFrame& frame)
  {
    Eigen::Quaterniond q(frame.imu.rotation_wxyz[0], frame.imu.rotation_wxyz[1],
                         frame.imu.rotation_wxyz[2], frame.imu.rotation_wxyz[3]);
    if (!std::isfinite(q.norm()) || q.norm() < 1e-9)
    {
      return Eigen::Quaterniond::Identity();
    }
    q.normalize();
    return q;
  }

  /**
   * @brief 提交当前帧到 VisionPreview。
   */
  void SubmitPreview(const SyncedFrame& frame)
  {
    if (!preview_.Running())
    {
      return;
    }
    const auto* image_frame_ptr = frame.GetImageFrame();
    if (image_frame_ptr == nullptr)
    {
      return;
    }
    const ImageFrame& image_frame = *image_frame_ptr;
    const Eigen::Quaterniond q_gimbal_to_world = FrameQuaternion(frame);

    int cv_type = -1;
    switch (CameraInfoV.encoding)
    {
      case CameraTypes::Encoding::RGB8:
      case CameraTypes::Encoding::BGR8:
        cv_type = CV_8UC3;
        break;
      case CameraTypes::Encoding::RGBA8:
      case CameraTypes::Encoding::BGRA8:
        cv_type = CV_8UC4;
        break;
      case CameraTypes::Encoding::MONO8:
        cv_type = CV_8UC1;
        break;
      default:
        return;
    }
    cv::Mat image(static_cast<int>(CameraInfoV.height),
                  static_cast<int>(CameraInfoV.width), cv_type,
                  const_cast<uint8_t*>(image_frame.data.data()),
                  static_cast<size_t>(CameraInfoV.step));
    cv::Mat bgr_image;
    if (CameraInfoV.encoding == CameraTypes::Encoding::BGR8)
    {
      bgr_image = image;
    }
    else if (CameraInfoV.encoding == CameraTypes::Encoding::RGB8)
    {
      cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
    }
    else if (CameraInfoV.encoding == CameraTypes::Encoding::RGBA8)
    {
      cv::cvtColor(image, bgr_image, cv::COLOR_RGBA2BGR);
    }
    else if (CameraInfoV.encoding == CameraTypes::Encoding::BGRA8)
    {
      cv::cvtColor(image, bgr_image, cv::COLOR_BGRA2BGR);
    }
    else if (CameraInfoV.encoding == CameraTypes::Encoding::MONO8)
    {
      cv::cvtColor(image, bgr_image, cv::COLOR_GRAY2BGR);
    }
    else
    {
      return;
    }
    if (bgr_image.empty())
    {
      return;
    }

    Snapshot snapshot;
    const bool have_snapshot = CollectSnapshot(image_frame.timestamp_us, snapshot);
    preview_.Submit(
        bgr_image,
        [snapshot, have_snapshot, q_gimbal_to_world, this](cv::Mat& canvas)
        {
          if (have_snapshot)
          {
            DrawPreview(canvas, snapshot, q_gimbal_to_world);
          }
        });
  }

  /**
   * @brief 停止 worker 线程。
   */
  void Stop()
  {
    const bool was_running = running_.exchange(false, std::memory_order_acq_rel);
    if (was_running && worker_thread_.joinable() &&
        worker_thread_.get_id() != std::this_thread::get_id())
    {
      worker_thread_.join();
    }
  }

 private:
  AimerPreviewConfig cfg_{};
  typename FrameSync::Subscriber subscriber_;
  VisionPreview preview_{};
  std::atomic<bool> running_{false};
  std::thread worker_thread_{};
  LibXR::Mutex snapshot_lock_{};
  bool have_target_{false};
  bool have_host_target_{false};
  bool have_host_fire_{false};
  ArmorTrackerTarget target_snapshot_{};
  AimerHostGimbalTarget host_target_snapshot_{};
  AimerHostFireNotify host_fire_snapshot_{};
  std::deque<Snapshot> snapshot_history_{};
  static constexpr std::size_t kMaxSnapshotHistory = 256;
};
