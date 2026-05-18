#pragma once

/**
 * @file AimerPreview.hpp
 * @brief Aimer 内部实时预览实现。
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "AimerTargetModel.hpp"
#include "VisionPreview.hpp"

/**
 * @brief Aimer 实时预览模块配置。
 */
struct AimerPreviewConfig
{
  /// 预览子系统运行参数。
  VisionPreview::RuntimeParam preview{};
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
 * @brief Aimer 预览：在 tracker 同源图像上绘制 Aimer 本帧状态。
 *
 * tracker/target_frame 中的图像指针只在回调期间有效。本类只把原图
 * cv::Mat 视图传给 VisionPreview::Submit()；实际深拷贝由 VisionPreview
 * 在 Submit() 入口完成。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class AimerPreview : public LibXR::Application
{
 public:
  using SourceFrame = ArmorDetectionsSourceFrame<CameraInfoV>;
  using ImageFrame = typename SourceFrame::ImageFrame;
  using TargetFramePacket = ArmorTrackerTargetFramePacket<CameraInfoV>;
  struct ProjectionTransform
  {
    std::array<double, 9> rotation{};
    std::array<double, 3> translation{};
  };

  /**
   * @brief 构造 Aimer preview。
   */
  AimerPreview(LibXR::HardwareContainer&, LibXR::ApplicationManager& app,
               AimerPreviewConfig cfg)
      : cfg_(std::move(cfg)), preview_(cfg_.preview)
  {
    app.Register(*this);
  }

  /**
   * @brief Aimer preview 不需要周期监控。
   */
  void OnMonitor() override {}

  /**
   * @brief 处理 Aimer Core 同步提交的本帧状态。
   */
  void OnAimerFrame(const AimerPreviewFrame& frame,
                    const TargetFramePacket* target_frame)
  {
    SubmitPreview(frame, target_frame);
  }

 private:
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
   * @brief tracker 输出坐标系点转像素。
   */
  static bool ProjectOutputPoint(const Eigen::Vector3d& point,
                                 const ProjectionTransform& projection,
                                 cv::Point& uv)
  {
    Eigen::Matrix3d R_output_to_camera;
    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
      {
        R_output_to_camera(row, col) =
            projection.rotation[static_cast<std::size_t>(row * 3 + col)];
      }
    }
    const Eigen::Vector3d t_output_to_camera(
        projection.translation[0], projection.translation[1],
        projection.translation[2]);
    return ProjectOpticalPoint(R_output_to_camera * point + t_output_to_camera,
                               uv);
  }

  /**
   * @brief 根据 target 几何状态构造装甲板四角。
   */
  static std::array<Eigen::Vector3d, 4> BuildArmorQuad(
      const Eigen::Vector4d& xyza, const ArmorTrackerTarget& target)
  {
    const Eigen::Vector3d center = xyza.head<3>();
    const double yaw = xyza[3];
    const Eigen::Vector3d width_dir(std::cos(yaw), std::sin(yaw), 0.0);
    const Eigen::Vector3d face_normal(-std::sin(yaw), std::cos(yaw), 0.0);
    const double tilt = AimerDetail::ARMOR_TILT_DEG * AimerDetail::DEG2RAD;
    const double tilt_sign = 1.0;
    const Eigen::Vector3d height_dir =
        face_normal * (tilt_sign * std::sin(tilt)) +
        Eigen::Vector3d(0.0, 0.0, std::cos(tilt));
    const Eigen::Vector3d half_w =
        0.5 * AimerDetail::SMALL_ARMOR_WIDTH_M * width_dir;
    const Eigen::Vector3d half_h = 0.5 * AimerDetail::ARMOR_HEIGHT_M * height_dir;
    return {center - half_w + half_h, center + half_w + half_h,
            center + half_w - half_h, center - half_w - half_h};
  }

  /**
   * @brief 绘制单个装甲板四边形。
   */
  static bool DrawArmorQuad(cv::Mat& canvas, const Eigen::Vector4d& xyza,
                            const ArmorTrackerTarget& target,
                            const ProjectionTransform& projection,
                            const cv::Scalar& color, int thickness)
  {
    const auto quad = BuildArmorQuad(xyza, target);
    std::array<cv::Point, 4> corners{};
    for (std::size_t i = 0; i < quad.size(); ++i)
    {
      if (!ProjectOutputPoint(quad[i], projection, corners[i]))
      {
        return false;
      }
    }

    for (int edge = 0; edge < 4; ++edge)
    {
      cv::line(canvas, corners[static_cast<std::size_t>(edge)],
               corners[static_cast<std::size_t>((edge + 1) % 4)], color,
               thickness, cv::LINE_AA);
    }
    return true;
  }

  /**
   * @brief 绘制 tracker 当前四个装甲板和当前绑定面。
   */
  static void DrawTrackerArmors(cv::Mat& canvas, const AimerPreviewFrame& frame,
                                const ProjectionTransform& projection)
  {
    if (!frame.have_target || !frame.target.tracking)
    {
      return;
    }

    AimerDetail::PredictedTarget current{frame.target};
    const auto armor_xyza_list = current.GetArmorXYZAList();
    std::array<cv::Point, 4> centers{};
    std::array<bool, 4> center_valid{};
    const int count =
        std::min<int>(4, static_cast<int>(armor_xyza_list.size()));

    for (int i = 0; i < count; ++i)
    {
      const bool tracked = i == frame.target.tracked_face_index;
      const cv::Scalar color =
          tracked ? cv::Scalar(80, 255, 80) : cv::Scalar(255, 180, 40);
      DrawArmorQuad(canvas, armor_xyza_list[static_cast<std::size_t>(i)],
                    frame.target, projection, color, tracked ? 2 : 1);

      cv::Point center_uv;
      if (ProjectOutputPoint(
              armor_xyza_list[static_cast<std::size_t>(i)].head<3>(),
              projection, center_uv))
      {
        centers[static_cast<std::size_t>(i)] = center_uv;
        center_valid[static_cast<std::size_t>(i)] = true;
        cv::circle(canvas, center_uv, tracked ? 5 : 4, color, -1, cv::LINE_AA);
        cv::putText(canvas, "T" + std::to_string(i), center_uv + cv::Point(6, 14),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);
      }
    }

    for (int i = 0; i < count && count > 1; ++i)
    {
      const int next = (i + 1) % count;
      if (center_valid[static_cast<std::size_t>(i)] &&
          center_valid[static_cast<std::size_t>(next)])
      {
        cv::line(canvas, centers[static_cast<std::size_t>(i)],
                 centers[static_cast<std::size_t>(next)],
                 cv::Scalar(120, 220, 255), 1, cv::LINE_AA);
      }
    }
  }

  /**
   * @brief 绘制 Aimer 预测选板和最终瞄点。
   */
  static void DrawPrediction(cv::Mat& canvas, const AimerPreviewFrame& frame,
                             const ProjectionTransform& projection)
  {
    if (!frame.have_target || !frame.target.tracking || !frame.aim_point_valid)
    {
      return;
    }

    const bool fire = frame.have_host_fire && frame.host_fire.isfire;
    const cv::Scalar color = fire ? cv::Scalar(0, 0, 255)
                                  : cv::Scalar(0, 180, 255);
    DrawArmorQuad(canvas, frame.aim_xyza, frame.target, projection,
                  color, 2);

    cv::Point aim_uv;
    if (ProjectOutputPoint(frame.aim_point, projection, aim_uv))
    {
      cv::drawMarker(canvas, aim_uv, color, cv::MARKER_TILTED_CROSS,
                     fire ? 24 : 20, 2, cv::LINE_AA);
      if (fire)
      {
        cv::circle(canvas, aim_uv, 11, color, 2, cv::LINE_AA);
      }
      cv::putText(canvas, "A" + std::to_string(frame.aim_armor_index),
                  aim_uv + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  color, 1, cv::LINE_AA);
    }
  }

  /**
   * @brief 绘制简短状态行。
   */
  static void DrawStatus(cv::Mat& canvas, const AimerPreviewFrame& frame)
  {
    const bool tracking = frame.have_target && frame.target.tracking;
    const bool fire = frame.have_host_fire && frame.host_fire.isfire;
    const std::size_t id_index = static_cast<std::size_t>(frame.target.id);
    const std::string id_name =
        tracking && id_index < ARMOR_NUMBER_NAMES.size()
            ? std::string(ARMOR_NUMBER_NAMES[id_index])
            : std::string("invalid");
    const std::string header =
        std::string("aimer ") + (tracking ? "TRACK" : "NO_TARGET") +
        " id=" + id_name + " track_face=" +
        std::to_string(tracking ? frame.target.tracked_face_index : -1) +
        " aim_face=" + std::to_string(frame.aim_armor_index) +
        " fire=" + std::to_string(fire ? 1 : 0);
    cv::putText(canvas, header, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
                0.68, cv::Scalar(40, 240, 40), 2, cv::LINE_AA);
  }

  /**
   * @brief 绘制完整 Aimer 预览。
   */
  static void DrawPreview(cv::Mat& canvas, const AimerPreviewFrame& frame,
                          const ProjectionTransform& projection)
  {
    DrawTrackerArmors(canvas, frame, projection);
    DrawPrediction(canvas, frame, projection);
    DrawStatus(canvas, frame);
  }

  /**
   * @brief 将源图像帧转换为 BGR Mat 视图或临时转换图。
   */
  static bool MakeBgrImage(const ImageFrame& image_frame, cv::Mat& bgr_image)
  {
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
        return false;
    }

    cv::Mat image(static_cast<int>(CameraInfoV.height),
                  static_cast<int>(CameraInfoV.width), cv_type,
                  const_cast<uint8_t*>(image_frame.data.data()),
                  static_cast<size_t>(CameraInfoV.step));
    switch (CameraInfoV.encoding)
    {
      case CameraTypes::Encoding::BGR8:
        bgr_image = image;
        break;
      case CameraTypes::Encoding::RGB8:
        cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
        break;
      case CameraTypes::Encoding::RGBA8:
        cv::cvtColor(image, bgr_image, cv::COLOR_RGBA2BGR);
        break;
      case CameraTypes::Encoding::BGRA8:
        cv::cvtColor(image, bgr_image, cv::COLOR_BGRA2BGR);
        break;
      case CameraTypes::Encoding::MONO8:
        cv::cvtColor(image, bgr_image, cv::COLOR_GRAY2BGR);
        break;
      default:
        return false;
    }

    return !bgr_image.empty();
  }

  /**
   * @brief 提交预览帧。
   */
  void SubmitPreview(const AimerPreviewFrame& frame,
                     const TargetFramePacket* target_frame)
  {
    if (!preview_.Running() || target_frame == nullptr ||
        target_frame->source_frame.image_frame == nullptr)
    {
      return;
    }
    const SourceFrame& source_frame = target_frame->source_frame;
    if (source_frame.image_timestamp_us != frame.image_timestamp_us ||
        source_frame.image_frame->timestamp_us != frame.image_timestamp_us)
    {
      return;
    }
    const ProjectionTransform projection{
        target_frame->output_to_camera_rotation,
        target_frame->output_to_camera_translation};

    cv::Mat bgr_image;
    if (!MakeBgrImage(*source_frame.image_frame, bgr_image))
    {
      return;
    }

    preview_.Submit(bgr_image,
                    [frame, projection](cv::Mat& canvas)
                    {
                      DrawPreview(canvas, frame, projection);
                    });
  }

 private:
  AimerPreviewConfig cfg_{};
  VisionPreview preview_{};
};
