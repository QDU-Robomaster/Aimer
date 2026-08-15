#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <vector>

#include "CameraBase.hpp"

namespace AimerDetail
{
/**
 * @brief Project one optical-frame point through native K/D into the current
 * frame.
 */
inline bool ProjectOpticalToFrame(double x, double y, double depth,
                                  const CameraTypes::CameraCalibration& calibration,
                                  const CameraTypes::FrameLayout& layout,
                                  const CameraTypes::FrameGeometry& geometry,
                                  std::array<double, 2>& frame_point)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(depth) || depth <= 1e-6)
  {
    return false;
  }

  if (!CameraBaseIntrinsicSanity::CameraCalibrationReasonable(calibration))
  {
    return false;
  }
  if (!CameraTypes::ValidateFrameGeometry(layout, calibration, geometry))
  {
    return false;
  }

  const CameraTypes::PnPDistCoeffs pnp_distortion =
      CameraTypes::BuildPnPDistCoeffs(calibration);
  if (pnp_distortion.requires_undistort_first)
  {
    return false;
  }
  cv::Mat camera_matrix(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      camera_matrix.at<double>(row, col) =
          calibration.camera_matrix[static_cast<std::size_t>(row * 3 + col)];
    }
  }

  cv::Mat distortion_coefficients;
  if (pnp_distortion.size > 0)
  {
    distortion_coefficients = cv::Mat(1, pnp_distortion.size, CV_64F);
    for (uint8_t index = 0; index < pnp_distortion.size; ++index)
    {
      distortion_coefficients.at<double>(0, index) = pnp_distortion.values[index];
    }
  }

  std::vector<cv::Point2d> native_points;
  cv::projectPoints(std::vector<cv::Point3d>{{x, y, depth}}, cv::Vec3d::all(0.0),
                    cv::Vec3d::all(0.0), camera_matrix, distortion_coefficients,
                    native_points);
  if (native_points.size() != 1 || !std::isfinite(native_points[0].x) ||
      !std::isfinite(native_points[0].y))
  {
    return false;
  }

  const double frame_u = CameraTypes::NativeToFrameX(geometry, native_points[0].x);
  const double frame_v = CameraTypes::NativeToFrameY(geometry, native_points[0].y);
  if (!std::isfinite(frame_u) || !std::isfinite(frame_v))
  {
    return false;
  }

  frame_point = {frame_u, frame_v};
  return true;
}
}  // namespace AimerDetail
