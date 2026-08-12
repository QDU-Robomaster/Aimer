#pragma once

#include <array>
#include <cmath>

#include "CameraBase.hpp"

namespace AimerDetail
{
/**
 * @brief Project one optical-frame point through native K into the current frame.
 */
inline bool ProjectOpticalToFrame(double x, double y, double depth,
                                  const CameraTypes::CameraCalibration& calibration,
                                  const CameraTypes::FrameGeometry& geometry,
                                  std::array<double, 2>& frame_point)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(depth) || depth <= 1e-6)
  {
    return false;
  }

  const double native_u =
      calibration.camera_matrix[0] * x / depth + calibration.camera_matrix[2];
  const double native_v =
      calibration.camera_matrix[4] * y / depth + calibration.camera_matrix[5];
  const double frame_u = CameraTypes::NativeToFrameX(geometry, native_u);
  const double frame_v = CameraTypes::NativeToFrameY(geometry, native_v);
  if (!std::isfinite(frame_u) || !std::isfinite(frame_v))
  {
    return false;
  }

  frame_point = {frame_u, frame_v};
  return true;
}
}  // namespace AimerDetail
