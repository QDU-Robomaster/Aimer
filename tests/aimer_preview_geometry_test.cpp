#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "AimerPreviewGeometry.hpp"

namespace
{
constexpr double tolerance = 1e-9;

void ExpectNear(double actual, double expected, const char* label)
{
  if (std::abs(actual - expected) > tolerance)
  {
    std::cerr << label << ": actual=" << actual << " expected=" << expected << '\n';
    std::exit(EXIT_FAILURE);
  }
}

CameraTypes::CameraCalibration MakeCalibration()
{
  CameraTypes::CameraCalibration calibration{};
  calibration.native_width = 1440;
  calibration.native_height = 1080;
  calibration.camera_matrix = {800.0, 0.0, 720.0, 0.0, 820.0, 540.0, 0.0, 0.0, 1.0};
  return calibration;
}

void TestWideAndCenteredRoi()
{
  const CameraTypes::CameraCalibration calibration = MakeCalibration();
  const CameraTypes::FrameGeometry wide{
      720, 540, 2160, 0, 0, 2, 2, CameraTypes::FRAME_GEOMETRY_NONE, 0, 0.0F, 0.0F};
  const CameraTypes::FrameGeometry roi{
      720, 540, 2160, 360, 270, 1, 1, CameraTypes::FRAME_GEOMETRY_NONE, 0, 0.0F, 0.0F};

  std::array<double, 2> wide_point{};
  std::array<double, 2> roi_point{};
  if (!AimerDetail::ProjectOpticalToFrame(0.1, -0.05, 2.0, calibration, wide,
                                          wide_point) ||
      !AimerDetail::ProjectOpticalToFrame(0.1, -0.05, 2.0, calibration, roi, roi_point))
  {
    std::exit(EXIT_FAILURE);
  }

  ExpectNear(wide_point[0], 380.0, "wide x");
  ExpectNear(wide_point[1], 259.75, "wide y");
  ExpectNear(roi_point[0], 400.0, "roi x");
  ExpectNear(roi_point[1], 249.5, "roi y");
}

void TestReverseFlags()
{
  const CameraTypes::CameraCalibration calibration = MakeCalibration();
  const CameraTypes::FrameGeometry geometry{
      720,
      540,
      2160,
      0,
      0,
      2,
      2,
      CameraTypes::FRAME_GEOMETRY_REVERSE_X | CameraTypes::FRAME_GEOMETRY_REVERSE_Y,
      0,
      0.0F,
      0.0F};

  std::array<double, 2> point{};
  if (!AimerDetail::ProjectOpticalToFrame(0.0, 0.0, 2.0, calibration, geometry, point))
  {
    std::exit(EXIT_FAILURE);
  }
  ExpectNear(point[0], 359.0, "reverse x");
  ExpectNear(point[1], 269.0, "reverse y");
}

void TestInvalidDepth()
{
  const CameraTypes::CameraCalibration calibration = MakeCalibration();
  const CameraTypes::FrameGeometry geometry{
      720, 540, 2160, 0, 0, 2, 2, CameraTypes::FRAME_GEOMETRY_NONE, 0, 0.0F, 0.0F};
  std::array<double, 2> point{};
  if (AimerDetail::ProjectOpticalToFrame(0.0, 0.0, 0.0, calibration, geometry, point) ||
      AimerDetail::ProjectOpticalToFrame(std::numeric_limits<double>::quiet_NaN(), 0.0,
                                         1.0, calibration, geometry, point))
  {
    std::exit(EXIT_FAILURE);
  }
}
}  // namespace

int main()
{
  TestWideAndCenteredRoi();
  TestReverseFlags();
  TestInvalidDepth();
  return EXIT_SUCCESS;
}
