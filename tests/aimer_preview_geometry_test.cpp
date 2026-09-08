#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "AimerPreviewGeometry.hpp"

namespace
{
constexpr double tolerance = 1e-8;
constexpr CameraTypes::FrameLayout kLayout{720, 540, 2160, CameraTypes::Encoding::BGR8};

void Expect(bool condition, const char* label)
{
  if (!condition)
  {
    std::cerr << label << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void ExpectNear(double actual, double expected, const std::string& label)
{
  if (std::abs(actual - expected) > tolerance)
  {
    std::cerr << label << ": actual=" << actual << " expected=" << expected << '\n';
    std::exit(EXIT_FAILURE);
  }
}

CameraTypes::CameraCalibration MakeCalibration(CameraTypes::DistortionModel model)
{
  CameraTypes::CameraCalibration calibration{};
  calibration.native_width = 1440;
  calibration.native_height = 1080;
  calibration.camera_matrix = {800.0, 0.0, 720.0, 0.0, 820.0, 540.0, 0.0, 0.0, 1.0};
  calibration.distortion_model = model;
  calibration.distortion_coefficients = {-0.32,  0.11,  0.0012, -0.0009,
                                         -0.025, 0.012, -0.006, 0.002};
  return calibration;
}

cv::Mat MakeCameraMatrix(const CameraTypes::CameraCalibration& calibration)
{
  cv::Mat camera_matrix(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      camera_matrix.at<double>(row, col) =
          calibration.camera_matrix[static_cast<std::size_t>(row * 3 + col)];
    }
  }
  return camera_matrix;
}

cv::Mat MakeReferenceDistortion(const CameraTypes::CameraCalibration& calibration)
{
  int size = 0;
  switch (calibration.distortion_model)
  {
    case CameraTypes::DistortionModel::NONE:
      return {};
    case CameraTypes::DistortionModel::PLUMB_BOB:
      size = 5;
      break;
    case CameraTypes::DistortionModel::RATIONAL_POLYNOMIAL:
      size = 8;
      break;
    default:
      return {};
  }

  cv::Mat distortion(1, size, CV_64F);
  for (int index = 0; index < size; ++index)
  {
    distortion.at<double>(0, index) =
        calibration.distortion_coefficients[static_cast<std::size_t>(index)];
  }
  return distortion;
}

std::array<double, 2> ReferenceProjection(
    const std::array<double, 3>& point, const CameraTypes::CameraCalibration& calibration,
    const CameraTypes::FrameGeometry& geometry)
{
  std::vector<cv::Point2d> native_points;
  cv::projectPoints(std::vector<cv::Point3d>{{point[0], point[1], point[2]}},
                    cv::Vec3d::all(0.0), cv::Vec3d::all(0.0),
                    MakeCameraMatrix(calibration), MakeReferenceDistortion(calibration),
                    native_points);
  return CameraTypes::NativeToFrame(geometry, native_points[0].x, native_points[0].y);
}

void TestMatchesOpenCvAcrossModelsAndFrameGeometry()
{
  const std::array<CameraTypes::DistortionModel, 3> models{
      CameraTypes::DistortionModel::NONE, CameraTypes::DistortionModel::PLUMB_BOB,
      CameraTypes::DistortionModel::RATIONAL_POLYNOMIAL};
  const std::array<const char*, 3> model_names{"none", "plumb_bob", "rational"};
  const std::array<CameraTypes::FrameGeometry, 3> geometries{
      CameraTypes::FrameGeometry{720, 540, 2160, 0, 0, 2, 2,
                                 CameraTypes::FRAME_GEOMETRY_NONE, 0, 0.0F, 0.0F},
      CameraTypes::FrameGeometry{720, 540, 2160, 360, 270, 1, 1,
                                 CameraTypes::FRAME_GEOMETRY_NONE, 0, 0.0F, 0.0F},
      CameraTypes::FrameGeometry{
          720, 540, 2160, 0, 0, 2, 2,
          CameraTypes::FRAME_GEOMETRY_REVERSE_X | CameraTypes::FRAME_GEOMETRY_REVERSE_Y,
          0, 0.0F, 0.0F}};
  const std::array<const char*, 3> geometry_names{"wide", "narrow", "reverse"};
  const std::array<double, 3> optical_point{0.75, -0.42, 2.1};

  for (std::size_t model_index = 0; model_index < models.size(); ++model_index)
  {
    const auto calibration = MakeCalibration(models[model_index]);
    for (std::size_t geometry_index = 0; geometry_index < geometries.size();
         ++geometry_index)
    {
      std::array<double, 2> actual{};
      Expect(AimerDetail::ProjectOpticalToFrame(optical_point[0], optical_point[1],
                                                optical_point[2], calibration, kLayout,
                                                geometries[geometry_index], actual),
             "supported projection must succeed");
      const auto expected =
          ReferenceProjection(optical_point, calibration, geometries[geometry_index]);
      const std::string prefix =
          std::string(model_names[model_index]) + "/" + geometry_names[geometry_index];
      ExpectNear(actual[0], expected[0], prefix + " x");
      ExpectNear(actual[1], expected[1], prefix + " y");
    }
  }

  const auto none = MakeCalibration(CameraTypes::DistortionModel::NONE);
  const auto plumb_bob = MakeCalibration(CameraTypes::DistortionModel::PLUMB_BOB);
  const auto none_point = ReferenceProjection(optical_point, none, geometries[0]);
  const auto distorted_point =
      ReferenceProjection(optical_point, plumb_bob, geometries[0]);
  Expect(std::hypot(none_point[0] - distorted_point[0],
                    none_point[1] - distorted_point[1]) > 1.0,
         "non-zero distortion fixture must detect a zero-D regression");
}

void TestFailClosedInputs()
{
  const CameraTypes::FrameGeometry geometry{
      720, 540, 2160, 0, 0, 2, 2, CameraTypes::FRAME_GEOMETRY_NONE, 0, 0.0F, 0.0F};
  std::array<double, 2> point{};
  const auto calibration = MakeCalibration(CameraTypes::DistortionModel::PLUMB_BOB);
  Expect(!AimerDetail::ProjectOpticalToFrame(0.0, 0.0, 0.0, calibration, kLayout,
                                             geometry, point),
         "zero depth must fail");
  Expect(
      !AimerDetail::ProjectOpticalToFrame(std::numeric_limits<double>::quiet_NaN(), 0.0,
                                          1.0, calibration, kLayout, geometry, point),
      "non-finite point must fail");

  auto invalid_k = calibration;
  invalid_k.camera_matrix[0] = std::numeric_limits<double>::infinity();
  Expect(!AimerDetail::ProjectOpticalToFrame(0.1, 0.1, 1.0, invalid_k, kLayout, geometry,
                                             point),
         "non-finite K must fail");

  auto invalid_d = calibration;
  invalid_d.distortion_coefficients[2] = std::numeric_limits<double>::quiet_NaN();
  Expect(!AimerDetail::ProjectOpticalToFrame(0.1, 0.1, 1.0, invalid_d, kLayout, geometry,
                                             point),
         "non-finite active D must fail");

  auto invalid_d_tail = calibration;
  invalid_d_tail.distortion_coefficients[13] = std::numeric_limits<double>::quiet_NaN();
  Expect(!AimerDetail::ProjectOpticalToFrame(0.1, 0.1, 1.0, invalid_d_tail, kLayout,
                                             geometry, point),
         "non-finite unused D tail must fail");

  auto nonstandard_k = calibration;
  nonstandard_k.camera_matrix[1] = 1.0;
  Expect(!AimerDetail::ProjectOpticalToFrame(0.1, 0.1, 1.0, nonstandard_k, kLayout,
                                             geometry, point),
         "nonstandard pinhole K must fail closed");

  auto unsupported = calibration;
  unsupported.distortion_model = CameraTypes::DistortionModel::EQUIDISTANT;
  Expect(!AimerDetail::ProjectOpticalToFrame(0.1, 0.1, 1.0, unsupported, kLayout,
                                             geometry, point),
         "unsupported distortion model must fail closed");

  auto invalid_geometry = geometry;
  invalid_geometry.decimation_x = 0;
  Expect(!AimerDetail::ProjectOpticalToFrame(0.1, 0.1, 1.0, calibration, kLayout,
                                             invalid_geometry, point),
         "invalid frame geometry must fail");

  invalid_geometry = geometry;
  invalid_geometry.flags = 0x8000U;
  Expect(!AimerDetail::ProjectOpticalToFrame(0.1, 0.1, 1.0, calibration, kLayout,
                                             invalid_geometry, point),
         "unknown frame geometry flags must fail");

  invalid_geometry = geometry;
  invalid_geometry.reserved = 1;
  Expect(!AimerDetail::ProjectOpticalToFrame(0.1, 0.1, 1.0, calibration, kLayout,
                                             invalid_geometry, point),
         "non-zero frame geometry reserved field must fail");

  invalid_geometry = geometry;
  invalid_geometry.roi_offset_x_native = 2;
  Expect(!AimerDetail::ProjectOpticalToFrame(0.1, 0.1, 1.0, calibration, kLayout,
                                             invalid_geometry, point),
         "frame geometry outside native calibration must fail");
}
}  // namespace

int main()
{
  TestMatchesOpenCvAcrossModelsAndFrameGeometry();
  TestFailClosedInputs();
  std::cout << "Aimer distortion projection tests passed\n";
  return EXIT_SUCCESS;
}
