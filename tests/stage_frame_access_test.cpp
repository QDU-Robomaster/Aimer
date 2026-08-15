#include <cstdlib>
#include <type_traits>

#include "ArmorTrackerTarget.hpp"

namespace
{
constexpr CameraTypes::FrameLayout kLayout{720, 540, 2160, CameraTypes::Encoding::BGR8};
using Frame = TrackedFrame<kLayout>;

static_assert(std::is_same_v<TrackedFrameMessage<kLayout>, const Frame*>);

// Compile the exact synchronous callback access surface used by Aimer.
void ReadStageFrame(const Frame& frame)
{
  const Frame::ImageFrame* const image = frame.GetImageFrame();
  if (image == nullptr)
  {
    return;
  }

  const auto& geometry = image->geometry;
  const auto& rotation = frame.imu.rotation_wxyz;
  const auto& target = frame.target;
  (void)geometry;
  (void)rotation;
  (void)target;
}

static_assert(std::is_invocable_v<decltype(ReadStageFrame), const Frame&>);
}  // namespace

int main() { return EXIT_SUCCESS; }
