// 模块接线：每收一个跟踪帧发一个瞄准帧；未跟踪时不控制，跟踪静止目标时瞄准正对的板。
//
// Module wiring: one aimed frame per tracked frame; no control without a target, and a
// static target is aimed at its facing plate.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "Aimer.hpp"
#include "libxr.hpp"

namespace
{
void Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}
}  // namespace

int main()
{
  LibXR::PlatformInit();
  LibXR::Topic tracked =
      LibXR::Topic::CreateTopic<const AutoAim::TrackedFrame*>("mod_tracked");
  Aimer aimer("mod");
  std::vector<AutoAim::AimedFrame> received;
  auto callback = LibXR::Topic::Callback::Create(
      [](bool, std::vector<AutoAim::AimedFrame>* out, const AutoAim::AimedFrame* frame)
      { out->push_back(*frame); }, &received);
  AutoAim::RequireTopic<const AutoAim::AimedFrame*>("mod_aimed")
      .RegisterCallback(callback);

  // 3 m 外静止的四板车，0 号板正对射手 / A static four-plate vehicle 3 m ahead.
  for (int i = 0; i < 60; ++i)
  {
    AutoAim::TrackedFrame frame{};
    frame.detected.synced.sequence = static_cast<uint64_t>(i + 1);
    frame.detected.synced.imu.rotation_wxyz = {1, 0, 0, 0};
    frame.target.image_timestamp_us = 10000ULL * (i + 1);
    frame.target.tracking = i >= 10;
    frame.target.id = ArmorNumber::TWO;
    frame.target.armors_num = 4;
    frame.target.position = Eigen::Vector3d(0.0, 3.0, 0.0);
    frame.target.radius_1 = frame.target.radius_2 = 0.25;
    const AutoAim::TrackedFrame* payload = &frame;
    tracked.Publish(payload);
  }

  Expect(received.size() == 60, "one aimed frame per tracked frame");
  Expect(received[5].tracked.detected.synced.sequence == 6,
         "aimed frame carries its input");
  Expect(!received[5].aim.control && received[5].aim.plate == -1,
         "no control without a target");
  const AutoAim::AimResult& last = received.back().aim;
  std::printf("aim: control %d plate %d point (%.3f, %.3f, %.3f) yaw %.4f pitch %.4f\n",
              last.control, last.plate, last.aim_point.x(), last.aim_point.y(),
              last.aim_point.z(), last.yaw, last.pitch);
  Expect(last.control, "controls the gimbal on a tracked target");
  Expect(last.plate >= 0, "selects a plate");
  Expect((last.aim_point - Eigen::Vector3d(0.0, 2.75, 0.0)).norm() < 0.02,
         "aims at the facing plate");

  // 物理约定的前哨站：yaw 0 的板朝外、在中心靠近射手一侧 / Physical outpost: the plate
  // at yaw 0 faces outward on the shooter's side of the centre.
  for (int i = 0; i < 60; ++i)
  {
    AutoAim::TrackedFrame frame{};
    frame.detected.synced.sequence = static_cast<uint64_t>(100 + i);
    frame.detected.synced.imu.rotation_wxyz = {1, 0, 0, 0};
    frame.target.image_timestamp_us = 1000000ULL + 10000ULL * i;
    frame.target.tracking = true;
    frame.target.id = ArmorNumber::OUTPOST;
    frame.target.armors_num = 3;
    frame.target.position = Eigen::Vector3d(0.0, 5.0, 0.0);
    frame.target.radius_1 = frame.target.radius_2 = 0.2765;
    const AutoAim::TrackedFrame* payload = &frame;
    tracked.Publish(payload);
  }
  const AutoAim::AimResult& outpost = received.back().aim;
  std::printf("outpost aim: control %d plate %d point (%.3f, %.3f, %.3f)\n",
              outpost.control, outpost.plate, outpost.aim_point.x(),
              outpost.aim_point.y(), outpost.aim_point.z());
  Expect(outpost.control && std::abs(outpost.aim_point.x()) < 0.02 &&
             std::abs(outpost.aim_point.y() - (5.0 - 0.2765)) < 0.02,
         "outpost plate on the near side of the centre");
  std::puts("aimer_module_test passed");
  std::fflush(stdout);
  std::_Exit(EXIT_SUCCESS);
}
