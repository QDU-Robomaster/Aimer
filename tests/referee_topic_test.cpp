#include <cstdlib>
#include <iostream>
#include <string>
#include <type_traits>

#include "Aimer.hpp"

static_assert(std::is_same_v<AimerRefereeSummary, RefereeTypes::RobotGameRefereePack>);
static_assert(AimerConfig{}.referee_topic == "robot_game_ref");

struct LogContext
{
  std::string expected;
  unsigned count{};
};

void ObserveLog(bool, LogContext* context, LibXR::ConstRawData data)
{
  const auto* log = static_cast<const LibXR::LogData*>(data.addr_);
  if (log != nullptr &&
      std::string_view(log->message).find(context->expected) != std::string_view::npos)
  {
    ++context->count;
  }
}

void Expect(bool ok, const char* message)
{
  if (!ok)
  {
    std::cerr << message << std::endl;
    std::_Exit(EXIT_FAILURE);
  }
}

int main(int argc, char** argv)
{
  Expect(argc == 2, "topic argument required");
  LibXR::PlatformInit();
  LibXR::HardwareContainer hw;
  LibXR::ApplicationManager app;
  LibXR::Topic::Domain host("host");
  std::string configured_name = argv[1];
  const std::string selected_name = configured_name;
  const char* other_name =
      selected_name == "sentry_ref" ? "robot_game_ref" : "sentry_ref";
  auto selected = LibXR::Topic::CreateTopic<RefereeTypes::RobotGameRefereePack>(
      selected_name.c_str(), &host, true);
  auto other = LibXR::Topic::CreateTopic<RefereeTypes::RobotGameRefereePack>(other_name,
                                                                             &host, true);
  AimerConfig config;
  config.referee_topic = configured_name;
  config.enable_runtime_log = true;
  AimerCore aimer(hw, app, config);
  configured_name.assign(128, 'x');

  LogContext context{"source=" + selected_name};
  LibXR::Topic log_topic(LibXR::Topic::Find("/xr/log"));
  auto callback = LibXR::Topic::Callback::Create(ObserveLog, &context);
  log_topic.RegisterCallback(callback);
  RefereeTypes::RobotGameRefereePack packet{};
  packet.robot_status.shooter_cooling_value = 17;
  packet.robot_status.shooter_heat_limit = 250;
  other.Publish(packet, LibXR::MicrosecondTimestamp(100));
  Expect(context.count == 0, "Aimer must not receive the other referee topic");
  selected.Publish(packet, LibXR::MicrosecondTimestamp(200));
  Expect(context.count > 0, "Aimer must process its configured producer-typed input");
  std::cout << "direct referee subscription passed: " << selected_name << std::endl;
  // Topic callbacks and platform STDIO threads have process lifetime.
  std::_Exit(EXIT_SUCCESS);
}
