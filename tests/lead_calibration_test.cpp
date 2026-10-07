#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "AimerLeadCalibration.hpp"

namespace
{
void Expect(bool ok, const char* message)
{
  if (!ok)
  {
    std::cerr << message << std::endl;
    std::_Exit(EXIT_FAILURE);
  }
}

/// Deterministic uniform numbers in [-1, 1).
class Lcg
{
 public:
  double Next()
  {
    state_ = state_ * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<double>(state_ >> 11) / static_cast<double>(1ULL << 52) - 1.0;
  }

 private:
  uint64_t state_{12345};
};

using AimerDetail::LeadCalibrator;
using Event = LeadCalibrator::Event;

/**
 * Feeds one batch of samples from a plant whose measured lead is gain * (offset + adjust),
 * as in the Webots runs where the regression sees about 0.3 of a delay change.
 */
LeadCalibrator::Result FeedBatch(LeadCalibrator& calibrator, Lcg& rng, double offset_s,
                                 double rate_amplitude = 0.8, double plant_gain = 0.3,
                                 double noise_rad = 0.002)
{
  for (int i = 0; i < 1000; ++i)
  {
    const double rate = rate_amplitude * rng.Next();
    const double lead = plant_gain * (offset_s + calibrator.Adjust());
    calibrator.AddSample(rate, lead * rate + 0.001 + noise_rad * rng.Next());
    if (calibrator.BatchReady())
    {
      return calibrator.CloseBatch();
    }
  }
  return {};
}
}  // namespace

int main()
{
  // Convergence: a 25 ms over-lead is removed within the correcting batches, then frozen.
  {
    LeadCalibrator calibrator({.calibration_batches = 5, .max_adjust_s = 0.05});
    Lcg rng;
    Event last = Event::NONE;
    int batches = 0;
    while (!calibrator.Frozen() && batches < 10)
    {
      const auto result = FeedBatch(calibrator, rng, 0.025);
      Expect(result.event == Event::CORRECTED || result.event == Event::FROZEN,
             "correcting batches report CORRECTED, the last one FROZEN");
      last = result.event;
      ++batches;
    }
    Expect(batches == 5 && last == Event::FROZEN, "frozen after the configured five batches");
    Expect(std::abs(0.025 + calibrator.Adjust()) < 0.006, "residual lead below 6 ms after freezing");

    // Once frozen, a small lead is only monitored and the correction stays.
    const double frozen = calibrator.Adjust();
    const auto monitored = FeedBatch(calibrator, rng, 0.025);
    Expect(monitored.event == Event::MONITORED && calibrator.Adjust() == frozen,
           "monitoring batches do not change the frozen correction");

    // A persistent new lead of 20 ms restarts the calibration after three batches.
    Event event = Event::NONE;
    int monitor_batches = 0;
    while (event != Event::RESTARTED && monitor_batches < 10)
    {
      event = FeedBatch(calibrator, rng, 0.045).event;
      ++monitor_batches;
    }
    Expect(event == Event::RESTARTED && monitor_batches == 3, "three drifting batches restart");
    Expect(!calibrator.Frozen(), "a restarted calibration corrects again");
    while (!calibrator.Frozen())
    {
      FeedBatch(calibrator, rng, 0.045);
    }
    Expect(std::abs(0.045 + calibrator.Adjust()) < 0.006, "recalibrated to the new lead");
  }

  // Batches without command-rate variance are skipped and change nothing.
  {
    LeadCalibrator calibrator;
    Lcg rng;
    const auto result = FeedBatch(calibrator, rng, 0.025, 0.05);
    Expect(result.event == Event::SKIPPED && calibrator.Adjust() == 0.0,
           "low command-rate variance skips the batch");
  }

  // The correction is limited to max_adjust_s.
  {
    LeadCalibrator calibrator({.calibration_batches = 5, .max_adjust_s = 0.02});
    Lcg rng;
    for (int i = 0; i < 5; ++i)
    {
      FeedBatch(calibrator, rng, 0.2);
    }
    Expect(calibrator.Frozen() && std::abs(calibrator.Adjust() + 0.02) < 1e-12,
           "correction clamped to -max_adjust_s");
  }
  return EXIT_SUCCESS;
}
