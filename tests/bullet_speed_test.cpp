#include <cmath>
#include <cstdlib>
#include <iostream>

#include "AimerBulletSpeed.hpp"

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

bool Near(double a, double b, double tol = 1e-9) { return std::abs(a - b) <= tol; }
}  // namespace

int main()
{
  using AimerDetail::BulletSpeedFilter;

  // No estimate before min_samples valid readings; invalid readings are not counted.
  BulletSpeedFilter filter({.window = 5, .min_samples = 3, .min_speed = 14.0,
                            .max_speed = 35.0, .outlier_floor = 0.5});
  double estimate = 0.0;
  Expect(!filter.Estimate(estimate), "no estimate without readings");
  Expect(!filter.AddSample(std::nan("")), "NaN is rejected");
  Expect(!filter.AddSample(0.0), "zero is rejected");
  Expect(!filter.AddSample(80.0), "an implausible speed is rejected");
  Expect(filter.AddSample(23.0) && filter.AddSample(23.2), "valid readings are accepted");
  Expect(!filter.Estimate(estimate), "two readings are not enough");
  Expect(filter.AddSample(22.8), "third reading");
  Expect(filter.Estimate(estimate) && Near(estimate, 23.0), "mean of three close readings");

  // A single outlier inside the valid range does not move the estimate.
  Expect(filter.AddSample(18.0), "an outlier inside the valid range enters the window");
  Expect(filter.Estimate(estimate) && Near(estimate, 23.0), "the outlier is gated out");
  Expect(filter.AddSample(23.1), "fifth reading");
  Expect(filter.Estimate(estimate) && Near(estimate, (23.0 + 23.2 + 22.8 + 23.1) / 4.0),
         "the outlier stays excluded with a full window");

  // Noise is averaged: the estimate is closer to the true speed than a raw reading.
  BulletSpeedFilter noisy({.window = 9, .min_samples = 3, .min_speed = 14.0,
                           .max_speed = 35.0, .outlier_floor = 0.5});
  const double readings[] = {24.3, 23.6, 24.1, 23.8, 24.4, 23.7, 24.0, 24.2, 23.9};
  for (const double v : readings)
  {
    noisy.AddSample(v);
  }
  Expect(noisy.Estimate(estimate) && std::abs(estimate - 24.0) < 0.05,
         "nine noisy readings average to the true speed");

  // A step change is followed once the new speed is the majority of the window, and the
  // estimate never jumps to a single new reading.
  BulletSpeedFilter step({.window = 5, .min_samples = 3, .min_speed = 14.0,
                          .max_speed = 35.0, .outlier_floor = 0.5});
  for (int i = 0; i < 5; ++i)
  {
    step.AddSample(23.0);
  }
  step.AddSample(25.0);
  Expect(step.Estimate(estimate) && Near(estimate, 23.0), "one new reading is gated out");
  step.AddSample(25.0);
  Expect(step.Estimate(estimate) && Near(estimate, 23.0), "two of five still gated out");
  step.AddSample(25.0);
  Expect(step.Estimate(estimate) && Near(estimate, 25.0), "majority switches the estimate");

  // Window and min_samples are clamped to usable values.
  BulletSpeedFilter tiny({.window = 0, .min_samples = 0, .min_speed = 14.0,
                          .max_speed = 35.0, .outlier_floor = 0.5});
  tiny.AddSample(20.0);
  Expect(tiny.Estimate(estimate) && Near(estimate, 20.0) && tiny.Count() == 1,
         "window 0 behaves as window 1");
  tiny.AddSample(21.0);
  Expect(tiny.Estimate(estimate) && Near(estimate, 21.0), "window 1 follows the last reading");
  return EXIT_SUCCESS;
}
