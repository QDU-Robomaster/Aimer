#include <cmath>
#include <cstdlib>
#include <iostream>

#include "AimerHeatFire.hpp"

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
  using namespace AimerDetail;

  // Heat: +10 per counted shot, at most one shot per minimum interval, cooling per second.
  HeatFireState state;
  CoolHeat(state, 1'000'000U, 40.0);
  Expect(state.valid && Near(state.heat, 0.0), "first call initialises the estimate");
  Expect(CountShot(state, 1'000'000U, 10.0, 0.05), "first shot is counted");
  Expect(!CountShot(state, 1'020'000U, 10.0, 0.05), "a request within 50 ms is not counted");
  Expect(CountShot(state, 1'050'000U, 10.0, 0.05), "a request after 50 ms is counted");
  Expect(Near(state.heat, 20.0), "two counted shots add 20 heat");
  CoolHeat(state, 1'300'000U, 40.0);
  Expect(Near(state.heat, 8.0), "0.3 s of cooling at 40/s removes 12 heat");
  CoolHeat(state, 2'000'000U, 40.0);
  Expect(Near(state.heat, 0.0), "heat does not go below zero");
  Expect(state.relax_ref_us == 1'050'000U, "the relaxation timer follows the last counted shot");
  StartEngagement(state, 3'000'000U);
  Expect(state.relax_ref_us == 3'000'000U && state.last_shot_us == 1'050'000U,
         "a new target restarts the relaxation timer only");

  // Recent shots: counted shots after a given time; at most the last HEAT_RECENT_SHOTS.
  Expect(CountedShotsSince(state, 999'999U) == 2, "both counted shots are recent");
  Expect(CountedShotsSince(state, 1'000'000U) == 1, "the start time is exclusive");
  Expect(CountedShotsSince(state, 1'050'000U) == 0, "no shot after the last one");
  HeatFireState many;
  CoolHeat(many, 0U, 0.0);
  for (uint64_t i = 1; i <= HEAT_RECENT_SHOTS + 4; ++i)
  {
    CountShot(many, i * 100'000U, 10.0, 0.05);
  }
  Expect(CountedShotsSince(many, 0U) == HEAT_RECENT_SHOTS, "only the last shots are kept");
  Expect(CountedShotsSince(many, (HEAT_RECENT_SHOTS + 2) * 100'000U) == 2,
         "the ring buffer keeps the newest shots");

  // Measured heat fusion: the measurement bounds the estimate from below, and from above
  // with the heat of the shots it may not include yet.
  HeatFireState fused;
  CoolHeat(fused, 0U, 40.0);
  fused.heat = 50.0;
  FuseMeasuredHeat(fused, 45.0, 10.0);
  Expect(Near(fused.heat, 50.0), "an estimate inside [m, m + unseen] is kept");
  FuseMeasuredHeat(fused, 30.0, 10.0);
  Expect(Near(fused.heat, 40.0), "an estimate above m + unseen falls to the upper bound");
  FuseMeasuredHeat(fused, 30.0, 0.0);
  Expect(Near(fused.heat, 30.0), "without unseen shots the estimate equals the measurement");
  FuseMeasuredHeat(fused, 80.0, 20.0);
  Expect(Near(fused.heat, 80.0), "an estimate below the measurement rises to it");
  FuseMeasuredHeat(fused, std::nan(""), 0.0);
  Expect(Near(fused.heat, 80.0), "a non-finite measurement is ignored");
  FuseMeasuredHeat(fused, -5.0, 0.0);
  Expect(Near(fused.heat, 0.0), "a negative measurement counts as zero");

  // Drift scenario: the local estimate counts requests that the launcher rejected; the
  // measurement removes the drift once those requests leave the window, while shots inside
  // the window stay counted.
  HeatFireState drift;
  CoolHeat(drift, 0U, 0.0);
  for (uint64_t t = 100'000U; t <= 1'000'000U; t += 100'000U)
  {
    CountShot(drift, t, 10.0, 0.05);
  }
  Expect(Near(drift.heat, 100.0), "ten local shots");
  const uint64_t now_us = 1'000'000U;
  const uint64_t window_us = 150'000U;
  const double unseen = 10.0 * static_cast<double>(CountedShotsSince(drift, now_us - window_us));
  Expect(Near(unseen, 20.0), "two shots inside the 150 ms window");
  FuseMeasuredHeat(drift, 60.0, unseen);
  Expect(Near(drift.heat, 80.0), "drift removed, unseen shots kept");

  // Hit probability: wider plates and smaller bias hit more often.
  const double centred = HitProbability(0.05, 0.0, 0.03);
  const double biased = HitProbability(0.05, 0.03, 0.03);
  const double narrow = HitProbability(0.03, 0.0, 0.03);
  Expect(centred > biased && centred > narrow, "bias and narrow plates lower P(hit)");
  Expect(Near(HitProbability(0.0, 0.0, 0.03), 0.0), "no hittable width gives zero");
  Expect(Near(HitProbability(0.05, 0.0, 0.03), 2.0 * 0.5 * std::erfc(-0.05 / 0.03 / std::sqrt(2.0)) - 1.0,
              1e-12),
         "centred probability is 2 Phi(w / sigma) - 1");

  // Exit-time offset and spread: the gimbal error is scaled, the command rate adds an
  // offset and a spread; gain 1 and zero rate terms give the request-time model.
  Expect(Near(ExitLateralBias(3.0, 0.01, 1.0, 0.5, 0.0), 0.03), "gain 1, no rate term: d |e|");
  Expect(Near(ExitLateralBias(3.0, 0.01, 0.65, 0.0, 0.01), 0.0195), "gain scales the error");
  Expect(Near(ExitLateralBias(3.0, -0.01, 0.5, 0.5, 0.01), 0.0), "rate offset can cancel the error");
  Expect(Near(ExitLateralBias(3.0, 0.0, 0.65, -0.5, 0.01), 0.015), "rate offset alone");
  Expect(Near(ExitLateralSigma(0.01, 0.0, 3.0, 0.0, 0.01), 0.01), "no rate: base sigma");
  Expect(Near(ExitLateralSigma(0.03, 0.04, 3.0, 0.0, 0.01), 0.05), "base and phase add in quadrature");
  Expect(Near(ExitLateralSigma(0.0, 0.0, 3.0, 1.0, 0.01), 0.03), "rate spread d w t");
  Expect(HitProbability(0.05, ExitLateralBias(3.0, 0.0, 0.65, 1.0, 0.01),
                        ExitLateralSigma(0.01, 0.02, 3.0, 1.0, 0.01)) <
             HitProbability(0.05, ExitLateralBias(3.0, 0.0, 0.65, 0.2, 0.01),
                            ExitLateralSigma(0.01, 0.02, 3.0, 0.2, 0.01)),
         "a faster command lowers P(hit)");

  // Threshold: rises with heat; relaxes towards the floor only at low heat after a pause.
  Expect(Near(HeatFireThreshold(0.55, 0.85, 0.3, 0.5, 0.0, 0.0), 0.55), "zero heat gives p_low");
  Expect(Near(HeatFireThreshold(0.55, 0.85, 0.3, 0.5, 1.0, 0.0), 0.85), "full heat gives p_high");
  Expect(Near(HeatFireThreshold(0.55, 0.85, 0.3, 0.5, 0.2, 0.4), 0.61), "no relax before relax_s");
  Expect(Near(HeatFireThreshold(0.55, 0.85, 0.3, 0.5, 0.2, 0.75), 0.455), "half way to the floor");
  Expect(Near(HeatFireThreshold(0.55, 0.85, 0.3, 0.5, 0.2, 5.0), 0.3), "floor after 2 relax_s");
  Expect(Near(HeatFireThreshold(0.55, 0.85, 0.3, 0.5, 0.8, 5.0), 0.79), "no relax at high heat");
  return EXIT_SUCCESS;
}
