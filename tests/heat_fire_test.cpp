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

  // Hit probability: wider plates and smaller bias hit more often.
  const double centred = HitProbability(0.05, 0.0, 0.03);
  const double biased = HitProbability(0.05, 0.03, 0.03);
  const double narrow = HitProbability(0.03, 0.0, 0.03);
  Expect(centred > biased && centred > narrow, "bias and narrow plates lower P(hit)");
  Expect(Near(HitProbability(0.0, 0.0, 0.03), 0.0), "no hittable width gives zero");
  Expect(Near(HitProbability(0.05, 0.0, 0.03), 2.0 * 0.5 * std::erfc(-0.05 / 0.03 / std::sqrt(2.0)) - 1.0,
              1e-12),
         "centred probability is 2 Phi(w / sigma) - 1");

  // Threshold: rises with heat; relaxes towards the floor only at low heat after a pause.
  Expect(Near(HeatFireThreshold(0.55, 0.85, 0.3, 0.5, 0.0, 0.0), 0.55), "zero heat gives p_low");
  Expect(Near(HeatFireThreshold(0.55, 0.85, 0.3, 0.5, 1.0, 0.0), 0.85), "full heat gives p_high");
  Expect(Near(HeatFireThreshold(0.55, 0.85, 0.3, 0.5, 0.2, 0.4), 0.61), "no relax before relax_s");
  Expect(Near(HeatFireThreshold(0.55, 0.85, 0.3, 0.5, 0.2, 0.75), 0.455), "half way to the floor");
  Expect(Near(HeatFireThreshold(0.55, 0.85, 0.3, 0.5, 0.2, 5.0), 0.3), "floor after 2 relax_s");
  Expect(Near(HeatFireThreshold(0.55, 0.85, 0.3, 0.5, 0.8, 5.0), 0.79), "no relax at high heat");
  return EXIT_SUCCESS;
}
