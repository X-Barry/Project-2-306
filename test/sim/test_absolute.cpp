// Spins the simulated wheel under the real Absolute class and checks every reading.
// Usage: test_absolute <wheel.dxf> [threshold]
#include <cstdio>
#include <cmath>
#include "Arduino.h"
#include "Absolute.h"

static int checks = 0, fails = 0;

static void expect(bool ok, const char *what, double got, double want)
{
  checks++;
  if (!ok && fails++ < 20)
    printf("  FAIL %-12s shaft %7.2f deg (wheel position %2d): got %.2f, want %.2f\n",
           what, sim::shaft_deg, sim::true_position(), got, want);
}

static const char *name(Direction d) { return d == Direction::CW ? "CW" : d == Direction::CCW ? "CCW" : "NONE"; }

int main(int argc, char **argv)
{
  sim::load_wheel(argc > 1 ? argv[1] : "../../wheel_gray5.dxf");
  int threshold = argc > 2 ? atoi(argv[2]) : 100;
  const int n = sim::positions();
  const double step = 360.0 / n;

  printf("Sensor levels: black %.0f, white %.0f, threshold %d\n", sim::black_counts, sim::white_counts, threshold);
  if (!(sim::white_counts < threshold && threshold < sim::black_counts))
    printf("  WARNING threshold is not between the white and black readings: every bit will read the same\n");

  sim::shaft_deg = step / 2;
  Absolute enc(threshold, 5);

  // 1. Turn slowly (0.25 deg at a time) two revolutions each way, reading after every nudge.
  for (int sign : {+1, -1})
  {
    int c0 = checks, f0 = fails, prev = sim::true_position();
    for (double moved = 0; moved < 720; moved += 0.25)
    {
      sim::shaft_deg += 0.25 * sign;
      enc.update();
      int pos = sim::true_position();
      expect(fabs(enc.getAngle() - pos * step) < 1e-3, "angle", enc.getAngle(), pos * step);
      bool moved_one = pos != prev;
      Direction want = !moved_one ? Direction::NONE : sign > 0 ? Direction::CW : Direction::CCW;
      if (enc.getDirection() != want && fails < 20)
        printf("  FAIL direction    shaft %7.2f deg: got %s, want %s\n", sim::shaft_deg, name(enc.getDirection()), name(want));
      checks++;
      fails += enc.getDirection() != want;
      expect(fabs(enc.getDisplacement() - (moved_one ? step : 0)) < 1e-3, "displacement", enc.getDisplacement(), moved_one ? step : 0);
      prev = pos;
    }
    printf("%s slow sweep, 2 turns: %d/%d checks passed\n", sign > 0 ? "Forward" : "Backward", checks - c0 - (fails - f0), checks - c0);
  }

  // 2. Jump between every pair of positions (what happens between two update() calls in the sketch).
  int c0 = checks, f0 = fails;
  for (int a = 0; a < n; a++)
    for (int b = 0; b < n; b++)
    {
      sim::shaft_deg = (a + 0.5) * step;
      enc.update();
      sim::shaft_deg = (b + 0.5) * step;
      enc.update();
      int d = ((b - a) % n + n) % n;
      double want = (d <= n / 2 ? d : n - d) * step;
      expect(fabs(enc.getDisplacement() - want) < 1e-3, "jump", enc.getDisplacement(), want);
    }
  printf("Jumps between all %d x %d positions: %d/%d checks passed\n", n, n, checks - c0 - (fails - f0), checks - c0);
  printf("  note: update() can't tell a %.0f deg move from a %.0f deg one the other way; a move of more than\n"
         "  180 deg between two update() calls is reported as the shorter move in the opposite direction.\n", 270.0, 90.0);

  printf("\n%s: %d of %d checks passed\n", fails ? "ABSOLUTE LIBRARY FAILED" : "ABSOLUTE LIBRARY OK", checks - fails, checks);
  return fails ? 1 : 0;
}
