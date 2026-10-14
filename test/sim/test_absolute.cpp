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
  enc.update(); // first reading, as setup() does

  // 1. Turn slowly (0.25 deg at a time) two revolutions each way, reading after every nudge.
  //    The displacement is a running total since resetDisplacement(), so it must follow both turns.
  for (int sign : {+1, -1})
  {
    int c0 = checks, f0 = fails, prev = sim::true_position(), sectors = 0;
    enc.resetDisplacement();
    for (double moved = 0; moved < 720; moved += 0.25)
    {
      sim::shaft_deg += 0.25 * sign;
      enc.update();
      enc.update(); // a reading is only used once two updates agree on it
      int pos = sim::true_position();
      expect(fabs(enc.getAngle() - pos * step) < 1e-3, "angle", enc.getAngle(), pos * step);
      if (pos != prev)
        sectors += sign;
      Direction want = sectors == 0 ? Direction::NONE : sectors > 0 ? Direction::CW : Direction::CCW;
      if (enc.getDirection() != want && fails < 20)
        printf("  FAIL direction    shaft %7.2f deg: got %s, want %s\n", sim::shaft_deg, name(enc.getDirection()), name(want));
      checks++;
      fails += enc.getDirection() != want;
      expect(fabs(enc.getDisplacement() - abs(sectors) * step) < 1e-2, "displacement", enc.getDisplacement(), abs(sectors) * step);
      prev = pos;
    }
    printf("%s slow sweep, 2 turns: %d/%d checks passed\n", sign > 0 ? "Forward" : "Backward", checks - c0 - (fails - f0), checks - c0);
  }

  // 2. Jump between every pair of positions (what happens between two update() calls in the sketch).
  //    Jumps up to MAX_STEP_DEG are counted; bigger ones are glitches: ignored until the same reading
  //    has come back CONFIRM_READS times, then the angle follows but the displacement doesn't.
  int c0 = checks, f0 = fails;
  for (int a = 0; a < n; a++)
    for (int b = 0; b < n; b++)
    {
      sim::shaft_deg = (a + 0.5) * step;
      for (int i = 0; i < Absolute::CONFIRM_READS; i++)
        enc.update();
      enc.resetDisplacement();
      sim::shaft_deg = (b + 0.5) * step;
      enc.update();
      enc.update();
      int d = ((b - a) % n + n) % n;
      double moved = (d <= n / 2 ? d : n - d) * step;
      bool real = moved <= Absolute::MAX_STEP_DEG;
      double want = real ? moved : 0;
      expect(fabs(enc.getDisplacement() - want) < 1e-3, "jump", enc.getDisplacement(), want);
      expect(fabs(enc.getAngle() - (real ? b : a) * step) < 1e-3, "jump angle", enc.getAngle(), (real ? b : a) * step);
      for (int i = 2; i < Absolute::CONFIRM_READS; i++)
        enc.update();
      expect(fabs(enc.getAngle() - b * step) < 1e-3, "resync", enc.getAngle(), b * step);
      expect(fabs(enc.getDisplacement() - want) < 1e-3, "resync disp", enc.getDisplacement(), want);
    }
  printf("Jumps between all %d x %d positions: %d/%d checks passed\n", n, n, checks - c0 - (fails - f0), checks - c0);
  printf("  note: a jump of more than %.0f deg between two update() calls is taken as a sensor glitch, so the\n"
         "  sketch must call update() often while the shaft turns.\n", Absolute::MAX_STEP_DEG);

  // 3. Sensor dropouts while turning: every pin reads black (code 0) for single updates, as seen on the
  //    rig. Before the glitch filter, three dropouts at 180 deg turned a 45 deg move into "1215 degrees CCW".
  c0 = checks, f0 = fails;
  for (double start = step / 4; start < 360; start += step / 2) // never on a sector edge
  {
    sim::shaft_deg = start;
    for (int i = 0; i < Absolute::CONFIRM_READS; i++)
      enc.update();
    enc.resetDisplacement();
    int prev = sim::true_position(), sectors = 0;
    for (int k = 1; k <= 180; k++) // 90 deg in 0.5 deg steps
    {
      sim::shaft_deg += 0.5;
      if (k % 7 == 3)
        sim::dark_reads = 5; // one whole update() reads dark
      enc.update();
      enc.update();
      int pos = sim::true_position();
      sectors += pos != prev;
      prev = pos;
    }
    sim::dark_reads = 5; // and the final reading drops out too
    enc.update();
    expect(fabs(enc.getDisplacement() - sectors * step) < 1e-3, "dropouts", enc.getDisplacement(), sectors * step);
    expect(enc.getDirection() == Direction::CW, "dropout dir", (int)enc.getDirection(), 1);
  }
  printf("90 deg moves with sensor dropouts, from %d start angles: %d/%d checks passed\n", 2 * n, checks - c0 - (fails - f0), checks - c0);

  printf("\n%s: %d of %d checks passed\n", fails ? "ABSOLUTE LIBRARY FAILED" : "ABSOLUTE LIBRARY OK", checks - fails, checks);
  return fails ? 1 : 0;
}
