# Performance testing (v2)

This folder is self-contained: everything needed to run the performance tests is in here.

Everything here runs from the Arduino IDE. Each sketch runs a whole test on its own, with no
board resets, and prints tagged CSV lines. You copy the Serial Monitor text into a file in
`data/`, then one Python script turns it into the tables and figures for the report.

| Folder / file | What it gives you |
|---|---|
| `RawSignalLogger/` | Raw A0–A5 readings while the disk turns. Use them for the waveform figures, the resistor and threshold for each channel, and the quadrature A/B phase offset and duty cycle. |
| `AbsolutePerformance/` | BaseCodeTwo's 4 s positioning move for ±20° to ±90° in 10° steps, 2 moves each (16 commands × 2 moves, about 3 min). |
| `QuadraturePerformance/` | BaseCodeOne's speed test for ±40 to ±320 RPM in 40 RPM steps, 1 run each, cut to 10 s (16 runs, about 3 min). |
| `analyze.py` | Statistics, tables (`.csv`) and figures (`.png`) written to `results/`. |
| `data/` | Put the copied Serial Monitor logs here. |
| `demo/` | The demo codes the test sketches are copied from: `BaseCodeTwo-Absolute/` (absolute encoder, threshold 50) and `Quadrature/` (optical quadrature encoder on A1 / A5, 8 slots, hysteresis 45–55). They run at 115200 baud. |

## 1. Run a test (Arduino IDE)

1. Open the sketch's `.ino` in the Arduino IDE and upload it to the Uno.
2. Open the **Serial Monitor**. Set the baud rate to **250000** and turn **timestamps off**.
   The analyzer ignores timestamps, but they make the text harder to read.
3. Type the command in the box and press Enter:

   | Sketch | Type | Result |
   |---|---|---|
   | RawSignalLogger | `40` / `-40` | Spins the motor slowly one way or the other and logs for 5 s |
   | | `0` | Motor off. Turn the disk by hand while it logs. |
   | AbsolutePerformance | `0` | Full sweep (±20° … ±90°) |
   | | e.g. `-30` | That command only (2 moves) |
   | QuadraturePerformance | `0` | Full sweep (±40 … ±320 RPM) |
   | | e.g. `-120` | That command only (1 run) |

4. Wait for `# done` (`# capture end` for the raw logger). Click in the Serial Monitor output,
   press **Ctrl+A** then **Ctrl+C**, and paste into a new text file in `data/`. For example:
   `data/abs_sweep_2026-10-16.txt`.
   Check that the first and last `ABS` / `QUAD` lines made it into the file.

You can run more commands without re-uploading: just type the next one. Several files are fine,
because the analyzer pools every file in `data/`.

The test sketches also print a `# summary` line after each command. It shows the mean, standard
deviation, max |error| and (for the absolute encoder) the average |error|, so you
can see the results at the bench without Python.

## 2. Analyse (Python)

Once:

```bash
pip install -r requirements.txt
```

Then, from this folder:

```bash
python analyze.py
```

It prints the tables and writes to `results/`:

| Output | Contents |
|---|---|
| `absolute_per_command.csv` | Per command: n, mean error, std, max \|error\|, average |error| (best 8 of 10 once a command has 10 or more readings), mean built-in / own, direction correct |
| `absolute_per_direction.csv` | CW and CCW separately: the same statistics plus linear fit slope, offset and R² of own vs built-in |
| `absolute_measured_vs_builtin.png` | Own vs built-in displacement with the y = x line and the CW / CCW fits |
| `absolute_error_vs_command.png` | Error vs commanded angle, mean ± 1 std, every reading shown faintly, with the ±14° full-marks band |
| `quadrature_*` | The same four files for speed (RPM), with the ±2.5 RPM band |
| `<file>_captureN.png` | Raw waveform of every pin that changes during a capture, with its midpoint threshold |
| `raw_levels.csv` | Min, max and midpoint of each pin, plus the sample rate |
| `raw_quadrature_phase.txt` | Period, duty cycle of A and B, and the phase of B after A (default pins A1 / A5) |

Options: `--window 5|10|all` picks the quadrature 5 s window. The default is `10`, the 5–10 s
window the demo assesses. `--pair A1 A2` changes the quadrature pins used for raw captures.

**Errors are always own − built-in, not own − commanded.** The PI controller does not reach the
commanded value exactly, so the built-in encoder is the reference.

## Output format

Lines are found by their tag, so anything else in the file (comments starting with `#`,
timestamps) is ignored.

```
ABS,cmd_deg,rep,dir_cmd,dir_own,builtin_deg,own_deg,error_deg,own_pos_deg
QUAD,cmd_rpm,run,t_s,dir_cmd,dir_builtin,dir_own,builtin_rpm,own_rpm,error_rpm
RAW,t_us,A0,A1,A2,A3,A4,A5
```

## Things to know

- **The motor runs as in the demo.** The PI controllers and built-in encoder counting are copied
  unchanged from the demo codes. Differences:
  - The time variables are `unsigned long` instead of `int`, because an `int` overflows after
    32.7 s and a sweep runs for minutes.
  - QuadraturePerformance leaves out BaseCodeOne's 100 ms "spontaneous speed" print.
  - To keep each full sweep to about 3 minutes, AbsolutePerformance does 2 moves per angle
    (`REPS`, the demo does 10) and QuadraturePerformance does 1 run per speed (`RUNS`) that stops
    after 10 s (`RUN_MS`, the demo runs 15 s). Raise them for more data. Running a sweep again
    also adds data, because the analyzer pools every file.
  - The test sketches use 250000 baud (the demo codes use 115200) so the long logs print quickly.
  - BaseCodeTwo's `kp` is worked out before the angle is typed in, so it is always
    `0.6 × 90 / 45`. AbsolutePerformance uses that same value.
- **Keep the copies in sync.**
  - `AbsolutePerformance/Absolute.h` and `Absolute.cpp` are copies of
    `demo/BaseCodeTwo-Absolute/`. The Arduino IDE only compiles files inside the sketch folder.
  - The optical encoder section of `QuadraturePerformance.ino` is a copy of `demo/Quadrature/Quadrature.ino`.
  - If you change the originals (thresholds, pins, slots), copy the changes across before testing.
- **Wrong direction every time?** If `dir_own` is always opposite to `dir_cmd`, set
  `OWN_DIRECTION_SIGN = -1` in AbsolutePerformance, or `OPTICAL_DIRECTION_SIGN` in the quadrature
  code, then fix the same thing in the demo code.
- **Raw logger sample rate** is about 550 samples/s across all six pins. Use a slow PWM so each slot
  gets plenty of samples. A 5 s capture is about 2,800 lines. Change `LOG_MS` if you need longer.
- **Seeing missing lines after pasting?** If the Serial Monitor didn't keep the whole log, run
  fewer commands per session (e.g. type one angle at a time) and save each to its own file.
