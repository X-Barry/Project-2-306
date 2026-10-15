#!/usr/bin/env python3
"""Turns Serial Monitor logs from the performance sketches into tables and figures.

1. Copy the Serial Monitor text into any .txt / .csv file in performance_v2/data/.
2. Run:  python analyze.py
3. Tables (.csv) and figures (.png) are written to performance_v2/results/.

Lines are found by their tag (ABS, QUAD or RAW), so Serial Monitor timestamps, # comments and
any other text are ignored. All ABS / QUAD lines in data/ are pooled, so several sessions can be
saved as separate files. Every RAW capture is analysed on its own.

Errors are always "own encoder - built-in encoder", not "own - commanded", because the PI
controller does not reach the commanded value exactly.

Options:
  --data DIR        folder with the logs (default: data/)
  --out DIR         folder for the results (default: results/)
  --window 5|10|all      which quadrature window to use (default 10 = the 5-10 s demo window)
  --pair A1 A5      the two quadrature channels in RAW captures (default A1 A5)
"""

import argparse
import csv
import re
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
LINE = re.compile(r"\b(ABS|QUAD|RAW),(\S+)")

ABS_COLUMNS = ["cmd", "rep", "dir_cmd", "dir_own", "builtin", "own", "error", "own_pos"]
QUAD_COLUMNS = ["cmd", "run", "t_s", "dir_cmd", "dir_builtin", "dir_own", "builtin", "own", "error"]
NUMERIC = {"cmd", "rep", "run", "t_s", "builtin", "own", "error", "own_pos"}

ABS_LIMIT = 14.0  # rubric: full marks within +-14 deg
QUAD_LIMIT = 2.5  # rubric: full marks within +-2.5 RPM

plt.rcParams.update({"font.size": 11, "figure.dpi": 150, "savefig.bbox": "tight"})


# ---------------------------------------------------------------- reading the logs


def read_logs(folder):
    """Returns (abs rows, quad rows, raw captures) from every file in the folder."""
    abs_rows, quad_rows, raw_captures = [], [], []
    files = sorted(p for p in folder.iterdir() if p.is_file() and p.suffix.lower() in (".txt", ".csv", ".log"))
    if not files:
        print(f"No .txt/.csv/.log files in {folder}. Copy the Serial Monitor output there first.")
    for path in files:
        current, last_t = None, None
        for line in path.read_text(errors="replace").splitlines():
            m = LINE.search(line)
            if not m:
                continue
            tag, fields = m.group(1), m.group(2).split(",")
            if tag == "ABS" and len(fields) == len(ABS_COLUMNS):
                abs_rows.append(parse(ABS_COLUMNS, fields, path.name))
            elif tag == "QUAD" and len(fields) == len(QUAD_COLUMNS):
                quad_rows.append(parse(QUAD_COLUMNS, fields, path.name))
            elif tag == "RAW" and len(fields) == 7:
                try:
                    values = [float(v) for v in fields]
                except ValueError:
                    continue
                if current is None or values[0] < last_t:  # time restarts at 0 for every capture
                    current = {"file": path.stem, "rows": []}
                    raw_captures.append(current)
                current["rows"].append(values)
                last_t = values[0]
    return abs_rows, quad_rows, raw_captures


def parse(columns, fields, source):
    row = {"file": source}
    try:
        for name, value in zip(columns, fields):
            row[name] = float(value) if name in NUMERIC else value
    except ValueError:
        return {}
    return row


# ---------------------------------------------------------------- statistics


def stats(errors):
    e = np.asarray(errors, dtype=float)
    abs_e = np.sort(np.abs(e))
    best = len(e) - 2 if len(e) >= 10 else len(e)  # the demo's "8 best out of 10"; fewer than 10: all of them
    return {
        "n": len(e),
        "mean_error": e.mean(),
        "std_error": e.std(ddof=1) if len(e) > 1 else 0.0,
        "max_abs_error": abs_e[-1],
        "best_avg_abs_error": abs_e[:best].mean(),
    }


def linear_fit(x, y):
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    if len(x) < 2 or np.ptp(x) == 0:
        return np.nan, np.nan, np.nan
    slope, offset = np.polyfit(x, y, 1)
    residual = y - (slope * x + offset)
    total = ((y - y.mean()) ** 2).sum()
    r2 = 1 - (residual**2).sum() / total if total > 0 else np.nan
    return slope, offset, r2


def direction_of(cmd):
    return "CW" if cmd > 0 else "CCW"


def analyse(rows, own_dir_key, unit, name, out, limit):
    """Per-command and per-direction tables plus the two figures, for ABS or QUAD rows."""
    rows = [r for r in rows if r]
    if not rows:
        return
    commands = sorted({r["cmd"] for r in rows}, key=lambda c: (abs(c), -c))

    per_command = []
    for cmd in commands:
        group = [r for r in rows if r["cmd"] == cmd]
        st = stats([r["error"] for r in group])
        correct = sum(r[own_dir_key] == direction_of(cmd) for r in group)
        per_command.append(
            {
                "command": cmd,
                "direction": direction_of(cmd),
                **st,
                "mean_builtin": np.mean([r["builtin"] for r in group]),
                "mean_own": np.mean([r["own"] for r in group]),
                "direction_correct": f"{correct}/{len(group)}",
            }
        )

    per_direction = []
    for direction in ("CW", "CCW"):
        group = [r for r in rows if direction_of(r["cmd"]) == direction]
        if not group:
            continue
        st = stats([r["error"] for r in group])
        slope, offset, r2 = linear_fit([r["builtin"] for r in group], [r["own"] for r in group])
        correct = sum(r[own_dir_key] == direction for r in group)
        per_direction.append(
            {
                "direction": direction,
                **st,
                "fit_slope": slope,
                f"fit_offset_{unit}": offset,
                "fit_r2": r2,
                "direction_correct": f"{correct}/{len(group)}",
            }
        )

    write_table(out / f"{name}_per_command.csv", per_command)
    write_table(out / f"{name}_per_direction.csv", per_direction)
    print_table(f"{name}: per command ({unit}, error = own - built-in)", per_command)
    print_table(f"{name}: per direction", per_direction)

    label = "absolute encoder" if name == "absolute" else "quadrature encoder"
    quantity = "displacement" if name == "absolute" else "speed"

    # measured vs built-in, with y = x
    fig, ax = plt.subplots(figsize=(6, 5))
    top = max(max(r["builtin"] for r in rows), max(r["own"] for r in rows)) * 1.05
    ax.plot([0, top], [0, top], "k--", lw=1, label="y = x (perfect)")
    for direction, marker, colour in (("CW", "o", "tab:blue"), ("CCW", "s", "tab:orange")):
        group = [r for r in rows if direction_of(r["cmd"]) == direction]
        if not group:
            continue
        x = np.array([r["builtin"] for r in group])
        y = np.array([r["own"] for r in group])
        slope, offset, r2 = linear_fit(x, y)
        ax.scatter(x, y, marker=marker, s=18, alpha=0.6, color=colour, label=direction)
        if not np.isnan(slope):
            xs = np.array([0, top])
            ax.plot(xs, slope * xs + offset, color=colour, lw=1,
                    label=f"{direction} fit: y = {slope:.3f}x {offset:+.2f}, R² = {r2:.4f}")
    ax.set_xlabel(f"Built-in encoder {quantity} ({unit})")
    ax.set_ylabel(f"Optical {label} {quantity} ({unit})")
    ax.set_xlim(0, top)
    ax.set_ylim(0, top)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.savefig(out / f"{name}_measured_vs_builtin.png")
    plt.close(fig)

    # error vs commanded value, mean +- 1 std, every reading shown faintly
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for direction, marker, colour, shift in (("CW", "o", "tab:blue", -0.01), ("CCW", "s", "tab:orange", 0.01)):
        table = [t for t in per_command if t["direction"] == direction]
        if not table:
            continue
        span = max(abs(c) for c in commands)
        x = np.array([abs(t["command"]) for t in table])
        ax.errorbar(x + shift * span, [t["mean_error"] for t in table], yerr=[t["std_error"] for t in table],
                    fmt=marker, color=colour, capsize=4, label=f"{direction} mean ± 1 std")
        group = [r for r in rows if direction_of(r["cmd"]) == direction]
        ax.scatter([abs(r["cmd"]) + shift * span for r in group], [r["error"] for r in group],
                   s=8, color=colour, alpha=0.25)
    ax.axhline(0, color="k", lw=0.8)
    for y in (limit, -limit):
        ax.axhline(y, color="tab:red", ls=":", lw=1)
    ax.text(ax.get_xlim()[1], limit, f" ±{limit:g} {unit} (full marks)", color="tab:red", va="bottom", ha="right", fontsize=8)
    ax.set_xlabel(f"Commanded {quantity} magnitude ({unit})")
    ax.set_ylabel(f"Error, own − built-in ({unit})")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.savefig(out / f"{name}_error_vs_command.png")
    plt.close(fig)


# ---------------------------------------------------------------- raw waveforms


def edges(digital):
    """Indices where a 0/1 signal goes from 0 to 1."""
    return np.flatnonzero(digital[1:] & ~digital[:-1]) + 1


def analyse_raw(captures, pair, out):
    if not captures:
        return
    summary = []
    for k, capture in enumerate(captures, start=1):
        data = np.array(capture["rows"])
        t = (data[:, 0] - data[0, 0]) / 1e6  # seconds
        values = data[:, 1:]
        name = f"{capture['file']}_capture{k}"
        active = [i for i in range(6) if np.ptp(values[:, i]) > 20] or list(range(6))

        fig, axes = plt.subplots(len(active), 1, figsize=(8, 1.6 * len(active) + 0.6), sharex=True, squeeze=False)
        for ax, i in zip(axes[:, 0], active):
            v = values[:, i]
            mid = (v.min() + v.max()) / 2
            ax.plot(t, v, lw=0.8)
            ax.axhline(mid, color="tab:red", ls=":", lw=0.8)
            ax.set_ylabel(f"A{i}", rotation=0, labelpad=15)
            ax.set_ylim(-20, 1043)
            ax.grid(alpha=0.3)
        axes[0, 0].set_title("Raw phototransistor signals (ADC counts 0-1023, dotted = midpoint threshold)", fontsize=10)
        axes[-1, 0].set_xlabel("Time (s)")
        fig.savefig(out / f"{name}.png")
        plt.close(fig)

        for i in range(6):
            v = values[:, i]
            summary.append({"capture": name, "pin": f"A{i}", "min": v.min(), "max": v.max(),
                            "midpoint": (v.min() + v.max()) / 2, "samples": len(v),
                            "sample_rate_hz": len(v) / t[-1] if t[-1] > 0 else np.nan})

        a, b = int(pair[0][1:]), int(pair[1][1:])
        if a in active and b in active and a != b:
            phase_line = quadrature_phase(t, values[:, a], values[:, b])
            if phase_line:
                print(f"{name}: {pair[0]} / {pair[1]}: {phase_line}")
                with open(out / "raw_quadrature_phase.txt", "a") as f:
                    f.write(f"{name}: {pair[0]} / {pair[1]}: {phase_line}\n")

    write_table(out / "raw_levels.csv", summary)
    print_table("raw captures: levels per pin (use the midpoint as a starting threshold)",
                [r for r in summary if r["max"] - r["min"] > 20] or summary)


def quadrature_phase(t, va, vb):
    """Duty cycle of each channel and the phase of B's rising edge after A's, in degrees of one period."""
    da = va >= (va.min() + va.max()) / 2
    db = vb >= (vb.min() + vb.max()) / 2
    ra, rb = edges(da), edges(db)
    if len(ra) < 3 or len(rb) < 2:
        return None
    period = np.median(np.diff(t[ra]))
    phases = []
    for i in ra[:-1]:
        later = rb[t[rb] > t[i]]
        if len(later):
            phases.append(((t[later[0]] - t[i]) / period * 360) % 360)
    if not phases:
        return None
    phases = np.array(phases)
    return (f"period {period * 1000:.1f} ms, duty A {da.mean() * 100:.1f} %, duty B {db.mean() * 100:.1f} %, "
            f"B rises {phases.mean():.1f} +/- {phases.std():.1f} deg after A (90 is ideal; 270 = B leads A)")


# ---------------------------------------------------------------- output helpers


def fmt(v):
    if isinstance(v, (float, np.floating)):
        return f"{v:.3f}" if not np.isnan(v) else "nan"
    return str(v)


def write_table(path, rows):
    if not rows:
        return
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow({k: fmt(v) for k, v in row.items()})


def print_table(title, rows):
    if not rows:
        return
    keys = list(rows[0].keys())
    cells = [[fmt(r[k]) for k in keys] for r in rows]
    widths = [max(len(k), *(len(c[i]) for c in cells)) for i, k in enumerate(keys)]
    print(f"\n{title}")
    print("  ".join(k.rjust(w) for k, w in zip(keys, widths)))
    for c in cells:
        print("  ".join(v.rjust(w) for v, w in zip(c, widths)))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", type=Path, default=HERE / "data")
    parser.add_argument("--out", type=Path, default=HERE / "results")
    parser.add_argument("--window", default="10", choices=["5", "10", "all"])
    parser.add_argument("--pair", nargs=2, default=["A1", "A5"], metavar=("A", "B"))
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "raw_quadrature_phase.txt").unlink(missing_ok=True)
    abs_rows, quad_rows, raw_captures = read_logs(args.data)
    abs_rows = [r for r in abs_rows if r]
    quad_rows = [r for r in quad_rows if r]
    if args.window != "all":
        quad_rows = [r for r in quad_rows if r["t_s"] == float(args.window)]
    print(f"Found {len(abs_rows)} ABS readings, {len(quad_rows)} QUAD readings "
          f"(window {args.window}), {len(raw_captures)} RAW captures")

    analyse(abs_rows, "dir_own", "deg", "absolute", args.out, ABS_LIMIT)
    analyse(quad_rows, "dir_own", "RPM", "quadrature", args.out, QUAD_LIMIT)
    analyse_raw(raw_captures, args.pair, args.out)
    print(f"\nTables and figures written to {args.out}")


if __name__ == "__main__":
    main()
