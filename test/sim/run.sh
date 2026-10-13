#!/usr/bin/env bash
# Builds and runs the encoder simulation against the real project code (nothing in proj/ is changed).
# From the repo root in a Windows terminal:   wsl bash test/sim/run.sh [degrees] [seconds]
set -u
cd "$(dirname "$0")"
SRC=../../proj/BaseCodeTwo-Absolute
WHEEL=../../wheel_gray5.dxf
OUT=/tmp/encoder-sim
mkdir -p "$OUT"
CXX="g++ -std=gnu++17 -Wall -Wno-unused-variable -Wno-psabi -I. -I$SRC"
status=0

echo "=== 1. Absolute library on the simulated wheel ($WHEEL)"
if $CXX sim.cpp test_absolute.cpp "$SRC/Absolute.cpp" -o "$OUT/test_absolute"; then
  "$OUT/test_absolute" "$WHEEL" || status=1
else
  echo "Absolute library DOES NOT COMPILE"; status=1
fi

echo
echo "=== 2. Full sketch with motor + wheel, typing ${1:-90} deg, ${2:-60} s"
if $CXX -DSKETCH_INO="\"$SRC/BaseCodeTwo-Absolute.ino\"" sim.cpp run_sketch.cpp "$SRC/Absolute.cpp" -o "$OUT/run_sketch"; then
  "$OUT/run_sketch" "$WHEEL" "${1:-90}" "${2:-60}" || status=1
else
  echo "Sketch DOES NOT COMPILE (errors above; the same code fails in the Arduino IDE)"; status=1
fi
exit $status
