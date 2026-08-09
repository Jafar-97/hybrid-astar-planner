#!/usr/bin/env bash
# Build (if needed), run a scenario, and render the visualisation.
# Usage: ./run.sh [obstacles|scurve]
set -euo pipefail
cd "$(dirname "$0")"

SCENARIO="${1:-obstacles}"
OUT="out/${SCENARIO}"

if [ ! -x build/planner_demo ]; then
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
  cmake --build build -j
fi

mkdir -p "$OUT"
./build/planner_demo "$SCENARIO" "$OUT"

if command -v python3 >/dev/null 2>&1; then
  python3 scripts/visualize.py "$OUT" || \
    echo "(visualisation skipped — need matplotlib/numpy/imageio)"
fi
echo "Outputs written to $OUT/"
