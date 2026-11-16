#!/usr/bin/env sh
# Offline verification entry point. Runs the headless CLI: every scene is
# actually simulated, raw records are written, then re-verified against live
# geometry (manifold validity, separating vector and distance). Exits non-zero
# on any failure, so a release cannot report a false pass.
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$ROOT"

FRAMES="${FRAMES:-240}"
OUT="${OUT:-release/evidence/local-runs}"

echo "== degenerate convex probes =="
node tools/cli.js degenerate --out "$OUT/degenerate.json"

echo "== scene simulation + manifold/distance verification =="
exec node tools/cli.js check --frames "$FRAMES" --dir "$OUT/runs"
