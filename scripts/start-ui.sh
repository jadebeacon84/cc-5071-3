#!/usr/bin/env sh
# Graphical entry point for the convex contact kit. Serves the canvas UI with
# Node's built-in http module (no dependencies, works fully offline). Open the
# printed URL, then drag a convex prop and use Step to single-step the solver.
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$ROOT"

PORT="${PORT:-8080}"
echo "Convex Contact Kit - graphical entry"
echo "open http://localhost:${PORT}/ (Ctrl+C to stop)"
PORT="$PORT" exec node tools/server.js
