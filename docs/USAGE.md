# Usage

## Graphical window (primary entry)

Start the local, fully offline server and open the printed URL:

```sh
sh scripts/start-ui.sh
# default port 8080; override with PORT=9000 sh scripts/start-ui.sh
```

Controls:

| Action | How |
| --- | --- |
| Move a convex object | press and drag any dynamic body (applies a real impulse) |
| Single solver step | **Step** (pauses continuous motion first) |
| Continuous / pause | **Play / Pause** |
| Change scene | the **Scene** selector |
| Simulation speed | the **Speed** selector (fixed 60 Hz timestep underneath) |
| Inspect geometry | Support / EPA / Manifold / **Separation** / AABB toggles |
| Reset | **Reset** rebuilds the scene from its source definition |

The side panel shows, per active pair, the live manifold-validity verdict, the
push-out normal, EPA and SAT depths, the GJK distance, the separating vector
with its state (`overlap`, `touch`, `separate`), and the per-contact impulse
trace. A separate section reports each body's degenerate-shape status. Any
exception from the geometry or solver path is shown in a red banner at the top.

## Headless CLI (assistant)

All commands run without a browser or network.

```sh
# List the five built-in scenes.
node tools/cli.js scenes

# Simulate one scene and persist its raw record.
node tools/cli.js run stack --frames 240 --out /tmp/stack.record.json

# Re-verify a raw record against a fresh replay (exit non-zero on any failure).
node tools/cli.js verify --record /tmp/stack.record.json --out /tmp/stack.verify.json

# Simulate + independently verify every scene in one pass.
node tools/cli.js check --frames 240 --dir release/evidence/local-runs/runs

# Explicit degenerate convex status probe.
node tools/cli.js degenerate --out /tmp/degenerate.json
```

`sh scripts/headless-check.sh` runs the degenerate probe and the full
multi-scene check together and is what the clean release builder invokes.

## Record format

A `*.record.json` file uses schema `convex-contact-record/v1` and contains, per
frame and per pair: GJK distance and stop reason, EPA depth/normal/method, SAT
depth, the serialized manifold, the computed separating vector, the frame's
manifold-validity verdict, and the solver's impulse/residual trace. These are
raw outputs captured at simulation time — there are no shipped contact-point
fixtures, and verification always recomputes from geometry.
