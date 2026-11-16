# Convex Contact Geometry Kit

A dependency-free 2D convex contact toolkit: real GJK/EPA geometry, SAT
cross-checking, clipped contact manifolds, a sequential-impulse solver,
headless verification, and a graphical inspection window. Everything runs
offline with the Node.js standard library and a browser canvas — no third-party
packages and no precomputed contact-point files.

## Entry points

### Graphical entry (primary)

```sh
sh scripts/start-ui.sh
# or: npm start / npm run ui  ->  http://localhost:8080
```

The canvas window is the main inspection entry. In it you can:

- **drag a convex prop** with the pointer to apply a real impulse and move it;
- **single-step the solver** with the **Step** button (Play/Pause for continuous);
- switch scenes (`push`, `stack`, `wedge`, `rotated`, `degenerate`);
- toggle GJK support trail, EPA polytope, the contact manifold, the
  **separating vector**, and AABBs;
- read the live side panel: per-pair manifold validity, normal, EPA/SAT depth,
  GJK distance, separating vector, impulse traces, and explicit degenerate
  shape status.

The separating vector and manifold validity shown in the window are recomputed
from the live geometry every frame (see `src/geometry/verify.js`); they are
never read from stored data. Geometry/solver exceptions surface as a red
banner in the window instead of being silently dropped.

### Command line (assistant only)

The CLI only assists the graphical entry with offline runs and evidence.

```sh
node tools/cli.js scenes                         # list scenes
node tools/cli.js run push --frames 240 --out r.json   # raw per-frame record
node tools/cli.js verify --record r.json         # independent re-verification
node tools/cli.js check --dir release/evidence/local-runs/runs  # all scenes
node tools/cli.js degenerate                     # explicit degenerate status
sh scripts/headless-check.sh                     # degenerate + full check
```

## What is verified

`src/geometry/verify.js` recomputes, from the shapes themselves:

- manifold structure (1–2 contacts, unit normal/tangent, distinct points,
  local anchors that reproduce the world contact point, bounded depths);
- EPA vs independent SAT agreement on minimum-translation depth and normal;
- that moving body B along the push-out normal by the reported depth actually
  separates it (a fresh GJK query, not a cached answer);
- the **separating vector** (`overlap` / `touch` / `separate`) and GJK
  distance;
- resting contacts within `CONTACT_SLOP`.

A raw record stores each frame's measurements as they were produced.
`verifyRecord` replays the scene from scratch and recomputes every predicate,
so an edited or forged record (distance, manifold-validity flag, or separating
vector) fails verification. See `tests/cli-record.test.js`.

## Degenerate shapes

Degenerate convex input is never silently accepted. `src/geometry/validate.js`
returns an explicit status — `valid`, `point`, `line`, `insufficient`,
`non-finite`, `concave`, or `bad-winding` — and `RotatedConvex`/`buildHull`
throw with that status attached. Run `node tools/cli.js degenerate` for the
probe table; the graphical panel lists each constructed body's status.

## Testing and release

```sh
npm test          # node:test unit suite (geometry, solver, validation, CLI)
npm run release   # clean-directory packaging with run evidence (see RELEASE.md)
```

## Layout

```
src/core/       vec2, mat22, rigid transform, tolerances
src/geometry/   shapes, GJK, EPA, SAT, manifold, detection, validation, verify, scenes
src/physics/    body, world (broad/narrow phase, audit), sequential-impulse solver
renderer/       canvas UI (drag, step, live manifold/separation inspection)
tools/          headless CLI, record/replay, clean-directory release builder
scripts/        launchers (start-ui.sh, headless-check.sh)
tests/          node:test suites
docs/           USAGE.md, RELEASE.md, VERIFICATION.md
release/        archive + committed run evidence (staging dir is gitignored)
```
