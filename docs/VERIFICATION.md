# Verification reference

All manifold and distance claims come from live computation
(`src/geometry/verify.js`), shared by the graphical window and the CLI.

## Separating vector

`separatingVector(shapeA, shapeB)` runs GJK and, for boundary candidates, the
SAT reference, then returns:

| state | meaning | vector |
| --- | --- | --- |
| `overlap` | B is embedded in A | `+depth * n`, the translation that clears B |
| `touch` | exact touching, zero gap | zero vector |
| `separate` | disjoint | `+gap * n` along the unit separating axis |

`n` always follows the module convention that the solver pushes body B along
`+n`. The magnitude equals the measured SAT depth (overlap) or GJK Euclidean
distance (separate).

## Manifold validity predicates

`verifyManifold` recomputes each of the following from the current shapes and
fails the verdict if any does not hold:

- `manifold-has-contacts`, `manifold-max-two-contacts`, `contacts-distinct`;
- `normal-unit-length`, `tangent-unit-length`;
- `depth-nonnegative-finite` — overlap contacts stay within `KEEP_SLOP`,
  separated resting contacts within `CONTACT_SLOP`;
- `anchors-reproduce-point` — each contact's local anchors transform back to
  its world point under both bodies' current transforms;
- `depth-matches-sat` — independently recomputed EPA depth agrees with the SAT
  reference within tolerance (minimum-translation depth, not point embedding);
- `normal-matches-sat` — push-out direction agreement;
- `contact-depth-bounded` — no non-finite or geometrically impossible depth;
- `pushout-actually-separates` — translating B along `+n` by the depth and
  re-running GJK from scratch reports no overlap;
- `separated-within-slop` — a non-overlap manifold only exists within
  `CONTACT_SLOP`;
- `tangential-span` — a two-contact manifold keeps a real span.

## Distance cross-check

For separated pairs the GJK Euclidean distance is additionally checked against
the brute-force closest-point reference in `tests/gjk-epa.test.js`. During
record verification the replayed live GJK gap must equal the recorded gap to
within `1e-9`, and the recorded separating vector must equal the recomputed
one to within `1e-7`.

## Record anti-forgery

`verifyRecord` rebuilds the scene from its source definition and replays the
exact pre-solve / post-solve ordering, then asserts:

- recomputed manifold verdict equals the record's `manifoldValid`;
- replayed GJK distance equals the recorded distance;
- recomputed separating vector/state equals the recorded one;
- recorded solver residual counts match the replay;
- post-settle residual penetration never streaks for three frames and the
  final frame is clean (early settling and one-to-two-frame impact residuals
  are legitimate and documented).

`tests/cli-record.test.js` demonstrates that forging a distance, a
manifold-validity flag, or a separating vector is detected.

## Solver residual policy

Initial interpenetration is resolved over several frames; moving-scene
collisions can leave a short impact residual. The policy accepts residuals only
while they demonstrably converge: no streak of three violating frames after the
settle window and zero violations on the final frame. A solver that stops
recovering fails the release instead of being hidden.
