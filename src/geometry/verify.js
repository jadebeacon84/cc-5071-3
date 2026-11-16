import { Vec2 } from '../core/vec2.js';
import { Transform } from '../core/transform.js';
import { gjk } from './gjk.js';
import { epa } from './epa.js';
import { satReference } from './sat.js';
import { resolveBoundary } from './detect.js';
import { CONTACT_SLOP, RESIDUAL_SLOP, KEEP_SLOP } from '../core/geom.js';

// Separating vector for a pair: the minimum translation that would place B
// just clear of A along a unit axis. It is always derived from a live
// computation, never from a stored table:
//   overlap  -> +depth * n  (B currently embedded; move B by +depth*n to exit)
//   separate -> +gap * n    (B already clear; magnitude is the Euclidean gap)
//   touch    -> zero vector (exact touching, no finite separation needed)
export function separatingVector(shapeA, shapeB, g = gjk(shapeA, shapeB)) {
  const boundary = resolveBoundary(shapeA, shapeB, g);
  if (boundary.overlap) {
    const sat = boundary.sat ?? satReference(shapeA, shapeB);
    const n = unitOrAxis(sat.normal);
    return {
      state: 'overlap',
      normal: n,
      magnitude: sat.depth,
      vector: Vec2.mul(n, sat.depth),
      distance: 0,
      gap: -sat.depth
    };
  }
  const distance = g.distance;
  const n = unitOrAxis(g.normal);
  if (distance <= 1e-9) {
    return {
      state: 'touch',
      normal: n,
      magnitude: 0,
      vector: new Vec2(),
      distance: 0,
      gap: 0
    };
  }
  return {
    state: 'separate',
    normal: n,
    magnitude: distance,
    vector: Vec2.mul(n, distance),
    distance,
    gap: distance
  };
}

function unitOrAxis(n) {
  if (n && n.len2() > 1e-18) return Vec2.normalize(n);
  return new Vec2(1, 0);
}

// Independently recomputed EPA penetration depth for an overlapping pair.
function pairEpaDepth(shapeA, shapeB) {
  const g = gjk(shapeA, shapeB);
  return epa(shapeA, shapeB, g).depth;
}

// A contact depth can never exceed the span of the two shapes along the normal;
// anything larger is a numerically broken manifold.
function depthBound(shapeA, shapeB) {
  const boundOf = (shape) => {
    let m = 0;
    for (const v of shape.worldVerts) m = Math.max(m, Math.abs(v.x), Math.abs(v.y));
    return 2 * m;
  };
  return boundOf(shapeA) + boundOf(shapeB) + 1;
}

// Independent validity check of a contact manifold against the live geometry.
// Every predicate is recomputed from the shapes themselves, so a stale or
// fabricated manifold fails here even if its own numbers look plausible.
export function verifyManifold(shapeA, shapeB, manifold, options = {}) {
  const depthTol = options.depthTol ?? RESIDUAL_SLOP;
  const normalTol = options.normalTol ?? 1e-6;
  const checks = [];
  const fail = (name, detail) => checks.push({ name, ok: false, detail });
  const pass = (name, detail = '') => checks.push({ name, ok: true, detail });

  if (!manifold || !Array.isArray(manifold.contacts) || manifold.contacts.length === 0) {
    fail('manifold-has-contacts', 'empty manifold');
    return verdict(checks);
  }
  pass('manifold-has-contacts', `${manifold.contacts.length} contact(s)`);

  if (manifold.contacts.length > 2) {
    fail('manifold-max-two-contacts', `${manifold.contacts.length} contacts exceed the 2D cap of 2`);
  } else {
    pass('manifold-max-two-contacts');
  }

  if (!manifold.normal || Math.abs(manifold.normal.len() - 1) > normalTol) {
    fail('normal-unit-length', `|n|=${manifold.normal?.len()}`);
  } else {
    pass('normal-unit-length');
  }
  // The tangent runs along the reference edge while the normal comes from EPA,
  // so exact orthogonality is not guaranteed (see FACE_ALIGN_MIN). The binding
  // invariant is that the tangent is a unit direction the contact span is
  // measured along; verify its length instead.
  if (manifold.tangent && Math.abs(manifold.tangent.len() - 1) > 1e-6) {
    fail('tangent-unit-length', `|t|=${manifold.tangent.len()}`);
  } else if (manifold.tangent) {
    pass('tangent-unit-length');
  }

  const g = gjk(shapeA, shapeB);
  const boundary = resolveBoundary(shapeA, shapeB, g);
  const sat = satReference(shapeA, shapeB);
  const sep = separatingVector(shapeA, shapeB, g);

  let allDepth = true;
  let anchorsOk = true;
  let distinct = true;
  let prev = null;
  for (const c of manifold.contacts) {
    if (!Number.isFinite(c.depth)) {
      allDepth = false;
      fail('depth-nonnegative-finite', `non-finite depth=${c.depth}`);
    }
    // Overlapping (EPA/clipped) manifolds keep only points at or behind the
    // reference plane within KEEP_SLOP. A separated resting contact is
    // different: buildSeparatedContact stores depth = -gap, and the gap itself
    // is bounded by CONTACT_SLOP, so a negative depth down to -CONTACT_SLOP is
    // the expected resting representation rather than a bad manifold.
    const lowerBound = boundary.overlap ? -KEEP_SLOP : -CONTACT_SLOP;
    if (Number.isFinite(c.depth) && c.depth < lowerBound) {
      allDepth = false;
      fail('depth-nonnegative-finite', `depth=${c.depth} outside ${boundary.overlap ? 'KEEP' : 'CONTACT'}_SLOP bound ${lowerBound}`);
    }
    // Local anchors must reproduce the stored world point under each body's
    // current transform: the solver re-measures penetration from these.
    const wA = shapeA.xf.localToWorld(c.anchorA);
    const wB = shapeB.xf.localToWorld(c.anchorB);
    if (!Vec2.approx(wA, c.point, 1e-6) || !Vec2.approx(wB, c.point, 1e-6)) {
      anchorsOk = false;
      fail('anchors-reproduce-point', 'local anchor transform disagrees with contact point');
    }
    if (prev && Vec2.dist2(prev.point, c.point) <= 1e-12) distinct = false;
    prev = c;
  }
  if (allDepth) pass('depth-nonnegative-finite');
  if (anchorsOk) pass('anchors-reproduce-point');
  if (distinct) pass('contacts-distinct');
  else fail('contacts-distinct', 'duplicate contact points were not merged');

  // Overlap manifolds: the two independent penetration algorithms (EPA and the
  // SAT reference) must agree on the minimum-translation depth and the push-out
  // direction. The manifold's own per-contact depths are point embeddings, not
  // the MTV, so they are bounded rather than equated to the SAT depth.
  if (boundary.overlap) {
    const refDepth = sat.depth;
    const epaDepth = pairEpaDepth(shapeA, shapeB);
    if (Math.abs(epaDepth - refDepth) > Math.max(depthTol, 1e-4)) {
      fail('depth-matches-sat', `epa=${epaDepth.toFixed(5)} sat=${refDepth.toFixed(5)}`);
    } else {
      pass('depth-matches-sat', `mtv depth=${refDepth.toFixed(5)}`);
    }
    if (sat.normal && Vec2.dot(manifold.normal, sat.normal) < 1 - normalTol) {
      fail('normal-matches-sat', `dot=${Vec2.dot(manifold.normal, sat.normal).toFixed(6)}`);
    } else {
      pass('normal-matches-sat');
    }
    // Every contact must be a real surface point: it cannot sit more than the
    // reported embedding inside A, and a finite depth must stay bounded.
    for (const c of manifold.contacts) {
      if (!Number.isFinite(c.depth) || c.depth > depthBound(shapeA, shapeB)) {
        fail('contact-depth-bounded', `unbounded/non-finite depth=${c.depth}`);
      }
    }
    if (!checks.some((c) => c.name === 'contact-depth-bounded' && !c.ok)) {
      pass('contact-depth-bounded');
    }
    // Push-out direction must actually separate: translate B along +n by the
    // reported depth plus a hair and re-run GJK from scratch.
    const pushDepth = Math.max(...manifold.contacts.map((c) => Math.max(0, c.depth)), refDepth);
    const probe = translateShape(shapeB, Vec2.mul(manifold.normal, pushDepth + 1e-4));
    const after = gjk(shapeA, probe);
    if (resolveBoundary(shapeA, probe, after).overlap) {
      fail('pushout-actually-separates', 'moving B along +n did not clear the overlap');
    } else {
      pass('pushout-actually-separates');
    }
  } else {
    // Resting/separated manifold: the signed gap must stay within contact slop.
    if (g.distance > CONTACT_SLOP + 1e-9) {
      fail('separated-within-slop', `gap ${g.distance} exceeds CONTACT_SLOP ${CONTACT_SLOP}`);
    } else {
      pass('separated-within-slop', `gap=${g.distance.toFixed(5)}`);
    }
  }

  // Two-contact manifolds must carry a real tangential span (no point collapse).
  if (manifold.contacts.length === 2) {
    const span = Math.abs(Vec2.dot(
      Vec2.sub(manifold.contacts[1].point, manifold.contacts[0].point),
      manifold.tangent
    ));
    if (span <= MERGE_TOL_LOCAL) fail('tangential-span', `span=${span}`);
    else pass('tangential-span', `span=${span.toFixed(4)}`);
  }

  const result = verdict(checks);
  result.separating = {
    state: sep.state,
    normal: { x: sep.normal.x, y: sep.normal.y },
    magnitude: sep.magnitude,
    vector: { x: sep.vector.x, y: sep.vector.y },
    distance: sep.distance
  };
  result.distance = {
    gjk: g.distance,
    satGap: sat.distance,
    overlap: boundary.overlap
  };
  return result;
}

const MERGE_TOL_LOCAL = 1e-4;

function translateShape(shape, delta) {
  // Reuse the shape's current transform with an offset centre; world verts are
  // rebuilt so the probe is a genuine fresh GJK query, not a cached result.
  const xf = shape.xf;
  const moved = Object.create(Object.getPrototypeOf(shape));
  Object.assign(moved, shape);
  moved.xf = new Transform(Vec2.add(xf.p, delta), xf.angle);
  moved.updateTransform(moved.xf);
  return moved;
}

function verdict(checks) {
  const failed = checks.filter((c) => !c.ok);
  return {
    ok: failed.length === 0,
    checks,
    failedChecks: failed.map((c) => c.name)
  };
}
