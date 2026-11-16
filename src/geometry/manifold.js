import { Vec2 } from '../core/vec2.js';
import {
  CONTACT_SLOP,
  EPS_LEN,
  FACE_ALIGN_MIN,
  KEEP_SLOP,
  MERGE_TOL
} from '../core/geom.js';

// Contact manifold for one (A, B) pair.
// Normal convention: n is B's push-out direction (the solver moves B along +n
// to separate), and every contact stores penetration depth >= 0 measured along
// that normal. World points are solver anchor positions.
export function buildManifold(shapeA, shapeB, epaResult) {
  // n is always B's push-out direction: the solver moves B along +n.
  const n = epaResult.normal.clone();
  const selection = chooseReference(shapeA, shapeB, n);
  const ref = selection.ref;
  const inc = selection.inc;
  const refIsA = ref === shapeA;

  // The reference edge supplies the contact face and tangent. Its own outward
  // normal may point along +n or -n depending on which body is the reference,
  // so orient it to the push-out normal explicitly.
  const rawEdge = ref.edges()[selection.edgeIndex];
  const flip = Vec2.dot(rawEdge.normal, n) < 0;
  const refEdge = flip
    ? { a: rawEdge.b, b: rawEdge.a, tangent: Vec2.neg(rawEdge.tangent), normal: n.clone(), index: rawEdge.index }
    : { a: rawEdge.a, b: rawEdge.b, tangent: rawEdge.tangent, normal: n.clone(), index: rawEdge.index };
  const normal = n.clone();
  // The tangent follows the reference edge direction: the clip planes are built
  // along it, so it must stay aligned with that edge even when the EPA normal
  // differs slightly from that edge's own outward normal (see FACE_ALIGN_MIN).
  // A zero-length degenerate edge has no direction, so fall back to the normal.
  const tangent = refEdge.tangent.len2() > EPS_LEN * EPS_LEN
    ? Vec2.normalize(refEdge.tangent)
    : Vec2.perpLeft(normal);

  const incidentEdges = inc.edges();
  const incident = chooseIncident(incidentEdges, normal);

  // Clip the incident edge segment against the two tangent-side planes of
  // the reference edge. Clipping is 1D: keep side >= 0, interpolate crossings.
  const v1 = clipSegmentToSide(incident.a, incident.b, refEdge.a, tangent);
  const segment = clipSegmentToSide(v1.a, v1.b, refEdge.b, Vec2.neg(tangent));

  // Depth is how far each (clipped) incident point is embedded behind the
  // reference plane. B is pushed out along +n, so a penetrating point lies on
  // the -n side of the plane and depth = -(its signed offset) is positive.
  let contacts = [];
  for (const p of [segment.a, segment.b]) {
    const offset = Vec2.dot(Vec2.sub(p, refEdge.a), normal);
    const depth = -offset;
    if (depth >= -KEEP_SLOP) {
      // Project the point onto the reference plane for a stable anchor.
      const point = Vec2.add(p, Vec2.mul(normal, depth));
      contacts.push(makeContact(point, normal, Math.max(0, depth), shapeA, shapeB, refIsA, tangent));
    }
  }

  if (contacts.length === 0) {
    // Numerical corner: keep the EPA witness rather than emitting nothing.
    const mid = Vec2.lerp(epaResult.witnessA, epaResult.witnessB, 0.5);
    contacts.push(makeContact(mid, normal, Math.max(0, epaResult.depth), shapeA, shapeB, refIsA, tangent));
  }

  contacts = mergeDuplicates(contacts);
  contacts = reduceContacts(contacts, tangent, normal);
  return {
    normal,
    tangent,
    refIsA,
    refEdgeIndex: selection.edgeIndex,
    incidentEdgeIndex: incident.index,
    nearCoplanar: selection.nearCoplanar,
    selectionTie: selection.tie,
    contacts,
    selection
  };
}

function chooseReference(shapeA, shapeB, n) {
  // n is B's push-out direction (the solver moves B along +n). The reference
  // face is the one most aligned with the contact axis: A's face opposes B's
  // motion, so it aligns with -n, while B's contacting face aligns with +n.
  const candA = bestEdge(shapeA, Vec2.neg(n));
  const candB = bestEdge(shapeB, n);
  const alignA = candA.align;
  const alignB = candB.align;

  const faceA = alignA >= FACE_ALIGN_MIN;
  const faceB = alignB >= FACE_ALIGN_MIN;
  const nearCoplanar = faceA && faceB && Math.abs(alignA - alignB) < 0.05;

  let ref;
  let edgeIndex;
  let tie = null;
  if (faceA && faceB) {
    // Prefer the longer edge (more stable manifold); deterministically break
    // ties by body id so near-coplanar edge-edge cases cannot flip per frame.
    const lenA = shapeA.edges()[candA.index].tangent.len();
    const lenB = shapeB.edges()[candB.index].tangent.len();
    if (nearCoplanar || Math.abs(lenA - lenB) <= MERGE_TOL) {
      tie = refIsAById(shapeA, shapeB);
    }
    ref = tie ? (tie === 'A' ? shapeA : shapeB) : lenA >= lenB ? shapeA : shapeB;
  } else if (faceA) {
    ref = shapeA;
  } else if (faceB) {
    ref = shapeB;
  } else {
    // Vertex-vertex / vertex-edge degeneracy: take the better aligned edge,
    // again with a deterministic id tie-break.
    ref = alignA >= alignB ? shapeA : shapeB;
  }
  edgeIndex = ref === shapeA ? candA.index : candB.index;
  return { ref, inc: ref === shapeA ? shapeB : shapeA, edgeIndex, nearCoplanar, tie, alignA, alignB };
}

function refIsAById(shapeA, shapeB) {
  return idOf(shapeA) <= idOf(shapeB) ? 'A' : 'B';
}

let shapeIdCounter = 0;
function idOf(shape) {
  if (shape.__id === undefined) shape.__id = shapeIdCounter++;
  return shape.__id;
}

function bestEdge(shape, targetNormal) {
  let best = { index: 0, align: -Infinity };
  for (const e of shape.edges()) {
    const align = Vec2.dot(e.normal, targetNormal);
    if (align > best.align) best = { index: e.index, align };
  }
  return best;
}

function chooseIncident(edges, refNormal) {
  let best = null;
  for (const e of edges) {
    // The incident face is the body-B edge that leads the push-out: its outward
    // normal is most aligned with the contact normal n.
    const score = Vec2.dot(e.normal, refNormal);
    if (!best || score > best.score) best = { ...e, score };
  }
  return best;
}

function clipSegmentToSide(a, b, sidePoint, side) {
  const da = Vec2.dot(Vec2.sub(a, sidePoint), side);
  const db = Vec2.dot(Vec2.sub(b, sidePoint), side);
  const insideA = da >= 0;
  const insideB = db >= 0;
  if (insideA && insideB) return { a: a.clone(), b: b.clone() };
  const t = da / (da - db);
  const crossing = Vec2.lerp(a, b, t);
  if (insideA) return { a: a.clone(), b: crossing };
  if (insideB) return { a: crossing, b: b.clone() };
  // Entirely outside: clamp to the nearer endpoint, caller filters by depth.
  return da > db ? { a: a.clone(), b: a.clone() } : { a: b.clone(), b: b.clone() };
}

function makeContact(point, normal, depth, shapeA, shapeB, refIsA, tangent) {
  const anchorA = shapeA.xf.worldToLocal(point);
  const anchorB = shapeB.xf.worldToLocal(point);
  const s = Vec2.dot(point, tangent);
  return {
    key: `${shapeA.__id}:${shapeB.__id}:${Math.round(s * 1e4)}`,
    point: point.clone(),
    anchorA: anchorA.clone(),
    anchorB: anchorB.clone(),
    normal: normal.clone(),
    depth,
    tangentS: s,
    refIsA,
    // Persistence bookkeeping used by the solver's warm start.
    persistentId: null,
    impulses: { normal: 0, tangent: 0 }
  };
}

function mergeDuplicates(contacts) {
  const out = [];
  for (const c of contacts) {
    const dup = out.find((o) => Vec2.dist2(o.point, c.point) <= MERGE_TOL * MERGE_TOL);
    if (!dup) {
      out.push(c);
    } else {
      // Keep the deeper of the two so a shallow numerical duplicate cannot
      // erase a constraint-relevant contact.
      if (c.depth > dup.depth) Object.assign(dup, c);
    }
  }
  return out;
}

// Reduction must preserve the meaningful spatial span: keep the two contacts
// furthest apart along the reference tangent, falling back to the deepest.
function reduceContacts(contacts, tangent) {
  if (contacts.length <= 2) return contacts;
  let span = -1;
  let pair = [contacts[0], contacts[1]];
  for (let i = 0; i < contacts.length; i++) {
    for (let j = i + 1; j < contacts.length; j++) {
      const d = Math.abs(Vec2.dot(Vec2.sub(contacts[j].point, contacts[i].point), tangent));
      if (d > span) {
        span = d;
        pair = [contacts[i], contacts[j]];
      }
    }
  }
  return pair;
}

// Separated (but within slop) manifold: a single witness contact, depth <= 0.
export function buildSeparatedContact(shapeA, shapeB, gjkResult) {
  if (gjkResult.distance > CONTACT_SLOP) return null;
  const n = gjkResult.normal || Vec2.normalize(Vec2.sub(gjkResult.witnessB, gjkResult.witnessA));
  const point = Vec2.lerp(gjkResult.witnessA, gjkResult.witnessB, 0.5);
  const contact = makeContact(point, n, 0, shapeA, shapeB, true, Vec2.perpLeft(n));
  contact.depth = -gjkResult.distance;
  return {
    normal: n,
    tangent: Vec2.perpLeft(n),
    refIsA: true,
    refEdgeIndex: -1,
    incidentEdgeIndex: -1,
    nearCoplanar: false,
    tie: null,
    contacts: [contact]
  };
}
