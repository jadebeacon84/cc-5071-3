import { Vec2 } from './vec2.js';

// Geometry tolerances. World units are metres; typical prop size 0.5..2 m.
export const EPS = 1e-9;
export const EPS_LEN = 1e-7;
export const EPS_LEN2 = EPS_LEN * EPS_LEN;
// A support vertex that does not extend the simplex by more than this stalls GJK/EPA
export const GJK_STALL_TOL = 1e-8;
export const EPA_STALL_TOL = 1e-6;
// A hull face whose origin distance is at or below this is treated as a seed
// artifact (origin resting on a seed edge) rather than a real boundary face.
export const EPA_BOUNDARY_EPS = 1e-7;
// Contacts are generated while the gap is no larger than this (resting touch)
export const CONTACT_SLOP = 0.01;
// Incident points slightly on the separated side are still kept on the manifold
export const KEEP_SLOP = 0.005;
// Two manifold points closer than this collapse into one
export const MERGE_TOL = 0.01;
// Residual penetration accepted after the position solve
export const RESIDUAL_SLOP = 0.01;
// Minimum |dot(normal, edgeNormal)| for an edge to count as a face (cos ~41 deg)
export const FACE_ALIGN_MIN = 0.75;

export function clamp(x, lo, hi) {
  return x < lo ? lo : x > hi ? hi : x;
}

// Closest point on segment a..b to point p. Returns parameter t and point.
export function closestOnSegment(a, b, p) {
  const ab = Vec2.sub(b, a);
  const len2 = ab.len2();
  if (len2 <= EPS_LEN2) {
    return { t: 0, point: a.clone() };
  }
  const t = clamp(Vec2.dot(Vec2.sub(p, a), ab) / len2, 0, 1);
  return { t, point: Vec2.lerp(a, b, t) };
}

// Signed twice-area of triangle (a, b, c); positive when CCW.
export function signedArea2(a, b, c) {
  return Vec2.cross(Vec2.sub(b, a), Vec2.sub(c, a));
}

export function polygonArea2(verts) {
  let sum = 0;
  for (let i = 0; i < verts.length; i++) {
    const a = verts[i];
    const b = verts[(i + 1) % verts.length];
    sum += Vec2.cross(a, b);
  }
  return sum;
}
