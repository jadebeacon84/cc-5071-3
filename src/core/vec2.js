export class Vec2 {
  constructor(x = 0, y = 0) {
    this.x = x;
    this.y = y;
  }

  clone() {
    return new Vec2(this.x, this.y);
  }

  set(x, y) {
    this.x = x;
    this.y = y;
    return this;
  }

  static add(a, b) {
    return new Vec2(a.x + b.x, a.y + b.y);
  }

  static sub(a, b) {
    return new Vec2(a.x - b.x, a.y - b.y);
  }

  static mul(a, s) {
    return new Vec2(a.x * s, a.y * s);
  }

  static neg(a) {
    return new Vec2(-a.x, -a.y);
  }

  static dot(a, b) {
    return a.x * b.x + a.y * b.y;
  }

  // z component of the 3D cross product a x b
  static cross(a, b) {
    return a.x * b.y - a.y * b.x;
  }

  // 90 degree counter-clockwise rotation
  static perpLeft(a) {
    return new Vec2(-a.y, a.x);
  }

  // 90 degree clockwise rotation
  static perpRight(a) {
    return new Vec2(a.y, -a.x);
  }

  static lerp(a, b, t) {
    return new Vec2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
  }

  len2() {
    return this.x * this.x + this.y * this.y;
  }

  len() {
    return Math.hypot(this.x, this.y);
  }

  static dist2(a, b) {
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    return dx * dx + dy * dy;
  }

  static dist(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y);
  }

  static normalize(a) {
    const l = Math.hypot(a.x, a.y);
    if (l <= 1e-12) return new Vec2(0, 0);
    return new Vec2(a.x / l, a.y / l);
  }

  static approx(a, b, tol = 1e-9) {
    return Vec2.dist2(a, b) <= tol * tol;
  }
}
