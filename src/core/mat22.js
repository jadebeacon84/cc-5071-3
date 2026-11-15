import { Vec2 } from './vec2.js';

// Rotation matrix stored as column vectors: [ex | ey]
export class Mat22 {
  constructor(ex = new Vec2(1, 0), ey = new Vec2(0, 1)) {
    this.ex = ex;
    this.ey = ey;
  }

  static fromAngle(angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return new Mat22(new Vec2(c, s), new Vec2(-s, c));
  }

  mulVec(v) {
    return new Vec2(
      this.ex.x * v.x + this.ey.x * v.y,
      this.ex.y * v.x + this.ey.y * v.y
    );
  }

  transposeMulVec(v) {
    return new Vec2(Vec2.dot(v, this.ex), Vec2.dot(v, this.ey));
  }

  transpose() {
    return new Mat22(
      new Vec2(this.ex.x, this.ey.x),
      new Vec2(this.ex.y, this.ey.y)
    );
  }

  angle() {
    return Math.atan2(this.ex.y, this.ex.x);
  }
}
