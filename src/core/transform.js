import { Mat22 } from './mat22.js';
import { Vec2 } from './vec2.js';

// Rigid transform: world = q * local + p
export class Transform {
  constructor(position = new Vec2(), angle = 0) {
    this.p = position.clone();
    this.q = Mat22.fromAngle(angle);
  }

  setAngle(angle) {
    this.q = Mat22.fromAngle(angle);
  }

  get angle() {
    return this.q.angle();
  }

  localToWorld(v) {
    return Vec2.add(this.q.mulVec(v), this.p);
  }

  worldToLocal(v) {
    return this.q.transposeMulVec(Vec2.sub(v, this.p));
  }

  directionToWorld(v) {
    return this.q.mulVec(v);
  }

  directionToLocal(v) {
    return this.q.transposeMulVec(v);
  }

  clone() {
    const t = new Transform(this.p, 0);
    t.q = new Mat22(this.exClone(), this.eyClone());
    return t;
  }

  exClone() {
    return this.q.ex.clone();
  }

  eyClone() {
    return this.q.ey.clone();
  }
}
