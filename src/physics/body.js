import { Transform } from '../core/transform.js';
import { Vec2 } from '../core/vec2.js';

export class Body {
  constructor(shape, options = {}) {
    this.shape = shape;
    this.id = options.id ?? Body.nextId++;
    this.label = options.label ?? `body-${this.id}`;
    this.static = options.static ?? false;
    this.density = options.density ?? 1;
    this.friction = options.friction ?? 0.4;
    this.restitution = options.restitution ?? 0;

    this.xf = new Transform(options.position ?? new Vec2(), options.angle ?? 0);
    this.shape.updateTransform(this.xf);

    this.velocity = new Vec2();
    this.angularVelocity = 0;
    this.force = new Vec2();
    this.torque = 0;

    this.mass = this.static ? 0 : this.shape.area() * this.density;
    this.invMass = this.static ? 0 : 1 / this.mass;
    const inertia = this.shape.inertiaLocal() * this.mass;
    this.inertia = this.static ? 0 : inertia;
    this.invInertia = this.static ? 0 : 1 / inertia;

    // Solver scratch (split impulses for position correction).
    this.pseudoVelocity = new Vec2();
    this.pseudoOmega = 0;
  }

  static nextId = 0;

  setPosition(p) {
    this.xf.p = p.clone();
    this.shape.updateTransform(this.xf);
  }

  setAngle(angle) {
    this.xf.setAngle(angle);
    this.shape.updateTransform(this.xf);
  }

  syncShape() {
    this.shape.updateTransform(this.xf);
  }

  applyImpulse(j, point) {
    if (this.static) return;
    this.velocity = Vec2.add(this.velocity, Vec2.mul(j, this.invMass));
    const r = Vec2.sub(point, this.xf.p);
    this.angularVelocity += this.invInertia * Vec2.cross(r, j);
  }
}
