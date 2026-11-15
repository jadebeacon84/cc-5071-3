import { Vec2 } from '../core/vec2.js';
import { RESIDUAL_SLOP } from '../core/geom.js';

// Sequential impulse velocity solver plus split-impulse position correction.
// Every impulse is recorded against a persistent contact id, and a post-solve
// audit verifies the bodies actually separate: the engine never silently
// absorbs a wrong normal direction.
export class Solver {
  constructor(world, options = {}) {
    this.world = world;
    this.velocityIterations = options.velocityIterations ?? 12;
    this.positionIterations = options.positionIterations ?? 6;
    this.baumgarte = options.baumgarte ?? 0.2;
    this.maxCorrection = options.maxCorrection ?? 0.2;
    this.gravity = options.gravity ?? new Vec2(0, -9.81);
    this.traces = [];
  }

  step(dt) {
    this.traces = [];
    for (const body of this.world.bodies) {
      if (body.static) continue;
      body.velocity = Vec2.add(body.velocity, Vec2.mul(this.gravity, dt));
    }

    const contacts = [];
    for (const pair of this.world.pairs) {
      for (const c of pair.manifold.contacts) {
        const prepared = this.prepare(pair, c, dt);
        if (prepared) {
          contacts.push(prepared);
          pair.solverContacts.push(prepared);
        }
      }
    }

    for (const c of contacts) this.warmStart(c);
    for (let i = 0; i < this.velocityIterations; i++) {
      for (const c of contacts) this.solveNormal(c);
      for (const c of contacts) this.solveFriction(c);
    }
    this.integrate(dt);

    const corrections = [];
    for (let i = 0; i < this.positionIterations; i++) {
      corrections.length = 0;
      let any = false;
      for (const c0 of contacts) {
        const c = this.preparePosition(c0);
        if (c.slop > RESIDUAL_SLOP) {
          this.applyCorrection(c);
          corrections.push(c);
          any = true;
        }
      }
      if (!any) break;
    }

    const audit = this.postAudit(contacts);
    for (const c of contacts) this.recordTrace(c, audit);
    return { contacts, corrections: corrections.length, audit };
  }

  prepare(pair, contact, dt) {
    const A = pair.bodyA;
    const B = pair.bodyB;
    const n = contact.normal;
    const t = pair.manifold.tangent;
    const rA = Vec2.sub(contact.point, A.xf.p);
    const rB = Vec2.sub(contact.point, B.xf.p);

    const kNormal =
      A.invMass +
      B.invMass +
      A.invInertia * Vec2.cross(rA, n) * Vec2.cross(rA, n) +
      B.invInertia * Vec2.cross(rB, n) * Vec2.cross(rB, n);
    const kTangent =
      A.invMass +
      B.invMass +
      A.invInertia * Vec2.cross(rA, t) * Vec2.cross(rA, t) +
      B.invInertia * Vec2.cross(rB, t) * Vec2.cross(rB, t);

    return {
      pair,
      contact,
      A,
      B,
      n,
      t,
      rA,
      rB,
      massNormal: kNormal > 1e-12 ? 1 / kNormal : 0,
      massTangent: kTangent > 1e-12 ? 1 / kTangent : 0,
      velocityBias: 0,
      accumulatedNormal: contact.impulses.normal,
      accumulatedTangent: contact.impulses.tangent,
      appliedNormal: 0,
      appliedTangent: 0,
      correction: new Vec2(),
      correctionA: new Vec2(),
      correctionB: new Vec2(),
      slop: contact.depth
    };
  }

  relativeVelocity(c) {
    const vA = Vec2.add(c.A.velocity, Vec2.mul(Vec2.perpLeft(c.rA), c.A.angularVelocity));
    const vB = Vec2.add(c.B.velocity, Vec2.mul(Vec2.perpLeft(c.rB), c.B.angularVelocity));
    return Vec2.sub(vB, vA);
  }

  warmStart(c) {
    const impulse = Vec2.add(Vec2.mul(c.n, c.accumulatedNormal), Vec2.mul(c.t, c.accumulatedTangent));
    c.A.applyImpulse(Vec2.neg(impulse), c.contact.point);
    c.B.applyImpulse(impulse, c.contact.point);
  }

  solveNormal(c) {
    const vn = Vec2.dot(this.relativeVelocity(c), c.n);
    let lambda = -vn * c.massNormal;
    const prev = c.accumulatedNormal;
    c.accumulatedNormal = Math.max(0, prev + lambda);
    lambda = c.accumulatedNormal - prev;
    c.appliedNormal += lambda;
    const impulse = Vec2.mul(c.n, lambda);
    c.A.applyImpulse(Vec2.neg(impulse), c.contact.point);
    c.B.applyImpulse(impulse, c.contact.point);
  }

  solveFriction(c) {
    const vt = Vec2.dot(this.relativeVelocity(c), c.t);
    let lambda = -vt * c.massTangent;
    const mu = Math.sqrt(c.A.friction * c.B.friction);
    const maxFriction = mu * c.accumulatedNormal;
    const prev = c.accumulatedTangent;
    c.accumulatedTangent = Math.max(-maxFriction, Math.min(maxFriction, prev + lambda));
    lambda = c.accumulatedTangent - prev;
    c.appliedTangent += lambda;
    const impulse = Vec2.mul(c.t, lambda);
    c.A.applyImpulse(Vec2.neg(impulse), c.contact.point);
    c.B.applyImpulse(impulse, c.contact.point);
  }

  integrate(dt) {
    for (const body of this.world.bodies) {
      if (body.static) continue;
      body.xf.p = Vec2.add(body.xf.p, Vec2.mul(body.velocity, dt));
      body.xf.setAngle(body.xf.angle + body.angularVelocity * dt);
      body.syncShape();
    }
  }

  // Recompute penetration from the moved geometry instead of trusting the
  // pre-solve depth. Each body's anchor is its own local surface point, so the
  // two world anchors stay distinct and measure real penetration. With n the
  // push-out direction (B moves along +n to separate), penetration is the
  // amount by which B's anchor lies past A's anchor along -n.
  preparePosition(c) {
    const wA = c.A.xf.localToWorld(c.contact.anchorA);
    const wB = c.B.xf.localToWorld(c.contact.anchorB);
    // Positive while overlapping; zero once B has been pushed out along n.
    c.slop = Math.max(0, c.contact.depth - Vec2.dot(Vec2.sub(wB, wA), c.n));
    c.worldA = wA;
    c.worldB = wB;
    return c;
  }

  applyCorrection(c) {
    const correction = Math.min(this.maxCorrection, this.baumgarte * c.slop);
    const lambda = correction * c.massNormal;
    // Push B along +n and A along -n (n is B's push-out direction).
    const impulse = Vec2.mul(c.n, lambda);
    if (!c.A.static) {
      c.A.xf.p = Vec2.sub(c.A.xf.p, Vec2.mul(impulse, c.A.invMass));
      c.A.xf.setAngle(
        c.A.xf.angle - c.A.invInertia * Vec2.cross(c.rA, impulse)
      );
      c.A.syncShape();
    }
    if (!c.B.static) {
      c.B.xf.p = Vec2.add(c.B.xf.p, Vec2.mul(impulse, c.B.invMass));
      c.B.xf.setAngle(
        c.B.xf.angle + c.B.invInertia * Vec2.cross(c.rB, impulse)
      );
      c.B.syncShape();
    }
    c.correction = impulse;
  }

  postAudit(contacts) {
    const violations = [];
    for (const c of contacts) {
      const wA = c.A.xf.localToWorld(c.contact.anchorA);
      const wB = c.B.xf.localToWorld(c.contact.anchorB);
      // Remaining penetration along the push-out normal.
      const residual = Math.max(0, c.contact.depth - Vec2.dot(Vec2.sub(wB, wA), c.n));
      c.residual = residual;
      if (residual > RESIDUAL_SLOP) {
        violations.push({
          id: c.contact.persistentId,
          residual,
          normal: c.n,
          pair: `${c.A.label}/${c.B.label}`
        });
      }
    }
    return { violations, count: contacts.length };
  }

  recordTrace(c, audit) {
    this.traces.push({
      id: c.contact.persistentId,
      pair: `${c.A.label}/${c.B.label}`,
      point: c.contact.point.clone(),
      normal: c.n.clone(),
      depthBefore: c.contact.depth,
      normalImpulse: c.accumulatedNormal,
      tangentImpulse: c.accumulatedTangent,
      correction: c.correction.clone(),
      residual: c.residual ?? 0,
      stopReason: c.pair.epa?.stopReason ?? 'gjk-separated'
    });
    c.contact.impulses.normal = c.accumulatedNormal;
    c.contact.impulses.tangent = c.accumulatedTangent;
  }
}
