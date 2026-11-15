import { Vec2 } from '../core/vec2.js';
import { RESIDUAL_SLOP } from '../core/geom.js';
import { satReference } from '../geometry/sat.js';
import { detectContact } from '../geometry/detect.js';

// World owns broadphase, narrow phase, contact persistence and audit records.
// Broadphase uses AABB overlap only as a cheap filter: no contact is ever
// generated from AABB overlap alone, every pair goes through GJK/EPA.
export class World {
  constructor(options = {}) {
    this.bodies = [];
    this.pairs = [];
    this.frame = 0;
    this.auditLog = [];
    this.options = {
      recordTrail: options.recordTrail ?? false,
      audit: options.audit ?? true,
      normalTolerance: options.normalTolerance ?? 0.1,
      depthTolerance: options.depthTolerance ?? 0.05,
      maxResidual: RESIDUAL_SLOP
    };
  }

  addBody(body) {
    this.bodies.push(body);
    return body;
  }

  aabbOverlap(a, b) {
    const ba = a.shape.aabb();
    const bb = b.shape.aabb();
    return ba.maxX >= bb.minX && ba.minX <= bb.maxX && ba.maxY >= bb.minY && ba.minY <= bb.maxY;
  }

  findPair(bodyA, bodyB) {
    return this.pairs.find(
      (p) =>
        (p.bodyA === bodyA && p.bodyB === bodyB) ||
        (p.bodyA === bodyB && p.bodyB === bodyA)
    );
  }

  step(dt) {
    this.frame++;
    for (const body of this.bodies) body.syncShape();

    const broadPairs = this.broadPhase();
    this.narrowPhase(broadPairs);
    return this.pairs;
  }

  broadPhase() {
    const out = [];
    for (let i = 0; i < this.bodies.length; i++) {
      for (let j = i + 1; j < this.bodies.length; j++) {
        const a = this.bodies[i];
        const b = this.bodies[j];
        if (a.static && b.static) continue;
        // Inflated AABBs keep resting (near-touching) pairs alive.
        if (this.aabbOverlap(a, b)) out.push([a, b]);
      }
    }
    return out;
  }

  narrowPhase(broadPairs) {
    const active = new Map();
    for (const [bodyA, bodyB] of broadPairs) {
      const result = this.collide(bodyA, bodyB);
      if (!result) continue;
      active.set(`${bodyA.id}:${bodyB.id}`, result);
    }
    // Retire pairs whose broadphase culling dropped them.
    this.pairs = this.pairs.filter((p) => active.has(`${p.bodyA.id}:${p.bodyB.id}`));
    for (const [, pair] of active) {
      const existing = this.findPair(pair.bodyA, pair.bodyB);
      if (existing) {
        this.persist(existing, pair);
      } else {
        this.assignContactIds(pair);
        this.pairs.push(pair);
      }
    }
  }

  collide(bodyA, bodyB) {
    const detected = detectContact(bodyA.shape, bodyB.shape, this.options.recordTrail);
    if (!detected) return null;
    const { gjk: g, epa: penetration, manifold, overlap } = detected;
    let sat = detected.sat;

    if (this.options.audit) {
      if (!sat) sat = satReference(bodyA.shape, bodyB.shape);
      const audit = this.crossCheck(g, penetration, sat, manifold);
      this.auditLog.push({ frame: this.frame, bodyA: bodyA.id, bodyB: bodyB.id, ...audit });
    }

    return {
      bodyA,
      bodyB,
      gjk: g,
      epa: penetration,
      sat,
      manifold,
      // Solver-side arrays, repopulated each frame from the manifold.
      solverContacts: []
    };
  }

  crossCheck(g, penetration, sat, manifold) {
    const issues = [];
    if (penetration) {
      if (!sat.intersect) issues.push('SAT says separated while GJK/EPA says overlap');
      const depthErr = Math.abs(penetration.depth - sat.depth);
      if (depthErr > this.options.depthTolerance) {
        issues.push(`depth mismatch epa=${penetration.depth.toFixed(4)} sat=${sat.depth.toFixed(4)}`);
      }
      if (sat.normal) {
        const sign = Vec2.dot(penetration.normal, sat.normal);
        if (sign < 1 - this.options.normalTolerance) {
          issues.push(`normal direction mismatch epa=(${penetration.normal.x.toFixed(2)},${penetration.normal.y.toFixed(2)}) sat=(${sat.normal.x.toFixed(2)},${sat.normal.y.toFixed(2)}) dot=${sign.toFixed(3)}`);
        }
      }
      for (const c of manifold.contacts) {
        if (c.depth < -1e-6) issues.push('contact reports negative depth after clipping');
      }
    } else if (sat && sat.intersect) {
      issues.push('SAT says overlap while GJK says separated');
    }
    return {
      issues,
      epaDepth: penetration?.depth ?? 0,
      satDepth: sat.depth,
      normalDot: penetration && sat.normal ? Vec2.dot(penetration.normal, sat.normal) : null
    };
  }

  assignContactIds(pair) {
    for (const c of pair.manifold.contacts) {
      c.persistentId = `${pair.bodyA.id}-${pair.bodyB.id}-${Math.round(c.point.x * 1e3)}-${Math.round(c.point.y * 1e3)}`;
    }
  }

  // Carry impulse history to the closest surviving contact when the manifold
  // gets rebuilt, so resting stacks warm start instead of re-jittering.
  persist(oldPair, newPair) {
    for (const c of newPair.manifold.contacts) {
      const nearest = oldPair.manifold.contacts
        .map((o) => ({ o, d2: Vec2.dist2(o.point, c.point) }))
        .filter((x) => x.d2 <= 0.02 * 0.02)
        .sort((x, y) => x.d2 - y.d2)[0];
      if (nearest) {
        c.persistentId = nearest.o.persistentId;
        c.impulses = { ...nearest.o.impulses };
      } else {
        c.persistentId = `${newPair.bodyA.id}-${newPair.bodyB.id}-${Math.round(c.point.x * 1e3)}-${Math.round(c.point.y * 1e3)}`;
      }
    }
    newPair.solverContacts = oldPair.solverContacts;
    const idx = this.pairs.indexOf(oldPair);
    this.pairs[idx] = newPair;
  }

  auditIssues() {
    return this.auditLog.filter((a) => a.issues.length > 0);
  }
}
