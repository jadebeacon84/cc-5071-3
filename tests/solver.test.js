import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Vec2 } from '../src/core/vec2.js';
import { RESIDUAL_SLOP } from '../src/core/geom.js';
import { BoxShape, WedgeShape, RotatedConvex } from '../src/geometry/shapes.js';
import { Body } from '../src/physics/body.js';
import { World } from '../src/physics/world.js';
import { Solver } from '../src/physics/solver.js';

function settle(world, solver, frames, dt = 1 / 60) {
  for (let i = 0; i < frames; i++) {
    world.step(dt);
    solver.step(dt);
  }
}

function maxPenetration(world) {
  let max = 0;
  for (const pair of world.pairs) {
    for (const c of pair.manifold.contacts) max = Math.max(max, c.depth);
  }
  return max;
}

test('overlapping dynamic boxes settle without residual penetration', () => {
  const world = new World();
  const solver = new Solver(world, { gravity: new Vec2(0, 0) });
  const a = new Body(new BoxShape(1, 1), { static: true, label: 'static' });
  a.setPosition(new Vec2(0, 0));
  const b = new Body(new BoxShape(1, 1), { label: 'dynamic' });
  b.setPosition(new Vec2(1.4, 0));
  world.addBody(a);
  world.addBody(b);

  settle(world, solver, 60);
  assert.ok(maxPenetration(world) <= 0.02, `residual ${maxPenetration(world)}`);
  assert.equal(world.auditIssues().length, 0);
});

test('a box resting on the floor stays supported and never sinks', () => {
  const world = new World();
  const solver = new Solver(world); // default gravity
  const floor = new Body(new BoxShape(8, 0.5), { static: true, label: 'floor' });
  floor.setPosition(new Vec2(0, -3));
  const box = new Body(new BoxShape(0.5, 0.5), { label: 'box' });
  box.setPosition(new Vec2(0, -2.1));
  world.addBody(floor);
  world.addBody(box);

  settle(world, solver, 120);
  // Floor top is at y = -2.5; box half-height 0.5 means resting centre ~ -2.0.
  assert.ok(box.xf.p.y > -2.2, `box sank to ${box.xf.p.y}`);
  assert.ok(Math.abs(box.velocity.y) < 0.05, `residual vertical velocity ${box.velocity.y}`);
  assert.equal(world.auditIssues().length, 0);
});

test('a vertical stack of three props remains standing after settling', () => {
  const world = new World();
  const solver = new Solver(world);
  const floor = new Body(new BoxShape(8, 0.5), { static: true, label: 'floor' });
  floor.setPosition(new Vec2(0, -3));
  const b1 = new Body(new BoxShape(0.5, 0.5), { label: 'b1' });
  b1.setPosition(new Vec2(0, -2.0));
  const b2 = new Body(new BoxShape(0.5, 0.5), { label: 'b2' });
  b2.setPosition(new Vec2(0.0, -1.0));
  const b3 = new Body(new WedgeShape(0.5, 0.5), { label: 'b3' });
  b3.setPosition(new Vec2(0.0, -0.1));
  world.addBody(floor);
  world.addBody(b1);
  world.addBody(b2);
  world.addBody(b3);

  settle(world, solver, 180);
  // Every body stays near the vertical axis (no catastrophic collapse/tunneling).
  for (const b of [b1, b2, b3]) {
    assert.ok(Math.abs(b.xf.p.x) < 0.4, `${b.label} drifted to x=${b.xf.p.x}`);
  }
  assert.ok(maxPenetration(world) <= 0.02);
  assert.equal(world.auditIssues().length, 0);
});

test('impulse trace corresponds to the body motion that resolved penetration', () => {
  const world = new World({ recordTrail: true });
  const solver = new Solver(world, { gravity: new Vec2(0, 0) });
  const a = new Body(new BoxShape(1, 1), { static: true, label: 'wall' });
  a.setPosition(new Vec2(0, 0));
  const b = new Body(new BoxShape(1, 1), { label: 'block' });
  b.setPosition(new Vec2(1.4, 0));
  const start = b.xf.p.clone();
  world.addBody(a);
  world.addBody(b);

  world.step(1 / 60);
  const result = solver.step(1 / 60);
  assert.ok(result.contacts.length >= 1);
  // The recorded normal impulse on B must point along the push-out normal.
  for (const t of solver.traces) {
    assert.ok(t.normalImpulse > 0 || t.correction.len2() > 0);
    assert.ok(Math.abs(t.normal.len() - 1) < 1e-9);
    assert.ok(['boundary-reached', 'support-plane-enumeration', 'gjk-separated'].includes(t.stopReason));
  }
  // B's actual displacement must be along the push-out normal (positive x here).
  settle(world, solver, 40);
  const moved = Vec2.sub(b.xf.p, start);
  assert.ok(Vec2.dot(moved, new Vec2(1, 0)) > 0.1, `B did not move out along +x: ${moved.x}`);
  // And the geometry confirms non-penetration within the position-correction
  // residual slop (the solver intentionally leaves a sub-slop resting gap).
  const residual = Math.max(0, ...world.pairs.flatMap((p) =>
    p.manifold.contacts.map((c) => c.depth)));
  assert.ok(residual <= RESIDUAL_SLOP, `residual penetration ${residual} exceeds slop`);
});

test('broadphase AABB overlap alone never creates a contact without narrow phase', () => {
  const world = new World();
  const solver = new Solver(world, { gravity: new Vec2(0, 0) });
  // A diamond has a wide axis-aligned bounding box but a hull cut back from its
  // corners. Placing a small box toward that corner makes the AABBs overlap on
  // both axes while the convex hulls remain clearly separated.
  const diamond = new RotatedConvex(
    [new Vec2(1.2, 0), new Vec2(0, 1.2), new Vec2(-1.2, 0), new Vec2(0, -1.2)],
    'diamond'
  );
  const a = new Body(diamond, { static: true, label: 'diamond' });
  a.setPosition(new Vec2(0, 0));
  const b = new Body(new BoxShape(0.6, 0.6), { label: 'b' });
  b.setPosition(new Vec2(1.4, 1.4));
  world.addBody(a);
  world.addBody(b);

  const broad = world.broadPhase();
  assert.ok(broad.length === 1, 'broadphase flags the AABB overlap');
  world.step(1 / 60);
  solver.step(1 / 60);
  // The shapes are geometrically separated, so the narrow phase must not invent
  // a penetrating contact merely because the inflated AABBs overlap.
  assert.equal(world.pairs.length, 0, 'no pair without real GJK/EPA overlap');
});

test('warm start keeps a resting stack free of audit violations across frames', () => {
  const world = new World({ audit: true });
  const solver = new Solver(world);
  const floor = new Body(new BoxShape(8, 0.5), { static: true, label: 'floor' });
  floor.setPosition(new Vec2(0, -3));
  const b1 = new Body(new BoxShape(0.5, 0.5), { label: 'b1' });
  b1.setPosition(new Vec2(0, -2.0));
  const b2 = new Body(new BoxShape(0.5, 0.5), { label: 'b2' });
  b2.setPosition(new Vec2(0.02, -1.0));
  world.addBody(floor);
  world.addBody(b1);
  world.addBody(b2);

  settle(world, solver, 150);
  const allIssues = world.auditIssues();
  assert.equal(allIssues.length, 0);
});
