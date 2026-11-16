import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Vec2 } from '../src/core/vec2.js';
import { RotatedConvex, buildHull } from '../src/geometry/shapes.js';
import { validateConvex, classifyRawPoints } from '../src/geometry/validate.js';

test('point cloud is explicitly classified as point and cannot be built', () => {
  const pts = [new Vec2(2, 2), new Vec2(2, 2), new Vec2(2, 2)];
  const r = validateConvex(pts);
  assert.equal(r.ok, false);
  assert.equal(r.status, 'point');
  assert.throws(() => new RotatedConvex(pts), (err) => err.status === 'point');
});

test('collinear points are explicitly classified as line and cannot be built', () => {
  const pts = [new Vec2(0, 0), new Vec2(1, 0), new Vec2(2, 0), new Vec2(3, 0)];
  const r = validateConvex(pts);
  assert.equal(r.ok, false);
  assert.equal(r.status, 'line');
  assert.throws(() => new RotatedConvex(pts), (err) => err.status === 'line');
});

test('fewer than three distinct points is an explicit insufficient status', () => {
  const r = classifyRawPoints([new Vec2(0, 0), new Vec2(1, 1)]);
  assert.equal(r.ok, false);
  assert.equal(r.status, 'insufficient');
  assert.equal(r.hullBlocked, true);
});

test('a non-finite coordinate is rejected rather than silently dropped', () => {
  const pts = [new Vec2(0, 0), new Vec2(1, 0), new Vec2(NaN, 1)];
  const r = validateConvex(pts);
  assert.equal(r.status, 'non-finite');
  assert.throws(() => buildHull(pts), (err) => err.status === 'non-finite');
  assert.throws(() => new RotatedConvex(pts), (err) => err.status === 'non-finite');
});

test('a valid convex polygon reports positive area and strict convexity', () => {
  const r = validateConvex([
    new Vec2(1, 0), new Vec2(0, 1), new Vec2(-1, 0), new Vec2(0, -1)
  ]);
  assert.equal(r.ok, true);
  assert.equal(r.status, 'valid');
  assert.ok(r.area > 0);
});

test('a concave polygon is flagged and not reported as valid convex', () => {
  const r = validateConvex([
    new Vec2(0, 0), new Vec2(2, 0), new Vec2(1, 0.5), new Vec2(2, 1), new Vec2(0, 1)
  ]);
  assert.equal(r.ok, false);
  assert.equal(r.status, 'concave');
});

test('noisy but recoverable input still constructs a hull with an explicit diagnosis', () => {
  const shape = new RotatedConvex([
    new Vec2(0, 0.5), new Vec2(0, 0.5),
    new Vec2(0.48, 0.16), new Vec2(0.3, -0.4),
    new Vec2(0.15, -0.4), new Vec2(-0.3, -0.4), new Vec2(-0.48, 0.16)
  ], 'noisy');
  assert.equal(shape.worldVerts.length, 5);
  assert.ok(shape.diagnosis);
  assert.equal(shape.diagnosis.vertexCount, 7);
});
