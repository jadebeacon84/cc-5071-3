import { test } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { runSceneRecord, verifyRecord } from '../tools/lib/records.js';

test('a real scene run produces a raw record that independently re-verifies', async () => {
  const record = await runSceneRecord('push', 60);
  assert.equal(record.schema, 'convex-contact-record/v1');
  assert.equal(record.framesData.length, 60);
  // Raw measurements are present per frame, never precomputed fixtures.
  const withPairs = record.framesData.filter((f) => f.pairs.length > 0);
  assert.ok(withPairs.length > 0);
  for (const frame of withPairs) {
    for (const p of frame.pairs) {
      assert.ok('gjkDistance' in p && 'separating' in p && 'manifold' in p);
      assert.equal(typeof p.manifoldValid, 'boolean');
    }
  }
  const summary = verifyRecord(record);
  assert.equal(summary.ok, true, JSON.stringify(summary.manifoldFailures));
});

test('tampering with a recorded distance is caught by independent replay', async () => {
  const record = await runSceneRecord('push', 30);
  const target = record.framesData
    .flatMap((f) => f.pairs.map((p) => ({ f, p })))
    .find((x) => Number.isFinite(x.p.gjkDistance));
  assert.ok(target, 'fixture records a GJK distance for every pair');
  const original = target.p.gjkDistance;
  target.p.gjkDistance = original + 1.0; // forge the raw distance
  const summary = verifyRecord(record);
  assert.equal(summary.ok, false);
  assert.ok(summary.distanceMismatches.length > 0);
});

test('forging a manifold validity flag is rejected against live geometry', async () => {
  const record = await runSceneRecord('stack', 30);
  const target = record.framesData
    .flatMap((f) => f.pairs)
    .find((p) => p.manifoldValid === true);
  assert.ok(target);
  target.manifoldValid = false;
  const summary = verifyRecord(record);
  assert.equal(summary.ok, false);
  assert.ok(summary.manifoldFailures.some((m) => m.reason?.includes('recomputed')));
});

test('tampering with the separating vector is detected on replay', async () => {
  const record = await runSceneRecord('push', 30);
  const target = record.framesData.flatMap((f) => f.pairs).find((p) => p.separating);
  target.separating.vector.x += 5;
  const summary = verifyRecord(record);
  assert.equal(summary.ok, false);
  assert.ok(summary.distanceMismatches.some((m) => m.reason?.includes('separating vector')));
});

test('every degenerate probe reports an explicit construction status', async () => {
  const { probeDegenerate } = await import('../tools/lib/records.js');
  const probes = probeDegenerate();
  const byName = Object.fromEntries(probes.map((p) => [p.name, p]));
  assert.equal(byName['point-cloud'].constructed.built, false);
  assert.equal(byName['point-cloud'].constructed.status, 'point');
  assert.equal(byName['collinear-line'].constructed.status, 'line');
  assert.equal(byName['non-finite'].constructed.status, 'non-finite');
  assert.equal(byName['noisy-pentagon-recovers'].constructed.built, true);
});

test('records can be persisted and read back through JSON without drift', async () => {
  const dir = await mkdtemp(join(tmpdir(), 'cck-record-'));
  try {
    const { writeJson, readJson } = await import('../tools/lib/records.js');
    const record = await runSceneRecord('degenerate', 24);
    const path = join(dir, 'scene.record.json');
    await writeJson(path, record);
    const reread = await readJson(path);
    assert.equal(reread.scene, 'degenerate');
    assert.equal(verifyRecord(reread).ok, true);
  } finally {
    await rm(dir, { recursive: true, force: true });
  }
});
