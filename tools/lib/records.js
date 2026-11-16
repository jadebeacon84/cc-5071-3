import { writeFile, mkdir, readFile } from 'node:fs/promises';
import { dirname } from 'node:path';
import { Vec2 } from '../../src/core/vec2.js';
import { buildScene, SCENE_NAMES } from '../../src/geometry/scenes.js';
import { verifyManifold, separatingVector } from '../../src/geometry/verify.js';
import { classifyRawPoints } from '../../src/geometry/validate.js';
import { RotatedConvex } from '../../src/geometry/shapes.js';

const FIXED_DT = 1 / 60;

// Drive a scene the same way the graphical loop does, but headlessly. Every
// frame's measurements are raw geometry outputs captured at the moment they
// were produced; verify only reads them back and recomputes independently.
export async function runSceneRecord(sceneName, frames) {
  const scene = buildScene(sceneName);
  const record = {
    schema: 'convex-contact-record/v1',
    scene: sceneName,
    frames,
    dt: FIXED_DT,
    node: process.version,
    generatedAt: new Date().toISOString(),
    framesData: []
  };

  for (let f = 0; f < frames; f++) {
    // Narrow phase first; the manifold is a pre-solve geometry snapshot and is
    // verified against that same geometry, before the solver moves the bodies.
    scene.world.step(FIXED_DT);
    const frameRec = {
      frame: scene.world.frame,
      bodies: scene.world.bodies.map((b) => ({
        label: b.label,
        static: b.static,
        position: { x: b.xf.p.x, y: b.xf.p.y },
        angle: b.xf.angle
      })),
      pairs: []
    };
    for (const pair of scene.world.pairs) {
      const g = pair.gjk;
      const sep = separatingVector(pair.bodyA.shape, pair.bodyB.shape, g);
      let verdict = null;
      try {
        verdict = verifyManifold(pair.bodyA.shape, pair.bodyB.shape, pair.manifold);
      } catch (err) {
        verdict = { ok: false, failedChecks: [`verifier-threw:${err.message}`] };
      }
      frameRec.pairs.push({
        bodyA: pair.bodyA.label,
        bodyB: pair.bodyB.label,
        overlap: pair.manifold ? pair.epa != null : false,
        manifoldValid: verdict.ok,
        manifoldFailedChecks: verdict.failedChecks,
        separating: {
          state: sep.state,
          magnitude: sep.magnitude,
          vector: { x: sep.vector.x, y: sep.vector.y },
          normal: { x: sep.normal.x, y: sep.normal.y }
        },
        gjkDistance: g.distance,
        gjkStopReason: g.stopReason,
        epa: pair.epa
          ? {
              depth: pair.epa.depth,
              method: pair.epa.method,
              stopReason: pair.epa.stopReason,
              normal: { x: pair.epa.normal.x, y: pair.epa.normal.y }
            }
          : null,
        sat: pair.sat
          ? {
              intersect: pair.sat.intersect,
              depth: pair.sat.depth,
              distance: pair.sat.distance
            }
          : null,
        manifold: serializeManifold(pair.manifold)
      });
    }
    frameRec.auditIssues = scene.world.auditLog
      .filter((a) => a.frame === scene.world.frame && a.issues.length > 0)
      .map((a) => ({ bodyA: a.bodyA, bodyB: a.bodyB, issues: a.issues }));
    record.framesData.push(frameRec);

    // Only after the snapshot is captured does the solver advance the bodies.
    const solverResult = scene.solver.step(FIXED_DT);
    frameRec.solver = {
      residualViolations: solverResult.audit.violations.length,
      traces: scene.solver.traces.map((t) => ({
        pair: t.pair,
        normalImpulse: t.normalImpulse,
        tangentImpulse: t.tangentImpulse,
        residual: t.residual,
        depthBefore: t.depthBefore
      }))
    };
  }
  return record;
}

function serializeManifold(m) {
  return {
    normal: { x: m.normal.x, y: m.normal.y },
    tangent: { x: m.tangent.x, y: m.tangent.y },
    refIsA: m.refIsA,
    contacts: m.contacts.map((c) => ({
      point: { x: c.point.x, y: c.point.y },
      depth: c.depth,
      anchorA: { x: c.anchorA.x, y: c.anchorA.y },
      anchorB: { x: c.anchorB.x, y: c.anchorB.y },
      tangentS: c.tangentS
    }))
  };
}

// Reconstruct shapes for an independent re-verification. Scene body specs are
// replayed from the same builder and advanced with the identical pre-solve /
// post-solve ordering, so the verifier measures live geometry rather than
// trusting stored numbers.
export function verifyRecord(record) {
  const scene = buildScene(record.scene);
  const summary = {
    schema: record.schema,
    scene: record.scene,
    frames: record.frames,
    pairChecks: 0,
    manifoldFailures: [],
    distanceMismatches: [],
    auditFailures: [],
    solverResidualFailures: [],
    residualTrajectory: [],
    totalResidualViolations: 0,
    settlingAuditIssues: 0,
    ok: true
  };
  const settleLimit = Math.max(1, Math.floor(record.frames / 4));

  for (let f = 0; f < record.frames; f++) {
    // Match runSceneRecord: narrow-phase snapshot before the solver moves bodies.
    scene.world.step(FIXED_DT);
    const frameRec = record.framesData[f];
    if (!frameRec) {
      summary.ok = false;
      summary.manifoldFailures.push({ frame: f + 1, reason: 'missing frame in record' });
      scene.solver.step(FIXED_DT);
      continue;
    }

    for (const recPair of frameRec.pairs) {
      const pair = scene.world.pairs.find(
        (p) => p.bodyA.label === recPair.bodyA && p.bodyB.label === recPair.bodyB
      );
      summary.pairChecks++;
      if (!pair) {
        summary.ok = false;
        summary.manifoldFailures.push({ frame: f + 1, pair: `${recPair.bodyA}/${recPair.bodyB}`, reason: 'pair absent in replay' });
        continue;
      }

      // Independent recomputation from the live pre-solve geometry.
      const verdict = verifyManifold(pair.bodyA.shape, pair.bodyB.shape, pair.manifold);
      if (!verdict.ok) {
        summary.ok = false;
        summary.manifoldFailures.push({
          frame: f + 1,
          pair: `${recPair.bodyA}/${recPair.bodyB}`,
          failedChecks: verdict.failedChecks
        });
      }
      // The raw record's own verdict must match the recomputed one: a forged or
      // edited record cannot claim validity the geometry does not support.
      if (recPair.manifoldValid !== verdict.ok) {
        summary.ok = false;
        summary.manifoldFailures.push({
          frame: f + 1,
          pair: `${recPair.bodyA}/${recPair.bodyB}`,
          reason: `record claims manifoldValid=${recPair.manifoldValid}, recomputed=${verdict.ok}`
        });
      }

      // Distance: live GJK must match the raw recorded gap.
      const liveGap = pair.gjk.distance;
      if (Math.abs(liveGap - recPair.gjkDistance) > 1e-9) {
        summary.ok = false;
        summary.distanceMismatches.push({
          frame: f + 1,
          pair: `${recPair.bodyA}/${recPair.bodyB}`,
          recorded: recPair.gjkDistance,
          replayed: liveGap
        });
      }
      // The recorded separating vector must equal the freshly computed one.
      const sep = separatingVector(pair.bodyA.shape, pair.bodyB.shape, pair.gjk);
      const recSep = recPair.separating;
      const vecErr = Math.hypot(sep.vector.x - recSep.vector.x, sep.vector.y - recSep.vector.y);
      if (vecErr > 1e-7 || sep.state !== recSep.state) {
        summary.ok = false;
        summary.distanceMismatches.push({
          frame: f + 1,
          pair: `${recPair.bodyA}/${recPair.bodyB}`,
          reason: 'recorded separating vector disagrees with recomputation'
        });
      }
    }

    if (frameRec.auditIssues.length > 0) {
      // Cross-check tolerances can be exceeded during initial penetration
      // settling; only persistent issues (after the settle window) fail the run.
      if (f + 1 > settleLimit) {
        summary.ok = false;
        summary.auditFailures.push(...frameRec.auditIssues.map((i) => ({ frame: f + 1, ...i })));
      } else {
        summary.settlingAuditIssues += frameRec.auditIssues.length;
      }
    }

    // Solver runs after the snapshot; its residual audit closes the frame.
    const solverResult = scene.solver.step(FIXED_DT);
    const violationCount = solverResult.audit.violations.length;
    const worstResidual = violationCount
      ? Math.max(...solverResult.audit.violations.map((v) => v.residual))
      : 0;
    summary.residualTrajectory.push({ frame: f + 1, violations: violationCount, worst: worstResidual });
    if (violationCount > 0) summary.totalResidualViolations += violationCount;
    if (frameRec.solver?.residualViolations !== violationCount) {
      summary.ok = false;
      summary.solverResidualFailures.push({
        frame: f + 1,
        reason: 'recorded residual count disagrees with replay'
      });
    }
  }

  // Residual policy. Two residual regimes are legitimate:
  //   1. settling - an initially interpenetrating scene can take several
  //      consecutive frames inside the settle window to remove the overlap;
  //   2. impact   - a multi-body collision after the settle window can leave a
  //      short residual streak (up to two frames) that the position solver then
  //      removes.
  // What must never happen is a residual streak of three or more frames after
  // the settle window (the solver stopped converging) or any residual on the
  // final frame.
  const traj = summary.residualTrajectory;
  const stuck = [];
  for (let i = 0; i < traj.length; i++) {
    if (traj[i].frame <= settleLimit || traj[i].violations === 0) continue;
    const a = traj[i];
    const b = traj[i + 1];
    const c = traj[i + 2];
    if (b && c && b.violations > 0 && c.violations > 0) stuck.push(a.frame);
  }
  const finalFrame = traj[traj.length - 1];
  if (stuck.length > 0 || (finalFrame && finalFrame.violations > 0)) {
    summary.ok = false;
    summary.solverResidualFailures.push({
      reason: 'residual streak >= 3 frames after the settle window, or residual on the final frame',
      streakStartFrames: stuck,
      finalViolations: finalFrame?.violations ?? 0
    });
  }
  return summary;
}

// Explicit degenerate-shape probe used by the CLI and release checks.
export function probeDegenerate() {
  const cases = [
    { name: 'point-cloud', expectBuilt: false, points: [new Vec2(1, 1), new Vec2(1, 1), new Vec2(1, 1)] },
    { name: 'collinear-line', expectBuilt: false, points: [new Vec2(0, 0), new Vec2(1, 0), new Vec2(2, 0), new Vec2(3, 0)] },
    { name: 'two-points', expectBuilt: false, points: [new Vec2(0, 0), new Vec2(1, 1)] },
    { name: 'non-finite', expectBuilt: false, points: [new Vec2(0, 0), new Vec2(1, 0), new Vec2(NaN, 1)] },
    {
      name: 'noisy-pentagon-recovers',
      expectBuilt: true,
      points: [
        new Vec2(0, 0.5), new Vec2(0, 0.5),
        new Vec2(0.48, 0.16), new Vec2(0.3, -0.4),
        new Vec2(0.15, -0.4), new Vec2(-0.3, -0.4), new Vec2(-0.48, 0.16)
      ]
    }
  ];
  return cases.map((c) => {
    const classification = classifyRawPoints(c.points);
    let constructed = null;
    try {
      const shape = new RotatedConvex(c.points, c.name);
      constructed = { built: true, vertexCount: shape.worldVerts.length, area: shape.area() };
    } catch (err) {
      constructed = { built: false, status: err.status, error: err.message };
    }
    return { name: c.name, expectBuilt: c.expectBuilt, classification, constructed };
  });
}

export async function writeJson(path, data) {
  await mkdir(dirname(path), { recursive: true });
  await writeFile(path, `${JSON.stringify(data, null, 2)}\n`, 'utf8');
}

export async function readJson(path) {
  return JSON.parse(await readFile(path, 'utf8'));
}

export { SCENE_NAMES };
