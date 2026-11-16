#!/usr/bin/env node
// Headless command-line interface. The graphical canvas remains the primary
// inspection entry; this CLI only assists with offline runs, raw records and
// re-verification. No network access and no third-party packages are used.
import { resolve } from 'node:path';
import {
  runSceneRecord,
  verifyRecord,
  probeDegenerate,
  writeJson,
  readJson,
  SCENE_NAMES
} from './lib/records.js';

const DEFAULT_FRAMES = 240;

function usage() {
  console.log(`convex-contact-kit CLI (headless assistant)

usage:
  node tools/cli.js scenes
      List the built-in collision scenes.

  node tools/cli.js run [scene] [--frames N] [--out FILE]
      Actually advance a scene with world.step + solver.step and write the raw
      per-frame record (GJK distance, EPA depth/normal, manifold, separating
      vector, impulse trace). Default scene: push. Default frames: ${DEFAULT_FRAMES}.

  node tools/cli.js verify --record FILE [--out FILE]
      Replay a raw record from scratch and recompute every manifold validity
      predicate and distance against live geometry. Exits non-zero on failure.

  node tools/cli.js check [--frames N] [--dir DIR]
      Run every scene, write raw records and re-verify each one in a single
      offline pass. Exits non-zero if any scene fails.

  node tools/cli.js degenerate [--out FILE]
      Probe degenerate convex inputs (point, line, two-point, non-finite) and
      print their explicit status; a recoverable noisy polygon is included.
`);
}

async function main() {
  const [command, ...rest] = process.argv.slice(2);
  const flags = parseFlags(rest);
  const positional = flags._;

  switch (command) {
    case 'scenes': {
      for (const name of SCENE_NAMES) console.log(name);
      return 0;
    }

    case 'run': {
      const scene = positional[0] ?? 'push';
      if (!SCENE_NAMES.includes(scene)) throw new Error(`unknown scene '${scene}'; try: ${SCENE_NAMES.join(', ')}`);
      const frames = Number(flags.frames ?? DEFAULT_FRAMES);
      const record = await runSceneRecord(scene, frames);
      const out = flags.out ? resolve(flags.out) : null;
      if (out) await writeJson(out, record);
      console.log(summarizeRecord(record, out));
      return 0;
    }

    case 'verify': {
      if (!flags.record) throw new Error("verify requires --record FILE");
      const record = await readJson(resolve(flags.record));
      const summary = verifyRecord(record);
      if (flags.out) await writeJson(resolve(flags.out), summary);
      console.log(renderSummary(summary));
      return summary.ok ? 0 : 1;
    }

    case 'check': {
      const frames = Number(flags.frames ?? DEFAULT_FRAMES);
      const dir = resolve(flags.dir ?? 'release/evidence/runs');
      let allOk = true;
      const index = [];
      for (const scene of SCENE_NAMES) {
        const record = await runSceneRecord(scene, frames);
        const recordPath = `${dir}/${scene}.record.json`;
        await writeJson(recordPath, record);
        const summary = verifyRecord(record);
        const summaryPath = `${dir}/${scene}.verify.json`;
        await writeJson(summaryPath, summary);
        index.push({ scene, recordPath, summaryPath, ok: summary.ok });
        console.log(renderSummary(summary));
        if (!summary.ok) allOk = false;
      }
      await writeJson(`${dir}/index.json`, {
        schema: 'convex-contact-evidence-index/v1',
        generatedAt: new Date().toISOString(),
        frames,
        scenes: index
      });
      console.log(allOk
        ? `all ${SCENE_NAMES.length} scenes verified; evidence in ${dir}`
        : `FAILURES present; evidence in ${dir}`);
      return allOk ? 0 : 1;
    }

    case 'degenerate': {
      const report = { schema: 'convex-contact-degenerate/v1', probes: probeDegenerate() };
      if (flags.out) await writeJson(resolve(flags.out), report);
      for (const p of report.probes) {
        const c = p.classification;
        const built = p.constructed.built
          ? `built(${p.constructed.vertexCount} verts)`
          : `rejected[${p.constructed.status ?? 'error'}]: ${p.constructed.error}`;
        const expectationMet = p.constructed.built === p.expectBuilt ? 'ok' : 'MISMATCH';
        console.log(`${p.name.padEnd(24)} status=${c.status.padEnd(11)} ${built} [${expectationMet}]`);
      }
      const bad = report.probes.filter((p) => p.constructed.built !== p.expectBuilt);
      if (bad.length) throw new Error('degenerate probe expectations were not met');
      return 0;
    }

    default:
      usage();
      return command === undefined ? 0 : 1;
  }
}

function parseFlags(args) {
  const out = { _: [] };
  for (let i = 0; i < args.length; i++) {
    const a = args[i];
    if (a.startsWith('--')) {
      const key = a.slice(2);
      const next = args[i + 1];
      if (next === undefined || next.startsWith('--')) out[key] = true;
      else {
        out[key] = next;
        i++;
      }
    } else {
      out._.push(a);
    }
  }
  return out;
}

function summarizeRecord(record, out) {
  const pairFrames = record.framesData.reduce((n, fr) => n + fr.pairs.length, 0);
  const issues = record.framesData.reduce((n, fr) => n + fr.auditIssues.length, 0);
  return [
    `scene=${record.scene} frames=${record.frames} pair-observations=${pairFrames} audit-issues=${issues}`,
    out ? `raw record: ${out}` : '(use --out FILE to persist the raw record)'
  ].join('\n');
}

function renderSummary(s) {
  const head = `[${s.ok ? 'PASS' : 'FAIL'}] scene=${s.scene} pair-checks=${s.pairChecks}`;
  const lines = [head];
  if (s.manifoldFailures.length) lines.push(`  manifold failures: ${JSON.stringify(s.manifoldFailures.slice(0, 5))}`);
  if (s.distanceMismatches.length) lines.push(`  distance mismatches: ${JSON.stringify(s.distanceMismatches.slice(0, 5))}`);
  if (s.auditFailures.length) lines.push(`  audit failures: ${JSON.stringify(s.auditFailures.slice(0, 5))}`);
  if (s.solverResidualFailures.length) lines.push(`  residual failures: ${s.solverResidualFailures.length}`);
  return lines.join('\n');
}

main().then((code) => process.exit(code)).catch((err) => {
  console.error(`error: ${err.message}`);
  process.exit(2);
});
