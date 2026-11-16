#!/usr/bin/env node
// Clean-directory release builder.
//
// The package is assembled in a freshly created empty directory from an
// explicit allow-list (never a working-tree snapshot), and the full test +
// manifold/distance verification is executed *inside that clean directory*.
// A sha256 manifest is recorded for every packaged file. The distributable
// archive is written only when every check passes; any failure removes the
// artifact so a broken convex-contact release can never be consumed.
import {
  rm, mkdir, cp, writeFile, readFile, stat, chmod
} from 'node:fs/promises';
import { existsSync } from 'node:fs';
import { join, relative, resolve, sep } from 'node:path';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { execGit, walk } from './lib/fs-util.js';

const ROOT = resolve(import.meta.dirname, '..');
const STAGING = join(ROOT, 'release', 'staging-clean');
const EVIDENCE = join(ROOT, 'release', 'evidence');
const pkg = JSON.parse(await readFile(join(ROOT, 'package.json'), 'utf8'));
const VERSION = pkg.version;

// Explicit allow-list: nothing outside this set enters the release. In
// particular there are no precomputed contact-point data files, and the
// mutable staging directory is never copied into itself.
const ALLOW_LIST = [
  'index.html',
  'package.json',
  'src',
  'renderer',
  'tests',
  'tools',
  'scripts',
  'docs'
];

function run(cmd, args, cwd, logFile) {
  return new Promise((resolvePromise, rejectPromise) => {
    const child = spawn(cmd, args, { cwd, env: process.env });
    let out = '';
    child.stdout.on('data', (d) => { out += d; process.stdout.write(d); });
    child.stderr.on('data', (d) => { out += d; process.stderr.write(d); });
    child.on('error', rejectPromise);
    child.on('close', async (code) => {
      await writeFile(logFile, out, 'utf8');
      code === 0 ? resolvePromise() : rejectPromise(new Error(`${cmd} ${args.join(' ')} exited ${code}`));
    });
  });
}

async function hashTree(base) {
  const entries = [];
  for (const file of await walk(base)) {
    const data = await readFile(file);
    entries.push({
      path: relative(base, file).split(sep).join('/'),
      sha256: createHash('sha256').update(data).digest('hex'),
      bytes: data.length
    });
  }
  entries.sort((a, b) => a.path.localeCompare(b.path));
  return entries;
}

async function main() {
  if (existsSync(STAGING)) await rm(STAGING, { recursive: true, force: true });
  await mkdir(STAGING, { recursive: true });
  await mkdir(EVIDENCE, { recursive: true });

  // 1. Assemble from the allow-list into the empty staging directory.
  const copied = [];
  for (const item of ALLOW_LIST) {
    const src = join(ROOT, item);
    if (!existsSync(src)) continue;
    const dst = join(STAGING, item);
    await mkdir(join(dst, '..'), { recursive: true });
    await cp(src, dst, { recursive: true });
    copied.push(item);
  }

  // Launcher scripts must be executable from the clean checkout.
  for (const sh of await walk(join(STAGING, 'scripts'))) {
    await chmod(sh, 0o755);
  }

  const checks = [
    { name: 'node-test', cmd: process.execPath, args: ['--test', ...(await findTests(STAGING))], log: 'clean-test.log' },
    { name: 'degenerate', cmd: process.execPath, args: ['tools/cli.js', 'degenerate', '--out', 'release-evidence/degenerate.json'], log: 'clean-degenerate.log' },
    { name: 'scene-check', cmd: process.execPath, args: ['tools/cli.js', 'check', '--dir', 'release-evidence/runs'], log: 'clean-check.log' }
  ];

  const evidenceDir = join(STAGING, 'release-evidence');
  await mkdir(evidenceDir, { recursive: true });
  const checkReport = [];
  let failed = null;
  for (const c of checks) {
    try {
      await run(c.cmd, c.args, STAGING, join(evidenceDir, c.log));
      checkReport.push({ name: c.name, ok: true, log: `release-evidence/${c.log}` });
    } catch (err) {
      checkReport.push({ name: c.name, ok: false, log: `release-evidence/${c.log}`, error: err.message });
      failed = c.name;
      break;
    }
  }

  // 2. Manifest of everything staged, with hashes and the source commit.
  const gitHead = await execGit(ROOT, ['rev-parse', 'HEAD']).catch(() => 'unknown');
  const files = await hashTree(STAGING);
  const manifest = {
    schema: 'convex-contact-release-manifest/v1',
    name: pkg.name,
    version: VERSION,
    builtAt: new Date().toISOString(),
    sourceCommit: gitHead.trim(),
    allowList: copied,
    checks: checkReport,
    files
  };
  await writeFile(join(STAGING, 'release-manifest.json'), `${JSON.stringify(manifest, null, 2)}\n`);

  // Persist the clean-run evidence and manifest next to the archive as the
  // repository's run evidence. Re-hash now that the manifest itself exists.
  await rm(join(EVIDENCE, 'clean-run'), { recursive: true, force: true });
  await cp(evidenceDir, join(EVIDENCE, 'clean-run'), { recursive: true });

  const archive = join(ROOT, 'release', `${pkg.name}-${VERSION}.tar.gz`);
  if (existsSync(archive)) await rm(archive);

  if (failed) {
    // Convex-contact failure: no artifact is published.
    await writeFile(join(EVIDENCE, 'clean-run', 'FAILED.json'),
      `${JSON.stringify({ failed, checkReport }, null, 2)}\n`);
    console.error(`release FAILED at '${failed}'; no archive produced. See release/evidence/clean-run/`);
    process.exit(1);
  }

  const finalFiles = await hashTree(STAGING);
  manifest.files = finalFiles;
  await writeFile(join(STAGING, 'release-manifest.json'), `${JSON.stringify(manifest, null, 2)}\n`);
  await writeFile(join(EVIDENCE, 'release-manifest.json'), `${JSON.stringify(manifest, null, 2)}\n`);

  // 3. Archive only after every check is green.
  await run('tar', ['-czf', archive, '-C', join(ROOT, 'release'), 'staging-clean'], ROOT,
    join(EVIDENCE, 'clean-run', 'packaging.log'));
  const archiveStat = await stat(archive);
  console.log(`release OK: ${relative(ROOT, archive)} (${archiveStat.size} bytes, ${finalFiles.length} files)`);
  console.log(`evidence: release/evidence/clean-run + release/evidence/release-manifest.json`);
}

async function findTests(base) {
  const tests = [];
  for (const f of await walk(join(base, 'tests'))) {
    if (f.endsWith('.test.js')) tests.push(relative(base, f));
  }
  return tests.sort();
}

main().catch((err) => {
  console.error(`release error: ${err.stack || err.message}`);
  process.exit(2);
});
