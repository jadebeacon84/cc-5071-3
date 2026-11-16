import { readdir, stat } from 'node:fs/promises';
import { join } from 'node:path';
import { spawn } from 'node:child_process';

// Recursive file listing (directories themselves are omitted).
export async function walk(dir) {
  const out = [];
  if (!await exists(dir)) return out;
  for (const name of await readdir(dir)) {
    const full = join(dir, name);
    const s = await stat(full);
    if (s.isDirectory()) out.push(...await walk(full));
    else out.push(full);
  }
  return out;
}

async function exists(p) {
  try {
    await stat(p);
    return true;
  } catch {
    return false;
  }
}

export function execGit(cwd, args) {
  return new Promise((resolvePromise, rejectPromise) => {
    const child = spawn('git', args, { cwd });
    let out = '';
    let err = '';
    child.stdout.on('data', (d) => { out += d; });
    child.stderr.on('data', (d) => { err += d; });
    child.on('error', rejectPromise);
    child.on('close', (code) => (code === 0 ? resolvePromise(out) : rejectPromise(new Error(err.trim() || `git ${args.join(' ')} failed`))));
  });
}
