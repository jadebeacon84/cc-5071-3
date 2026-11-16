# Release process

The release is built in a **fresh, empty directory** from an explicit
allow-list and is only published when every check passes inside that clean
directory.

```sh
npm run release      # node tools/package-release.js
```

## Steps performed by `tools/package-release.js`

1. Delete and recreate `release/staging-clean/` (the clean directory).
2. Copy only the allow-listed paths (`index.html`, `package.json`, `src`,
   `renderer`, `tests`, `tools`, `scripts`, `docs`). A working-tree snapshot is
   never copied, and no precomputed contact data exists to ship.
3. Inside the clean directory, run:
   - the full `node --test` unit suite;
   - `node tools/cli.js degenerate` (explicit degenerate convex status);
   - `node tools/cli.js check` (every scene actually simulated, raw records
     written, then independently re-verified for manifold validity,
     separating vector and distance).
4. Hash every staged file (sha256) into `release-manifest.json` with the
   source commit and per-check results.
5. Write the run evidence to `release/evidence/`.
6. **Only if every check is green**, create
   `release/convex-contact-kit-<version>.tar.gz`.

## Failure means no artifact

If any check fails, the builder exits non-zero, writes
`release/evidence/clean-run/FAILED.json`, and does **not** create or update the
archive. A broken convex-contact release therefore cannot be consumed. The
failure path is exercised as part of development; search the release history
for the `FAILED.json` behavior.

## Committed run evidence

Tracked under `release/` (the mutable `staging-clean/` and local-run output are
gitignored):

- `convex-contact-kit-<version>.tar.gz` — the distributable;
- `evidence/release-manifest.json` — allow-list, checks, per-file hashes;
- `evidence/clean-run/` — clean-directory test/degenerate/scene logs and the
  raw `*.record.json` / `*.verify.json` pairs for every scene.

Manifold validity and distance claims are backed by these original records;
`verifyRecord` recomputes each claim from a fresh replay, so the evidence
cannot be made to pass by editing numbers.

## Verifying a distributed archive

```sh
mkdir clean && tar -xzf release/convex-contact-kit-1.0.0.tar.gz -C clean
cd clean/staging-clean
node --test tests/*.test.js
node tools/cli.js check --dir /tmp/replayed-runs
sh scripts/start-ui.sh   # graphical entry works from the extracted copy
```

The archive contains its own `release-manifest.json` and `release-evidence/`.
