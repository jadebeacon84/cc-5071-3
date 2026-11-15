import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { extname, join, normalize } from 'node:path';
import { fileURLToPath } from 'node:url';
import { dirname } from 'node:path';

const root = dirname(dirname(fileURLToPath(import.meta.url)));
const port = process.env.PORT ? Number(process.env.PORT) : 8080;

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml'
};

const server = createServer(async (req, res) => {
  try {
    // Parse the raw path (not new URL, which applies RFC 3986 dot-segment
    // removal and would normalize an encoded %2e%2e before we can reject it).
    const rawPath = req.url.split('?')[0].split('#')[0];
    const urlPath = decodeURIComponent(rawPath);
    const rel = normalize(urlPath === '/' ? 'index.html' : urlPath.slice(1));
    const filePath = join(root, rel);
    // Reject any resolved path that escapes the site root (e.g. encoded ../).
    if (filePath !== root && !filePath.startsWith(root + '/')) {
      res.writeHead(400).end('bad path');
      return;
    }
    const data = await readFile(filePath);
    res.writeHead(200, { 'Content-Type': MIME[extname(filePath)] ?? 'application/octet-stream' });
    res.end(data);
  } catch (err) {
    if (err.code === 'ENOENT') {
      res.writeHead(404, { 'Content-Type': 'text/plain' }).end('not found');
    } else {
      res.writeHead(500, { 'Content-Type': 'text/plain' }).end(`server error: ${err.message}`);
    }
  }
});

server.listen(port, () => {
  console.log(`convex-contact-kit UI: http://localhost:${port}`);
});
