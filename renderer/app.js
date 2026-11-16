import { Vec2 } from '../src/core/vec2.js';
import { buildScene, SCENE_NAMES } from '../src/geometry/scenes.js';
import { RESIDUAL_SLOP } from '../src/core/geom.js';
import { separatingVector, verifyManifold } from '../src/geometry/verify.js';
import { validateConvex } from '../src/geometry/validate.js';

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const panel = document.getElementById('panel');
const stage = document.getElementById('stage');
const banner = document.getElementById('error-banner');

const sceneSelect = document.getElementById('scene');
const playBtn = document.getElementById('play');
const stepBtn = document.getElementById('step');
const resetBtn = document.getElementById('reset');
const speedSelect = document.getElementById('speed');
const showSupport = document.getElementById('show-support');
const showEpa = document.getElementById('show-epa');
const showManifold = document.getElementById('show-manifold');
const showSeparation = document.getElementById('show-separation');
const showAabb = document.getElementById('show-aabb');

for (const name of SCENE_NAMES) {
  const opt = document.createElement('option');
  opt.value = name;
  opt.textContent = name;
  sceneSelect.appendChild(opt);
}

const COLORS = {
  body: '#6fb1f0',
  static: '#9fb4c7',
  bodyFill: 'rgba(111,177,240,0.12)',
  staticFill: 'rgba(159,180,199,0.10)',
  edge: '#3a4a5c',
  contact: '#ff8a5c',
  normal: '#ffe08a',
  support: '#c792ea',
  epa: '#4fc3f7',
  separation: '#ef7d7d',
  aabb: 'rgba(242,193,78,0.55)',
  witness: '#66d18e'
};

// Exceptions from the actual geometry/solver path are surfaced in the UI rather
// than silently dropping a frame. The banner is cleared on the next good frame.
function showError(err) {
  banner.textContent = `convex contact error: ${err?.message || err}`;
  banner.classList.add('show');
}
function clearError() {
  banner.classList.remove('show');
  banner.textContent = '';
}
window.addEventListener('unhandledrejection', (e) => showError(e.reason));
window.addEventListener('error', (e) => showError(e.message));

let scene = buildScene(sceneSelect.value);
let running = true;
let last = performance.now();
let accumulator = 0;
const FIXED_DT = 1 / 60;

// World-to-screen camera: world y is up, canvas y is down.
const camera = { scale: 64, cx: 0, cy: 0 };

function resize() {
  const dpr = window.devicePixelRatio || 1;
  const rect = stage.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  canvas.style.width = `${rect.width}px`;
  canvas.style.height = `${rect.height}px`;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  camera.w = rect.width;
  camera.h = rect.height;
}
window.addEventListener('resize', resize);

function toScreen(p) {
  return {
    x: camera.w / 2 + (p.x - camera.cx) * camera.scale,
    y: camera.h / 2 - (p.y - camera.cy) * camera.scale
  };
}
function toWorld(s) {
  return new Vec2(
    (s.x - camera.w / 2) / camera.scale + camera.cx,
    -(s.y - camera.h / 2) / camera.scale + camera.cy
  );
}

function line(a, b, color, width = 1, dash = null) {
  const pa = toScreen(a);
  const pb = toScreen(b);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash ?? []);
  ctx.beginPath();
  ctx.moveTo(pa.x, pa.y);
  ctx.lineTo(pb.x, pb.y);
  ctx.stroke();
  ctx.setLineDash([]);
}

function dot(p, color, radius = 3) {
  const s = toScreen(p);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(s.x, s.y, radius, 0, Math.PI * 2);
  ctx.fill();
}

function polygon(points, stroke, fill, width = 1.5) {
  if (points.length < 2) return;
  ctx.beginPath();
  points.forEach((p, i) => {
    const s = toScreen(p);
    if (i === 0) ctx.moveTo(s.x, s.y);
    else ctx.lineTo(s.x, s.y);
  });
  ctx.closePath();
  if (fill) {
    ctx.fillStyle = fill;
    ctx.fill();
  }
  ctx.strokeStyle = stroke;
  ctx.lineWidth = width;
  ctx.stroke();
}

function drawGrid() {
  const step = camera.scale;
  ctx.strokeStyle = 'rgba(255,255,255,0.04)';
  ctx.lineWidth = 1;
  const ox = camera.w / 2 - camera.cx * camera.scale;
  const oy = camera.h / 2 + camera.cy * camera.scale;
  for (let x = ox % step; x < camera.w; x += step) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, camera.h); ctx.stroke();
  }
  for (let y = oy % step; y < camera.h; y += step) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(camera.w, y); ctx.stroke();
  }
  // World axes.
  const o = toScreen(new Vec2(0, 0));
  ctx.strokeStyle = 'rgba(255,255,255,0.18)';
  ctx.beginPath(); ctx.moveTo(0, o.y); ctx.lineTo(camera.w, o.y); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(o.x, 0); ctx.lineTo(o.x, camera.h); ctx.stroke();
}

function drawAabb(body) {
  const b = body.shape.aabb();
  const corners = [
    new Vec2(b.minX, b.minY), new Vec2(b.maxX, b.minY),
    new Vec2(b.maxX, b.maxY), new Vec2(b.minX, b.maxY)
  ];
  polygon(corners, COLORS.aabb, null, 1);
}

function drawBody(body) {
  const shape = body.shape;
  const stroke = body.static ? COLORS.static : COLORS.body;
  const fill = body.static ? COLORS.staticFill : COLORS.bodyFill;
  polygon(shape.worldVerts, stroke, fill);
  // Mass centre / local origin marker.
  dot(body.xf.p, stroke, 2.5);
  // Local x axis to make orientation visible.
  const axis = new Vec2(Math.cos(body.xf.angle), Math.sin(body.xf.angle));
  line(body.xf.p, Vec2.add(body.xf.p, Vec2.mul(axis, 0.25)), stroke, 1);
}

// GJK support trail: each step's query direction and the Minkowski support.
function drawSupportTrail(pair) {
  const trail = pair.gjk.trail;
  if (!trail) return;
  const origin = new Vec2(0, 0);
  for (const step of trail.steps) {
    // Support direction emanates from the Minkowski origin (drawn at A's frame
    // origin for spatial context). The D points themselves are relative, so draw
    // them translated to body A's centre as a reference anchor.
    const anchor = pair.bodyA.xf.p;
    const dWorld = Vec2.add(anchor, step.point);
    dot(dWorld, COLORS.support, 2.5);
    line(anchor, Vec2.add(anchor, Vec2.mul(Vec2.normalize(step.dir), 0.5)), COLORS.support, 1, [3, 3]);
    // Witness points on the two real bodies.
    dot(step.a, COLORS.witness, 2);
    dot(step.b, COLORS.witness, 2);
  }
  void origin;
}

// EPA polytope lives in D = A - B space; translate to A's centre to visualize.
function drawEpa(pair) {
  const epa = pair.epa;
  if (!epa || !epa.trail || !epa.trail.polytopes) return;
  const anchor = pair.bodyA.xf.p;
  const snaps = epa.trail.polytopes;
  // Draw all expansion hulls faintly, the final one solid.
  snaps.forEach((poly, idx) => {
    const isLast = idx === snaps.length - 1;
    const pts = poly.map((p) => Vec2.add(anchor, p));
    polygon(pts, isLast ? COLORS.epa : 'rgba(79,195,247,0.30)', null, isLast ? 1.5 : 1);
  });
  // Penetration normal from the contact midpoint.
  for (const c of pair.manifold.contacts) {
    line(c.point, Vec2.add(c.point, Vec2.mul(c.normal, 0.45)), COLORS.normal, 2);
  }
}

function drawManifold(pair) {
  const m = pair.manifold;
  for (const c of m.contacts) {
    dot(c.point, COLORS.contact, 4.5);
    // Push-out normal n: B moves along +n to separate.
    line(c.point, Vec2.add(c.point, Vec2.mul(m.normal, 0.35)), COLORS.normal, 2);
    // Tangent span between the two retained contacts shows manifold extent.
  }
  if (m.contacts.length === 2) {
    line(m.contacts[0].point, m.contacts[1].point, COLORS.contact, 1.5, [4, 3]);
  }
}

// Separating vector computed live from GJK/SAT (not stored data): the minimum
// translation of B along a unit axis that clears A. Drawn from B's centre.
function drawSeparation(pair) {
  const sep = separatingVector(pair.bodyA.shape, pair.bodyB.shape, pair.gjk);
  if (sep.state === 'touch' || sep.vector.len2() <= 1e-14) return;
  const origin = pair.bodyB.xf.p;
  const tip = Vec2.add(origin, sep.vector);
  line(origin, tip, COLORS.separation, 2.5, [7, 4]);
  // Arrow head.
  const head = 0.11;
  const back = Vec2.add(tip, Vec2.mul(Vec2.neg(sep.normal), head));
  const side = Vec2.perpLeft(sep.normal);
  line(tip, Vec2.add(back, Vec2.mul(side, head * 0.55)), COLORS.separation, 2.5);
  line(tip, Vec2.sub(back, Vec2.mul(side, head * 0.55)), COLORS.separation, 2.5);
}

function render() {
  ctx.clearRect(0, 0, camera.w, camera.h);
  drawGrid();

  if (showAabb.checked) for (const body of scene.world.bodies) drawAabb(body);
  for (const body of scene.world.bodies) drawBody(body);

  for (const pair of scene.world.pairs) {
    if (showSupport.checked) drawSupportTrail(pair);
    if (showEpa.checked) drawEpa(pair);
    if (showManifold.checked) drawManifold(pair);
    if (showSeparation.checked) drawSeparation(pair);
  }
}

function fmt(v, digits = 3) {
  return Number.isFinite(v) ? v.toFixed(digits) : '–';
}
function vec(v, digits = 2) {
  return v ? `(${fmt(v.x, digits)}, ${fmt(v.y, digits)})` : '–';
}

function renderPanel() {
  const traces = scene.solver.traces;
  const issues = scene.world.auditIssues();
  const pairs = scene.world.pairs;

  let html = `<h2>Simulation</h2>
    <div class="kv"><span>scene</span><span>${scene.name}</span></div>
    <div class="kv"><span>frame</span><span>${scene.world.frame}</span></div>
    <div class="kv"><span>bodies</span><span>${scene.world.bodies.length}</span></div>
    <div class="kv"><span>active pairs</span><span>${pairs.length}</span></div>
    <div class="kv"><span>residual slop</span><span>${RESIDUAL_SLOP}</span></div>
    <div class="kv"><span>SAT cross-check</span>
      <span class="badge ${issues.length ? 'bad' : 'ok'}">${issues.length ? `${issues.length} issue(s)` : 'consistent'}</span>
    </div>`;

  for (const issue of issues.slice(0, 6)) {
    for (const msg of issue.issues) html += `<div class="issue">• ${msg}</div>`;
  }

  html += `<h2>Contact pairs</h2>`;
  if (pairs.length === 0) {
    html += `<div class="muted">No narrow-phase contacts. AABB broadphase alone never creates one.</div>`;
  }
  for (const pair of pairs) {
    const m = pair.manifold;
    const epa = pair.epa;
    // Live manifold validity + separating vector, recomputed from geometry.
    let verdict = null;
    try {
      verdict = verifyManifold(pair.bodyA.shape, pair.bodyB.shape, m);
    } catch (err) {
      showError(err);
    }
    const sep = (() => {
      try {
        return separatingVector(pair.bodyA.shape, pair.bodyB.shape, pair.gjk);
      } catch (err) {
        showError(err);
        return null;
      }
    })();
    html += `<div class="kv"><span>${pair.bodyA.label} → ${pair.bodyB.label}</span>
      <span class="badge method">${epa ? epa.method : 'separated'}</span></div>`;
    html += `<div class="kv"><span>manifold validity</span>
      <span class="badge ${verdict?.ok ? 'ok' : 'bad'}">${verdict ? (verdict.ok ? 'valid' : verdict.failedChecks.join(',')) : 'error'}</span></div>`;
    html += `<div class="kv"><span>normal n (push-out B)</span><span>${vec(m.normal)}</span></div>`;
    if (sep) {
      html += `<div class="kv"><span>separating vector (${sep.state})</span>
        <span>${vec(sep.vector)} |d|=${fmt(sep.magnitude, 4)}</span></div>`;
    }
    if (epa) {
      html += `<div class="kv"><span>EPA depth</span><span>${fmt(epa.depth, 4)}</span></div>`;
      html += `<div class="kv"><span>SAT depth</span><span>${fmt(pair.sat?.depth, 4)}</span></div>`;
      html += `<div class="kv"><span>GJK distance</span><span>${fmt(pair.gjk.distance, 4)}</span></div>`;
      html += `<div class="kv"><span>stop reason</span><span>${epa.stopReason}</span></div>`;
      if (pair.sat?.normal) {
        const dot = Vec2.dot(epa.normal, pair.sat.normal);
        html += `<div class="kv"><span>n · SAT normal</span><span>${fmt(dot, 5)}</span></div>`;
      }
    } else {
      html += `<div class="kv"><span>GJK gap</span><span>${fmt(pair.gjk.distance, 4)}</span></div>`;
    }
    html += `<div class="kv"><span>contacts (≤2, span kept)</span><span>${m.contacts.length}</span></div>`;
  }

  html += `<h2>Degenerate shape status</h2>`;
  let anyDegenerate = false;
  for (const body of scene.world.bodies) {
    const diagnosis = validateConvex(body.shape.localVerts);
    if (!diagnosis.ok) anyDegenerate = true;
    html += `<div class="kv ${diagnosis.ok ? '' : 'degen'}"><span>${body.label}</span>
      <span>${diagnosis.status} (${diagnosis.vertexCount}v${diagnosis.ok ? `, area ${fmt(diagnosis.area, 3)}` : ''})</span></div>`;
  }
  if (!anyDegenerate) html += `<div class="muted">All constructed bodies are valid strictly-convex polygons; rejected point/line inputs are reported via the CLI.</div>`;

  html += `<h2>Per-contact impulse trace</h2>`;
  if (traces.length === 0) {
    html += `<div class="muted">No solver contacts this frame.</div>`;
  } else {
    html += `<table><thead><tr>
      <th>pair</th><th>Jn</th><th>Jt</th><th>depth</th><th>residual</th><th>stop</th>
    </tr></thead><tbody>`;
    for (const t of traces) {
      const bad = t.residual > RESIDUAL_SLOP;
      html += `<tr>
        <td>${t.pair}</td>
        <td>${fmt(t.normalImpulse, 3)}</td>
        <td>${fmt(t.tangentImpulse, 3)}</td>
        <td>${fmt(t.depthBefore, 3)}</td>
        <td class="${bad ? 'issue' : ''}">${fmt(t.residual, 4)}</td>
        <td>${t.stopReason.split(':')[0]}</td>
      </tr>`;
    }
    html += `</tbody></table>`;
    html += `<div class="hint">Jn/Jt are accumulated normal/tangent impulses (warm-started).
      Residual is re-measured from body-local anchors after position correction.</div>`;
  }

  html += `<h2>Legend</h2><div class="legend">
    <span class="swatch" style="background:${COLORS.body}"></span><span>dynamic convex body</span>
    <span class="swatch" style="background:${COLORS.static}"></span><span>static body / floor</span>
    <span class="swatch" style="background:${COLORS.contact}"></span><span>manifold contact point</span>
    <span class="swatch" style="background:${COLORS.normal}"></span><span>push-out normal n (B moves +n)</span>
    <span class="swatch" style="background:${COLORS.support}"></span><span>GJK support query / D vertex</span>
    <span class="swatch" style="background:${COLORS.epa}"></span><span>EPA expanding polytope (D space)</span>
    <span class="swatch" style="background:${COLORS.separation}"></span><span>separating vector (live GJK/SAT)</span>
    <span class="swatch" style="background:${COLORS.witness}"></span><span>witness points on A and B</span>
    <span class="swatch" style="background:${COLORS.aabb}"></span><span>AABB broadphase (filter only)</span>
  </div>`;

  panel.innerHTML = html;
}

function frame(now) {
  try {
    const elapsed = Math.min(0.1, (now - last) / 1000);
    last = now;
    if (running) {
      accumulator += elapsed * Number(speedSelect.value);
      let guard = 0;
      while (accumulator >= FIXED_DT && guard < 8) {
        scene.world.step(FIXED_DT);
        scene.solver.step(FIXED_DT);
        accumulator -= FIXED_DT;
        guard++;
      }
    }
    render();
    renderPanel();
    clearError();
  } catch (err) {
    showError(err);
  }
  requestAnimationFrame(frame);
}

function resetScene() {
  scene = buildScene(sceneSelect.value);
  accumulator = 0;
}

playBtn.addEventListener('click', () => {
  running = !running;
  playBtn.textContent = running ? 'Pause' : 'Play';
  playBtn.classList.toggle('active', running);
});
stepBtn.addEventListener('click', () => {
  try {
    running = false;
    playBtn.textContent = 'Play';
    playBtn.classList.remove('active');
    scene.world.step(FIXED_DT);
    scene.solver.step(FIXED_DT);
    clearError();
  } catch (err) {
    showError(err);
  }
});
resetBtn.addEventListener('click', resetScene);
sceneSelect.addEventListener('change', resetScene);

// Mouse drag applies a real impulse to a prop, so the geometry visibly moves it.
let dragging = null;
canvas.addEventListener('pointerdown', (e) => {
  const rect = canvas.getBoundingClientRect();
  const w = toWorld(new Vec2(e.clientX - rect.left, e.clientY - rect.top));
  let best = null;
  for (const body of scene.world.bodies) {
    if (body.static) continue;
    const d2 = Vec2.dist2(w, body.xf.p);
    if (!best || d2 < best.d2) best = { body, d2 };
  }
  if (best && best.d2 < 1.5) dragging = { body: best.body, last: w };
});
canvas.addEventListener('pointermove', (e) => {
  if (!dragging) return;
  const rect = canvas.getBoundingClientRect();
  const w = toWorld(new Vec2(e.clientX - rect.left, e.clientY - rect.top));
  const delta = Vec2.sub(w, dragging.last);
  // Map pointer motion into an impulse scaled by body mass.
  const j = Vec2.mul(delta, dragging.body.mass * 18);
  dragging.body.applyImpulse(j, dragging.body.xf.p);
  dragging.last = w;
});
window.addEventListener('pointerup', () => { dragging = null; });

resize();
requestAnimationFrame(frame);
