// js/panels/viz/viz.js - 2D top-down map using plain canvas.
// Coordinate system: +X right, +Y up  (matches arena world frame).
// No external dependencies.

import { on, deg2rad } from '../../shared/utils.js';
import { sendJson } from '../../shared/ws.js';

const canvas = document.getElementById('vizCanvas');
const ctx    = canvas.getContext('2d');

// ── Camera (world-mm at canvas centre, zoom in px/mm) ─────────────────────────
const cam = { x: 0, y: 0, zoom: 0.25 };
let followRobot = false;
let showGrid    = false;

const WALL_THRESH = 9;

// World (+x right, +y up) → canvas px (+sx right, +sy down)
function w2c(wx, wy) {
  return [
    canvas.width  * 0.5 + (wx - cam.x) * cam.zoom,
    canvas.height * 0.5 - (wy - cam.y) * cam.zoom,
  ];
}

// Canvas px → world mm
function c2w(sx, sy) {
  return [
    cam.x + (sx - canvas.width  * 0.5) / cam.zoom,
    cam.y - (sy - canvas.height * 0.5) / cam.zoom,
  ];
}

// ── Arena dimensions ──────────────────────────────────────────────────────────
const arena = { widthMm: 555, heightMm: 355 };

// Known arena tag positions — mirrors PoseService::applyArenaLayout exactly.
// 2 tags per wall at 1/3 and 2/3, clockwise from top-left (IDs 0-7).
const arenaTagPos = new Map();

function buildArenaTagPos() {
  const W = arena.widthMm, H = arena.heightMm;
  // face_deg: direction the tag surface faces, CCW from +X — matches firmware applyArenaLayout()
  arenaTagPos.set(0, { x: W/3,   y: H,     face_deg: 270 }); // top wall    → faces down
  arenaTagPos.set(1, { x: 2*W/3, y: H,     face_deg: 270 }); // top wall    → faces down
  arenaTagPos.set(2, { x: W,     y: 2*H/3, face_deg: 180 }); // right wall  → faces left
  arenaTagPos.set(3, { x: W,     y: H/3,   face_deg: 180 }); // right wall  → faces left
  arenaTagPos.set(4, { x: 2*W/3, y: 0,     face_deg: 90  }); // bottom wall → faces up
  arenaTagPos.set(5, { x: W/3,   y: 0,     face_deg: 90  }); // bottom wall → faces up
  arenaTagPos.set(6, { x: 0,     y: H/3,   face_deg: 0   }); // left wall   → faces right
  arenaTagPos.set(7, { x: 0,     y: 2*H/3, face_deg: 0   }); // left wall   → faces right
}

buildArenaTagPos();

fetch('/api/config')
  .then(r => r.json())
  .then(cfg => {
    if (cfg?.arena) {
      arena.widthMm  = cfg.arena.width_mm  ?? 555;
      arena.heightMm = cfg.arena.height_mm ?? 355;
    }
    buildArenaTagPos();
    fitArena();
  })
  .catch(() => fitArena());

function fitArena() {
  cam.x = arena.widthMm  / 2;
  cam.y = arena.heightMm / 2;
  // widthMm maps to canvas width, heightMm maps to canvas height
  const zx = canvas.width  * 0.82 / arena.widthMm;
  const zy = canvas.height * 0.82 / arena.heightMm;
  cam.zoom = Math.min(zx, zy, 2);
}

// ── Map / pose / tag state ─────────────────────────────────────────────────────
let mapData = null;
const pose  = { x: 0, y: 0, yaw_d: 0, have: false };

const TAG_EMA_ALPHA   = 0.15;
const TAG_LOCK_N      = 10;
const TAG_RESET_MM    = 600;
const TAG_DEADBAND_MM = 30;
const tagMarkers      = new Map();   // non-arena tags only
const tagObs          = new Map();   // id → {az_deg, dist_mm, world_x, world_y, yaw_d, t} for debug

// ── Draw ──────────────────────────────────────────────────────────────────────

function draw() {
  const W = canvas.width, H = canvas.height;

  ctx.fillStyle = '#f5f8f5';
  ctx.fillRect(0, 0, W, H);

  if (showGrid) drawGrid(W, H);
  drawArena();
  if (mapData)  drawMap();
  drawArenaTags();
  drawTagObservations();
  for (const [id, t] of tagMarkers) drawTag(id, t);
  if (pose.have) drawRobot();

  requestAnimationFrame(draw);
}

function drawGrid(W, H) {
  const step = 500;

  const xMin = cam.x - W * 0.5 / cam.zoom;
  const xMax = cam.x + W * 0.5 / cam.zoom;
  const yMin = cam.y - H * 0.5 / cam.zoom;
  const yMax = cam.y + H * 0.5 / cam.zoom;

  ctx.strokeStyle = '#ccdacc';
  ctx.lineWidth   = 1;
  ctx.beginPath();

  for (let x = Math.ceil(xMin / step) * step; x <= xMax; x += step) {
    const [sx] = w2c(x, 0);
    ctx.moveTo(sx, 0); ctx.lineTo(sx, H);
  }
  for (let y = Math.ceil(yMin / step) * step; y <= yMax; y += step) {
    const [, sy] = w2c(0, y);
    ctx.moveTo(0, sy); ctx.lineTo(W, sy);
  }
  ctx.stroke();

  const [ox, oy] = w2c(0, 0);
  ctx.strokeStyle = '#a8c4a8';
  ctx.lineWidth   = 1.5;
  ctx.beginPath();
  ctx.moveTo(ox, 0); ctx.lineTo(ox, H);
  ctx.moveTo(0, oy); ctx.lineTo(W, oy);
  ctx.stroke();
}

function drawArena() {
  const AW = arena.widthMm, AH = arena.heightMm;

  ctx.beginPath();
  ctx.moveTo(...w2c(0,  0));
  ctx.lineTo(...w2c(AW, 0));
  ctx.lineTo(...w2c(AW, AH));
  ctx.lineTo(...w2c(0,  AH));
  ctx.closePath();
  ctx.strokeStyle = '#000';
  ctx.lineWidth   = 2;
  ctx.stroke();
}

function drawMap() {
  const { origin_x, origin_y, cell_mm, nx, ny, cells } = mapData;
  const sz = cell_mm * cam.zoom;

  for (let row = 0; row < ny; row++) {
    for (let col = 0; col < nx; col++) {
      const v = cells[row * nx + col];
      if (v <= WALL_THRESH) continue;

      const cx = origin_x + row * cell_mm + cell_mm * 0.5;
      const cy = origin_y + col * cell_mm + cell_mm * 0.5;
      const [sx, sy] = w2c(cx, cy);
      const t = v / 255;

      ctx.fillStyle = `hsl(130,50%,${Math.round(68 - 48 * t)}%)`;
      ctx.fillRect(sx - sz * 0.5, sy - sz * 0.5, sz, sz);
    }
  }
}

function drawRobot() {
  const [sx, sy] = w2c(pose.x, pose.y);
  const r        = Math.max(33 * cam.zoom, 5);
  const yaw      = deg2rad(pose.yaw_d);

  ctx.beginPath();
  ctx.arc(sx, sy, r, 0, Math.PI * 2);
  ctx.fillStyle   = '#A2C594';
  ctx.fill();
  ctx.strokeStyle = '#5a8a6a';
  ctx.lineWidth   = 1.5;
  ctx.stroke();

  // Heading arrow: +x right on screen, +y up on screen
  // yaw=0 → facing +x → arrow points right
  const arrowPx = Math.max(r + 10, 14);
  ctx.beginPath();
  ctx.moveTo(sx, sy);
  ctx.lineTo(sx + Math.cos(yaw) * arrowPx, sy - Math.sin(yaw) * arrowPx);
  ctx.strokeStyle = '#1a2e20';
  ctx.lineWidth   = 2;
  ctx.stroke();
}

function drawTag(id, t) {
  const [sx, sy] = w2c(t.x_mm, t.y_mm);
  const s        = Math.max(33 * cam.zoom, 6);

  ctx.fillStyle   = '#ffeb3b';
  ctx.fillRect(sx - s, sy - s, s * 2, s * 2);
  ctx.strokeStyle = '#aa8800';
  ctx.lineWidth   = 1.5;
  ctx.strokeRect(sx - s, sy - s, s * 2, s * 2);

  const fontSize = Math.max(9, Math.round(s * 0.9));
  ctx.fillStyle      = '#000';
  ctx.font           = `bold ${fontSize}px system-ui,sans-serif`;
  ctx.textAlign      = 'center';
  ctx.textBaseline   = 'middle';
  ctx.fillText(String(id), sx, sy);
}

// ── Debug: arena tag markers + observation lines ──────────────────────────────

function drawArenaTags() {
  const s = Math.max(12 * cam.zoom, 9);
  for (const [id, t] of arenaTagPos) {
    const [sx, sy] = w2c(t.x, t.y);

    // Convert face_deg (world CCW from +X) to canvas direction.
    // Canvas Y is flipped relative to world (+Y world = -Y canvas), so negate sin.
    const fRad = (t.face_deg ?? 0) * Math.PI / 180;
    const dx =  Math.cos(fRad);   // canvas X component of face direction
    const dy = -Math.sin(fRad);   // canvas Y component (negated for Y-flip)
    const px = -dy, py = dx;      // perpendicular — used for triangle base width

    // Arrow triangle: tip points into the arena (face direction), base sits behind centre
    const tipX  = sx + dx * s * 1.5,  tipY  = sy + dy * s * 1.5;
    const baseX = sx - dx * s * 0.6,  baseY = sy - dy * s * 0.6;

    ctx.beginPath();
    ctx.moveTo(tipX, tipY);
    ctx.lineTo(baseX + px * s * 0.9, baseY + py * s * 0.9);
    ctx.lineTo(baseX - px * s * 0.9, baseY - py * s * 0.9);
    ctx.closePath();
    ctx.fillStyle   = '#7b1fa2';
    ctx.strokeStyle = '#fff';
    ctx.lineWidth   = 1.5;
    ctx.fill();
    ctx.stroke();

    // ID label at the shape centre
    ctx.fillStyle    = '#fff';
    ctx.font         = `bold ${Math.max(9, Math.round(s * 0.85))}px system-ui,sans-serif`;
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(String(id), sx, sy);
  }
}

function drawTagObservations() {
  if (!pose.have) return;
  const now = Date.now();
  for (const [id, obs] of tagObs) {
    if (now - obs.t > 2000) continue;
    if (!Number.isFinite(obs.robot_x)) continue;
    const alpha = Math.max(0, 1 - (now - obs.t) / 2000);

    // Dot at computed ROBOT position (where robot must be to see this tag at this angle)
    const [dx, dy] = w2c(obs.robot_x, obs.robot_y);
    ctx.save();
    ctx.globalAlpha = alpha;

    // Line from computed robot position to the known tag marker
    const knownTag = arenaTagPos.get(id);
    if (knownTag) {
      const [tx, ty] = w2c(knownTag.x, knownTag.y);
      ctx.beginPath();
      ctx.moveTo(dx, dy);
      ctx.lineTo(tx, ty);
      ctx.strokeStyle = '#e91e63';
      ctx.lineWidth   = 1;
      ctx.setLineDash([4, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Cross-hair dot at estimated robot position
    ctx.beginPath();
    ctx.arc(dx, dy, 5, 0, Math.PI * 2);
    ctx.fillStyle   = '#e91e63';
    ctx.strokeStyle = '#fff';
    ctx.lineWidth   = 1.5;
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle    = '#b71c1c';
    ctx.font         = '10px monospace';
    ctx.textAlign    = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText(`id:${id} az:${obs.az_deg.toFixed(1)}° d:${obs.dist_mm.toFixed(0)}mm`, dx + 7, dy - 12);
    ctx.fillText(`yaw:${obs.yaw_d.toFixed(1)}° → pos(${obs.robot_x.toFixed(0)},${obs.robot_y.toFixed(0)})`, dx + 7, dy);
    ctx.restore();
  }
}

// ── Input: pan and zoom ───────────────────────────────────────────────────────

let drag = null;

canvas.addEventListener('mousedown', e => {
  if (e.button === 0 || e.button === 1) {
    drag = { ex: e.clientX, ey: e.clientY, cx: cam.x, cy: cam.y };
    e.preventDefault();
  }
});

window.addEventListener('mouseup', () => { drag = null; });

window.addEventListener('mousemove', e => {
  if (!drag) return;
  // Drag right → cam.x decreases (see more world to right)
  // Drag up   → cam.y increases (see more world above)
  cam.x = drag.cx + (drag.ex - e.clientX) / cam.zoom;
  cam.y = drag.cy + (drag.ey - e.clientY) / cam.zoom;
  followRobot = false;
});

canvas.addEventListener('wheel', e => {
  e.preventDefault();
  const rect     = canvas.getBoundingClientRect();
  const mx       = e.clientX - rect.left;
  const my       = e.clientY - rect.top;
  const [wx, wy] = c2w(mx, my);

  const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15;
  cam.zoom = Math.max(0.02, Math.min(cam.zoom * factor, 5));

  // Keep world point under cursor fixed
  cam.x = wx - (mx - canvas.width  * 0.5) / cam.zoom;
  cam.y = wy + (my - canvas.height * 0.5) / cam.zoom;
}, { passive: false });

// ── Events ────────────────────────────────────────────────────────────────────

on('pose', ({ detail: p }) => {
  pose.x     = p.x_mm;
  pose.y     = p.y_mm;
  pose.yaw_d = p.yaw_d;
  pose.have  = true;
  if (followRobot) { cam.x = pose.x; cam.y = pose.y; }
});

on('mapgrid', ({ detail: d }) => {
  mapData    = d;
  pose.x     = d.robot_x;
  pose.y     = d.robot_y;
  pose.yaw_d = d.robot_yaw * (180 / Math.PI);
  pose.have  = true;
  if (followRobot) { cam.x = pose.x; cam.y = pose.y; }
});

function handleTag(o) {
  if (!o || !pose.have) return;
  const id = o.id ?? 0;

  const azDeg = Number(o.azDeg);
  if (!Number.isFinite(azDeg)) return;
  let r_mm = NaN;
  if (Number.isFinite(o.dist_m)) r_mm = o.dist_m * 1000;
  if (!Number.isFinite(r_mm) || r_mm < 50) return;

  const yawRad = deg2rad(pose.yaw_d);
  const azRad  = deg2rad(azDeg);
  const bx     = r_mm * Math.cos(azRad);
  const by     = -r_mm * Math.sin(azRad);
  const tw_x   = pose.x + Math.cos(yawRad) * bx - Math.sin(yawRad) * by;
  const tw_y   = pose.y + Math.sin(yawRad) * bx + Math.cos(yawRad) * by;

  // Compute robot position estimate from this tag (for arena tags we know the tag world pos)
  let robot_x = NaN, robot_y = NaN;
  const knownTag = arenaTagPos.get(id);
  if (knownTag) {
    // robot_world = tag_world - R(yaw) * body_offset_to_tag
    const yawRad2 = deg2rad(pose.yaw_d);
    robot_x = knownTag.x - (Math.cos(yawRad2) * bx - Math.sin(yawRad2) * by);
    robot_y = knownTag.y - (Math.sin(yawRad2) * bx + Math.cos(yawRad2) * by);
  }
  tagObs.set(id, { az_deg: azDeg, dist_mm: r_mm, robot_x, robot_y, yaw_d: pose.yaw_d, t: Date.now() });

  // Arena tags (0-7) are fixed — only track floating markers for id >= 8
  if (id < 8) return;

  let m = tagMarkers.get(id);
  if (!m) { tagMarkers.set(id, { x_mm: tw_x, y_mm: tw_y, nObs: 1 }); return; }

  const nObs = m.nObs + 1;
  const dist = Math.hypot(tw_x - m.x_mm, tw_y - m.y_mm);
  if (nObs >= TAG_LOCK_N) {
    if (dist >= TAG_RESET_MM) { m.x_mm = tw_x; m.y_mm = tw_y; }
  } else if (dist > TAG_DEADBAND_MM) {
    m.x_mm += TAG_EMA_ALPHA * (tw_x - m.x_mm);
    m.y_mm += TAG_EMA_ALPHA * (tw_y - m.y_mm);
  }
  m.nObs = nObs;
}

on('apriltag',             ({ detail }) => handleTag(detail));
on('apriltag/observation', ({ detail }) => handleTag(detail));

// ── Resize ────────────────────────────────────────────────────────────────────

function resizeCanvas() {
  const outer = document.getElementById('panel-viz');
  const bar   = outer.querySelector('.tab-bar');
  canvas.width  = outer.clientWidth  || 600;
  canvas.height = (outer.clientHeight - (bar ? bar.offsetHeight : 0)) || 600;
}

new ResizeObserver(resizeCanvas).observe(document.getElementById('panel-viz'));
resizeCanvas();

// ── Tab switching ─────────────────────────────────────────────────────────────

const tabBtns  = document.querySelectorAll('#panel-viz .tab-btn');
const tabPanes = document.querySelectorAll('#panel-viz .tab-pane');

tabBtns.forEach(btn => {
  btn.addEventListener('click', () => {
    tabBtns.forEach(b  => b.classList.remove('active'));
    tabPanes.forEach(p => p.classList.remove('active'));
    btn.classList.add('active');
    document.getElementById('tab-' + btn.dataset.tab).classList.add('active');
    if (btn.dataset.tab === 'map')    resizeCanvas();
    if (btn.dataset.tab === 'camera') startCamera();
    else                              stopCamera();
  });
});

// ── Camera feed ───────────────────────────────────────────────────────────────

const camFeed = document.getElementById('camFeed');

function startCamera() {
  camFeed.src = `http://${location.hostname}:81/mjpeg`;
}

function stopCamera() {
  camFeed.src = '';
}

// ── Camera servo overlay ──────────────────────────────────────────────────────

const camServo1    = document.getElementById('camServo1');
const camServo1Val = document.getElementById('camServo1Val');

if (camServo1) {
  let servoTimer = null;
  camServo1.addEventListener('input', () => {
    const deg = Number(camServo1.value);
    camServo1Val.textContent = `${deg}°`;
    clearTimeout(servoTimer);
    servoTimer = setTimeout(() => sendJson({ servo1: deg }), 20);
  });
}

// ── Start render loop ─────────────────────────────────────────────────────────

draw();
setTimeout(startCamera, 800);

// ── Public API ────────────────────────────────────────────────────────────────

window.__viz_setScale    = s  => { cam.zoom = 0.02 + (s / 3) * 0.98; };
window.__viz_setGrid     = show => { showGrid = show; };
window.__viz_clearMap    = ()   => { mapData  = null; };
window.__viz_clearTags   = ()   => { tagMarkers.clear(); };
window.__viz_fitArena    = fitArena;
window.__viz_resetCamera = () => {
  cam.x = pose.have ? pose.x : arena.widthMm  / 2;
  cam.y = pose.have ? pose.y : arena.heightMm / 2;
};
window.__viz_followBot   = enabled => {
  followRobot = enabled;
  if (enabled && pose.have) { cam.x = pose.x; cam.y = pose.y; }
};
