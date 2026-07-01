import { on } from '../../shared/utils.js';

const root   = document.getElementById('panel-viz');
const canvas = root.querySelector('#viz');
const ctx    = canvas.getContext('2d', { alpha:false });

let W = canvas.width, H = canvas.height;
let cx = W/2, cy = H/2;

let scale = 0.5;     // px/mm
let showGrid = true; // toggled by Controls panel

const CELL_MM = 50;
const OCC_MAX = 12, OCC_MIN = -12;
const OCC_HIT = 3,  OCC_FREE = -1;

let pose = { have:false, x:0, y:0, yaw:0, seq:0, t:0 };
const occ = new Map();

function syncVizSize() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const w = Math.max(1, Math.round(rect.width  * dpr));
  const h = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w; canvas.height = h;
    W = canvas.width; H = canvas.height;
    cx = W/2; cy = H/2;
  }
}
function worldToScreen(x_mm, y_mm){ return [ Math.round(cx - x_mm*scale), Math.round(cy + y_mm*scale) ]; }

function drawGrid() {
  if (!showGrid) return;
  const stepPx = Math.max(20, Math.round(100 * scale));
  ctx.strokeStyle = '#1f2633';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = cx; x < W; x += stepPx) { ctx.moveTo(x,0); ctx.lineTo(x,H); }
  for (let x = cx; x > 0;  x -= stepPx) { ctx.moveTo(x,0); ctx.lineTo(x,H); }
  for (let y = cy; y < H; y += stepPx) { ctx.moveTo(0,y); ctx.lineTo(W,y); }
  for (let y = cy; y > 0;  y -= stepPx) { ctx.moveTo(0,y); ctx.lineTo(W,y); }
  ctx.stroke();
}
function drawAxes(){
  ctx.strokeStyle = '#2e3647'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0,cy); ctx.lineTo(W,cy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx,0); ctx.lineTo(cx,H); ctx.stroke();
}
function drawPose(){
  if (!pose.have) return;
  const L = 40;
  const rad = -pose.yaw * Math.PI/180;
  const pts = [[-L/2, L/2], [0,-L/2], [L/2, L/2]];
  const cos = Math.cos(rad), sin = Math.sin(rad);
  let wx = [], wy = [];
  for (const [px,py] of pts) { wx.push(px*cos - py*sin + pose.x); wy.push(px*sin + py*cos + pose.y); }
  ctx.fillStyle = '#e7eaf0';
  ctx.beginPath();
  for (let i=0;i<3;i++){ const [sx,sy] = worldToScreen(wx[i], wy[i]); if (i===0) ctx.moveTo(sx,sy); else ctx.lineTo(sx,sy); }
  ctx.closePath(); ctx.fill();
}
function drawOccupancy(){
  const OCC_DRAW_THR = 2;
  for (const [kk,v] of occ){
    if (Math.abs(v) < OCC_DRAW_THR) continue;
    const [ixStr, iyStr] = kk.split(',');
    const ix = +ixStr, iy = +iyStr;
    const x_mm = (ix + 0.5) * CELL_MM;
    const y_mm = (iy + 0.5) * CELL_MM;
    const [sx, sy] = worldToScreen(x_mm, y_mm);
    const w = CELL_MM * scale, h = CELL_MM * scale;
    const a = Math.min(0.85, Math.abs(v) / OCC_MAX);
    ctx.fillStyle = (v >= 0) ? `rgba(255,100,100,${a})` : `rgba(100,160,255,${a})`;
    ctx.fillRect(Math.round(sx - w/2), Math.round(sy - h/2), Math.max(1,w), Math.max(1,h));
  }
}
function redraw(){
  syncVizSize();
  ctx.clearRect(0,0,W,H);
  drawGrid();
  drawAxes();
  drawOccupancy();
  drawPose();
}

function worldToCell(x_mm,y_mm){ return [ Math.floor(x_mm / CELL_MM), Math.floor(y_mm / CELL_MM) ]; }
function bresenham(ix0, iy0, ix1, iy1, includeLast=false){
  const pts = []; let dx = Math.abs(ix1 - ix0), sx = ix0 < ix1 ? 1 : -1;
  let dy = -Math.abs(iy1 - iy0), sy = iy0 < iy1 ? 1 : -1;
  let err = dx + dy, x = ix0, y = iy0;
  while (true){
    if (!(x === ix1 && y === iy1)) pts.push([x,y]); else { if (includeLast) pts.push([x,y]); break; }
    const e2 = 2*err; if (e2 >= dy) { err += dy; x += sx; } if (e2 <= dx) { err += dx; y += sy; }
  }
  return pts;
}

// Listen to WS events
on('pose', ({detail:p})=>{
  pose.have = true; pose.seq=p.seq; pose.t=p.t_ms;
  pose.x = p.y_mm; pose.y = p.x_mm; pose.yaw = p.yaw_d;
  redraw();
});
on('lidar', ({detail:l})=>{
  const rx = pose.x, ry = pose.y;
  const [rix, riy] = worldToCell(rx, ry);
  for (let i=0;i<l.pts;i++){
    const wx = l.yw[i], wy = l.xw[i]; // swapped for X-right, Y-up
    const [hix,hiy] = worldToCell(wx, wy);
    for (const [cx,cy] of bresenham(rix, riy, hix, hiy, false)) {
      const key = `${cx},${cy}`; occ.set(key, Math.max(OCC_MIN, (occ.get(key)||0) + (-1)));
    }
    const k2 = `${hix},${hiy}`; occ.set(k2, Math.min(OCC_MAX, (occ.get(k2)||0) + OCC_HIT));
  }
  redraw();
});

// Expose minimal API for the controls panel
window.__viz_setGrid = (v)=>{ showGrid = v; redraw(); };
window.__viz_setScale = (s)=>{ scale = s; redraw(); };

// First paint + resize
redraw();
window.addEventListener('resize', redraw);
