// js/panels/controls/controls.js
// Right joystick (rotation), LED, field-centric toggle, keyboard and gamepad input.
// Runs a 50 Hz motion loop that combines joystick + keyboard + gamepad into a
// single sendMotionVector() call with heartbeat to prevent firmware teleop timeout.
import { sendJson, sendMotionVector } from '../../shared/ws.js';
import { Joystick } from '../../shared/joystick.js';
import { on } from '../../shared/utils.js';

const root = document.getElementById('panel-controls');

// ── Right joystick (rotation) ─────────────────────────────────────────────────
const joyRCanvas = root.querySelector('#joyRCanvas');
let joyRotX = 0;

new Joystick(joyRCanvas, {
  onInput(x, _y) { joyRotX = -x; },
});

// ── Field-centric toggle ──────────────────────────────────────────────────────
const fcBtn = document.getElementById('fcBtn');
let fieldCentric = true;

fcBtn?.addEventListener('click', () => {
  fieldCentric = !fieldCentric;
  fcBtn.textContent = fieldCentric ? 'FC: ON' : 'FC: OFF';
  fcBtn.classList.toggle('ok', fieldCentric);
  sendJson({ field_centric: fieldCentric, heading_hold_enabled: fieldCentric });
});

// ── Zero / reset pose ─────────────────────────────────────────────────────────
const zeroBtn = document.getElementById('zeroBtn');
zeroBtn?.addEventListener('click', () => {
  sendJson({ cmd: 'reset_pose' });
});

// ── Kinematic calibration ─────────────────────────────────────────────────────
const calibBtn    = document.getElementById('calibBtn');
const calibStatus = document.getElementById('calibStatus');
let calibActive   = false;
let calibDoneTimer = null;

function setCalibState(state, detail) {
  calibActive = (state === 'running');
  if (calibDoneTimer) { clearTimeout(calibDoneTimer); calibDoneTimer = null; }
  if (calibBtn) {
    calibBtn.textContent = calibActive ? 'CANCEL CAL' : 'CALIBRATE';
    calibBtn.classList.toggle('bad', calibActive);
  }
  if (calibStatus) {
    if (state === 'running') {
      calibStatus.textContent = detail || 'running…';
      calibStatus.style.color = '#fa0';
    } else if (state === 'done') {
      calibStatus.textContent = 'saved ✓';
      calibStatus.style.color = '#4f4';
      calibDoneTimer = setTimeout(() => { calibStatus.textContent = 'idle'; calibStatus.style.color = '#888'; }, 4000);
    } else if (state === 'error') {
      calibStatus.textContent = detail || 'failed';
      calibStatus.style.color = '#f44';
      calibDoneTimer = setTimeout(() => { calibStatus.textContent = 'idle'; calibStatus.style.color = '#888'; }, 4000);
    } else {
      calibStatus.textContent = 'idle';
      calibStatus.style.color = '#888';
    }
  }
}

calibBtn?.addEventListener('click', () => {
  if (calibActive) {
    sendJson({ cmd: 'calibrate', cancel: true });
    setCalibState('idle');
  } else {
    sendJson({ cmd: 'calibrate' });
    setCalibState('running');
  }
});

on('calib_status', (e) => {
  const s = e.detail;
  if (s.done)  setCalibState('done');
  else if (s.error) setCalibState('error', s.error);
  else if (s.step)  setCalibState('running', s.step);
});

// ── Localization sweep ────────────────────────────────────────────────────────
const locSweepBtn    = document.getElementById('locSweepBtn');
const locSweepStatus = document.getElementById('locSweepStatus');
let sweepActive = false;
let sweepDoneTimer = null;

function setSweepState(state) {
  sweepActive = (state === 'active');
  if (sweepDoneTimer) { clearTimeout(sweepDoneTimer); sweepDoneTimer = null; }
  if (locSweepBtn) {
    locSweepBtn.textContent = sweepActive ? 'STOP SWEEP' : 'LOC SWEEP';
    locSweepBtn.classList.toggle('bad', sweepActive);
  }
  if (locSweepStatus) {
    if (state === 'active') {
      locSweepStatus.textContent = 'scanning…';
      locSweepStatus.style.color = '#fa0';
    } else if (state === 'converged') {
      locSweepStatus.textContent = 'converged ✓';
      locSweepStatus.style.color = '#4f4';
      sweepDoneTimer = setTimeout(() => { locSweepStatus.textContent = 'idle'; locSweepStatus.style.color = '#888'; }, 4000);
    } else if (state === 'no_fix') {
      locSweepStatus.textContent = 'no fix — face a tag';
      locSweepStatus.style.color = '#f84';
      sweepDoneTimer = setTimeout(() => { locSweepStatus.textContent = 'idle'; locSweepStatus.style.color = '#888'; }, 6000);
    } else {
      locSweepStatus.textContent = 'idle';
      locSweepStatus.style.color = '#888';
    }
  }
}

locSweepBtn?.addEventListener('click', () => {
  if (sweepActive) {
    sendJson({ cmd: 'stop_loc_sweep' });
    setSweepState('idle');
  } else {
    sendJson({ cmd: 'start_loc_sweep' });
    setSweepState('active');
  }
});

on('loc_sweep_status', (e) => setSweepState(e.detail.loc_sweep_status));

// ── LED ───────────────────────────────────────────────────────────────────────
const ledRRange    = document.getElementById('ledRRange');
const ledRVal      = document.getElementById('ledRVal');
const ledGRange    = document.getElementById('ledGRange');
const ledGVal      = document.getElementById('ledGVal');
const ledBRange    = document.getElementById('ledBRange');
const ledBVal      = document.getElementById('ledBVal');
const ledToggleBtn = document.getElementById('ledToggleBtn');

function throttle(fn, waitMs = 40) {
  let last = 0, timer = null, pending = null;
  return (...args) => {
    pending = args;
    const run = () => { last = performance.now(); timer = null; fn(...pending); };
    const elapsed = performance.now() - last;
    if (elapsed >= waitMs) run();
    else if (!timer) timer = setTimeout(run, waitMs - elapsed);
  };
}

let ledEnabled = false;
function updateLedLabels() {
  ledRVal.textContent = `${Math.round(+ledRRange.value)}`;
  ledGVal.textContent = `${Math.round(+ledGRange.value)}`;
  ledBVal.textContent = `${Math.round(+ledBRange.value)}`;
}
function applyLed() {
  if (ledEnabled) {
    sendJson({ led: { r: Math.round(+ledRRange.value), g: Math.round(+ledGRange.value), b: Math.round(+ledBRange.value) } });
    ledToggleBtn.textContent = 'LED ON';
  } else {
    sendJson({ led: { r: 0, g: 0, b: 0 } });
    ledToggleBtn.textContent = 'LED OFF';
  }
}
const sendLed = throttle(() => { updateLedLabels(); if (ledEnabled) applyLed(); }, 40);
[ledRRange, ledGRange, ledBRange].forEach(el => { el.oninput = () => sendLed(); });
ledToggleBtn.onclick = () => { ledEnabled = !ledEnabled; applyLed(); };
updateLedLabels();

// ── Rotation speed ────────────────────────────────────────────────────────────
const rotSpeedRange = document.getElementById('rotSpeedRange');
const rotSpeedVal   = document.getElementById('rotSpeedVal');
rotSpeedRange.oninput = () => {
  rotSpeedVal.textContent = `${Math.round(+rotSpeedRange.value * 100)}%`;
};

// ── Keyboard ──────────────────────────────────────────────────────────────────
const held = new Set();
const MOVE_KEYS = new Set(['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'KeyZ', 'KeyX']);

function kbMotion() {
  let log = 0, lat = 0, rot = 0;
  if (held.has('ArrowUp'))    log += 1;
  if (held.has('ArrowDown'))  log -= 1;
  if (held.has('ArrowRight')) lat += 1;
  if (held.has('ArrowLeft'))  lat -= 1;
  if (held.has('KeyZ'))       rot -= 1;
  if (held.has('KeyX'))       rot += 1;
  const mag = Math.hypot(log, lat);
  if (mag > 1) { log /= mag; lat /= mag; }
  return { log, lat, rot };
}

let lastSent = { log: 999, lat: 999, rot: 999 };
let lastSentMs = 0;
const HEARTBEAT_MS = 150;  // must stay below firmware teleopTimeoutMs (500 ms)

window.addEventListener('keydown', e => {
  if (e.code === 'Space' || e.code === 'KeyS') {
    held.clear();
    sendMotionVector(0, 0, 0);
    lastSent = { log: 0, lat: 0, rot: 0 };
    return;
  }
  if (MOVE_KEYS.has(e.code)) held.add(e.code);
  if (e.code === 'KeyY') { window.__viz_clearMap?.(); window.__viz_clearTags?.(); }
});
window.addEventListener('keyup', e => { if (MOVE_KEYS.has(e.code)) held.delete(e.code); });

// ── Gamepad ───────────────────────────────────────────────────────────────────
const gpStatus = document.getElementById('gpStatus');
let gpEnabled = false;
let gpIndex   = null;

function setGpStatus(on) {
  if (!gpStatus) return;
  gpStatus.textContent = on ? 'GP: ON' : 'GP: none';
  gpStatus.classList.toggle('ok', on);
}

function shapeAxis(v, dead = 0.16) {
  const a = Math.abs(v ?? 0);
  if (a <= dead) return 0;
  return Math.sign(v) * ((a - dead) / (1 - dead));
}

function getTurnAxis(gp) {
  for (const v of [gp.axes?.[2], gp.axes?.[3], gp.axes?.[4]]) {
    const s = shapeAxis(v ?? 0);
    if (Math.abs(s) > 0) return s;
  }
  const lt = gp.buttons?.[6]?.value ?? 0;
  const rt = gp.buttons?.[7]?.value ?? 0;
  const lb = gp.buttons?.[4]?.pressed ? 1 : 0;
  const rb = gp.buttons?.[5]?.pressed ? 1 : 0;
  return shapeAxis((rb + rt) - (lb + lt), 0.05);
}

function readGp() {
  const pads = navigator.getGamepads?.() ?? [];
  if (gpIndex !== null && pads[gpIndex]) return pads[gpIndex];
  return [...pads].find(Boolean) ?? null;
}

window.addEventListener('gamepadconnected', e => {
  gpIndex = e.gamepad.index;
  gpEnabled = true;
  setGpStatus(true);
});
window.addEventListener('gamepaddisconnected', e => {
  if (gpIndex !== e.gamepad.index) return;
  const fallback = [...(navigator.getGamepads?.() ?? [])].find((p, i) => p && i !== e.gamepad.index);
  if (fallback) { gpIndex = fallback.index; }
  else { gpEnabled = false; gpIndex = null; setGpStatus(false); }
});

setGpStatus(false);

// ── Motion loop (50 Hz) ───────────────────────────────────────────────────────
function clamp(v) { return Math.max(-1, Math.min(1, v)); }

setInterval(() => {
  let log = 0, lat = 0, rot = 0;

  if (gpEnabled) {
    const gp = readGp();
    if (gp && !gp.buttons?.[0]?.pressed) {
      lat = shapeAxis(gp.axes?.[0] ?? 0);
      log = -shapeAxis(gp.axes?.[1] ?? 0);
      rot = getTurnAxis(gp);
    }
  } else {
    const L  = window.__joyL ?? { log: 0, lat: 0 };
    const kb = kbMotion();
    const ts = window.__transSpeed ?? 1.0;
    const rs = +rotSpeedRange.value;
    log = clamp((L.log + kb.log) * ts);
    lat = clamp((L.lat + kb.lat) * ts);
    rot = clamp((joyRotX + kb.rot) * rs);
  }

  const now = performance.now();
  const dirty = Math.abs(log - lastSent.log) > 0.005 ||
                Math.abs(lat - lastSent.lat) > 0.005 ||
                Math.abs(rot - lastSent.rot) > 0.005;
  if (dirty || (now - lastSentMs) >= HEARTBEAT_MS) {
    if (sendMotionVector(log, lat, rot)) {
      lastSent = { log, lat, rot };
      lastSentMs = now;
    }
  }
}, 20);
