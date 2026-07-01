// js/panels/joyL/joyL.js
// Left joystick panel: translation control (log/lat), servo sliders, and buzzer.
// Writes joystick output to window.__joyL for consumption by controls.js motion loop.
import { Joystick } from '../../shared/joystick.js';
import { sendJson } from '../../shared/ws.js';

const root = document.getElementById('panel-joyL');

// ── Left joystick (translation) ───────────────────────────────────────────────
window.__joyL = { log: 0, lat: 0 };

const canvas = document.getElementById('joyLCanvas');
new Joystick(canvas, {
  onInput(x, y) { window.__joyL = { lat: x, log: -y }; },
});

// ── Translation speed ─────────────────────────────────────────────────────────
const transSpeedRange = document.getElementById('transSpeedRange');
const transSpeedVal   = document.getElementById('transSpeedVal');
window.__transSpeed = +transSpeedRange.value;
transSpeedRange.oninput = () => {
  window.__transSpeed = +transSpeedRange.value;
  transSpeedVal.textContent = `${Math.round(window.__transSpeed * 100)}%`;
};

// ── Throttle helper ───────────────────────────────────────────────────────────
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

// ── Servos ────────────────────────────────────────────────────────────────────
const servo1Range = document.getElementById('servo1Range');
const servo1Val   = document.getElementById('servo1Val');
const servo2Range = document.getElementById('servo2Range');
const servo2Val   = document.getElementById('servo2Val');

const sendServo1 = throttle(deg => sendJson({ servo1: deg }), 40);
const sendServo2 = throttle(deg => sendJson({ servo2: deg }), 40);
servo1Range.oninput = () => { const d = Math.round(+servo1Range.value); servo1Val.textContent = `${d}`; sendServo1(d); };
servo2Range.oninput = () => { const d = Math.round(+servo2Range.value); servo2Val.textContent = `${d}`; sendServo2(d); };

// ── Buzzer ────────────────────────────────────────────────────────────────────
const buzzerPitchRange = document.getElementById('buzzerPitchRange');
const buzzerPitchVal   = document.getElementById('buzzerPitchVal');
const buzzerTimeRange  = document.getElementById('buzzerTimeRange');
const buzzerTimeVal    = document.getElementById('buzzerTimeVal');
const buzzBtn          = document.getElementById('buzzBtn');

buzzerPitchRange.oninput = () => { buzzerPitchVal.textContent = `${Math.round(+buzzerPitchRange.value)}`; };
buzzerTimeRange.oninput  = () => { buzzerTimeVal.textContent = (+buzzerTimeRange.value).toFixed(1); };

let buzzTimer = null;
buzzBtn.onclick = () => {
  if (buzzTimer) {
    clearTimeout(buzzTimer); buzzTimer = null;
    sendJson({ buzzer: 0 }); buzzBtn.textContent = 'Buzz';
    return;
  }
  const hz = Math.round(+buzzerPitchRange.value);
  const ms = Math.round(+buzzerTimeRange.value * 1000);
  sendJson({ buzzer: hz });
  buzzBtn.textContent = `Buzzing ${(+buzzerTimeRange.value).toFixed(1)} s`;
  buzzTimer = setTimeout(() => { buzzTimer = null; sendJson({ buzzer: 0 }); buzzBtn.textContent = 'Buzz'; }, ms);
};
