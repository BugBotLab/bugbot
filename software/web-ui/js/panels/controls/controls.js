import { sendControl } from '../../shared/ws.js';

const root = document.getElementById('panel-controls');

const speedRange = root.querySelector('#speedRange');
const speedVal   = root.querySelector('#speedVal');
const scaleRange = root.querySelector('#scaleRange');
const scaleVal   = root.querySelector('#scaleVal');
const toggleGrid = root.querySelector('#toggleGrid');
const clearMap   = root.querySelector('#clearMap');
const stopBtn    = root.querySelector('#stopBtn');

const DIR = { STOP:0, FWD:1, BACK:2, STRAFE_L:3, STRAFE_R:4, TURN_L:5, TURN_R:6 };

// wire speed / scale
speedRange.oninput = ()=> speedVal.textContent = (+speedRange.value).toFixed(2);
scaleRange.oninput = ()=> { const s=+scaleRange.value; scaleVal.textContent=s.toFixed(2); window.__viz_setScale?.(s); };

// grid + clear
let gridOn = true;
toggleGrid.onclick = ()=>{ gridOn = !gridOn; window.__viz_setGrid?.(gridOn); };
clearMap.onclick = ()=>{ /* simple trick: reload viz panel to clear map */
  const pane = document.getElementById('panel-viz');
  const html = pane.innerHTML; pane.innerHTML = html; // reset DOM -> breaks bindings
  import('./viz.js'); // re-hydrate
};

// drive buttons
root.querySelectorAll('[data-dir]').forEach(btn=>{
  btn.addEventListener('mousedown', ()=> sendControl(+btn.dataset.dir, +speedRange.value));
});
stopBtn.onclick = ()=> sendControl(DIR.STOP, 0);

// keyboard
const held = new Set();
const CONTROL_KEYS = new Set(['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','KeyZ','KeyX']);
function recomputeDir() {
  if (held.has('KeyZ')) return DIR.TURN_L;
  if (held.has('KeyX')) return DIR.TURN_R;
  if (held.has('ArrowLeft') ^ held.has('ArrowRight')) return held.has('ArrowLeft') ? DIR.STRAFE_L : DIR.STRAFE_R;
  if (held.has('ArrowUp') ^ held.has('ArrowDown')) return held.has('ArrowUp') ? DIR.FWD : DIR.BACK;
  return DIR.STOP;
}
let lastDir = DIR.STOP;
setInterval(()=>{
  const dir = recomputeDir(), spd = +speedRange.value;
  if (dir !== lastDir || dir !== DIR.STOP) { sendControl(dir, spd); lastDir=dir; }
}, 100);
window.addEventListener('keydown', e=>{
  if (e.code === 'Space' || e.code === 'KeyS') { sendControl(DIR.STOP, 0); return; }
  if (CONTROL_KEYS.has(e.code)) held.add(e.code);
  if (e.code >= 'Digit0' && e.code <= 'Digit9') {
    const raw = (e.code.charCodeAt(5) - 48) / 9.0;
    speedRange.value = raw.toFixed(2);
    speedVal.textContent = (+speedRange.value).toFixed(2);
  }
  if (e.code === 'KeyG') { gridOn=!gridOn; window.__viz_setGrid?.(gridOn); }
  if (e.code === 'KeyY') { clearMap.click(); }
});
window.addEventListener('keyup', e=>{ if (CONTROL_KEYS.has(e.code)) held.delete(e.code); });
