import { defaultWsURL, emit } from './utils.js';

const wsStatus = document.getElementById('wsStatus');
const statsEl  = document.getElementById('stats');
const wsUrlEl  = document.getElementById('wsUrl');
const connectBtn = document.getElementById('connectBtn');

wsUrlEl.value = defaultWsURL();

let ws = null;
let reconnectTimer = null;
let countdownTimer = null;
let backoffMs = 1000;
const BO_MIN = 1000, BO_MAX = 15000, BO_FACTOR = 1.7;

let posePkts = 0, lidarPkts = 0;

function setWSState(connected, msg) {
  wsStatus.textContent = msg || (connected ? 'WS: connected' : 'WS: disconnected');
  wsStatus.classList.toggle('ok', connected);
  wsStatus.classList.toggle('bad', !connected);
}
function clearTimers() { if (reconnectTimer) clearTimeout(reconnectTimer); reconnectTimer=null; if (countdownTimer) clearInterval(countdownTimer); countdownTimer=null; }
function resetBackoff(){ backoffMs = BO_MIN; }
function scheduleReconnect(reason='reconnecting'){
  clearTimers();
  const jitter = backoffMs * (0.2 * (Math.random()*2 - 1));
  const delay = Math.max(BO_MIN, Math.min(BO_MAX, Math.round(backoffMs + jitter)));
  let secsLeft = Math.ceil(delay/1000);
  setWSState(false, `WS: ${reason} in ${secsLeft}s`);
  countdownTimer = setInterval(()=>{ secsLeft--; if(secsLeft<=0){clearInterval(countdownTimer); countdownTimer=null;} else setWSState(false, `WS: ${reason} in ${secsLeft}s`); }, 1000);
  reconnectTimer = setTimeout(()=>{ reconnectTimer=null; connectWS(); }, delay);
  backoffMs = Math.min(BO_MAX, Math.max(BO_MIN, Math.round(backoffMs*BO_FACTOR)));
}

export function connectWS(){
  clearTimers();
  try { if (ws) ws.close(); } catch {}
  ws = new WebSocket(wsUrlEl.value.trim());
  ws.binaryType = 'arraybuffer';

  ws.onopen = ()=>{ resetBackoff(); setWSState(true, 'WS: connected'); };
  ws.onclose = ()=>{ setWSState(false, 'WS: disconnected'); if (document.visibilityState==='hidden') return; scheduleReconnect('reconnecting'); };
  ws.onerror = ()=> setWSState(false, 'WS: error');

  ws.onmessage = (ev)=>{
    if (typeof ev.data === 'string') return;
    const buf = ev.data, n = buf.byteLength;

    const PKT_POSE = 20, HDR_A_SZ=12, LIDAR4=HDR_A_SZ+32, LIDAR8=HDR_A_SZ+64;

    if (n === PKT_POSE){
      const dv = new DataView(buf);
      const pkt = { seq:dv.getUint32(0,true), t_ms:dv.getUint32(4,true), x_mm:dv.getFloat32(8,true), y_mm:dv.getFloat32(12,true), yaw_d:dv.getFloat32(16,true) };
      posePkts++; statsEl.textContent = `pose ${posePkts} | lidar ${lidarPkts}`;
      emit('pose', pkt);
      return;
    }
    if (n === LIDAR4 || n === LIDAR8){
      const dv = new DataView(buf);
      const pts = (n===LIDAR4)?4:8;
      const arrF = new Float32Array(buf, HDR_A_SZ, pts*2);
      const pkt = { seq:dv.getUint32(0,true), t_ms:dv.getUint32(4,true), row:dv.getUint8(8), vmask:dv.getUint8(9), hz:dv.getUint16(10,true), xw:arrF.slice(0,pts), yw:arrF.slice(pts,pts*2), pts };
      lidarPkts++; statsEl.textContent = `pose ${posePkts} | lidar ${lidarPkts}`;
      emit('lidar', pkt);
      return;
    }
  };
}

export function sendControl(dir, speed) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  if (typeof sendControl._seq !== 'number') sendControl._seq = 0;
  const CTRL_MAGIC = 0x4242;
  const buf = new ArrayBuffer(2+4+1+4);
  const dv = new DataView(buf);
  dv.setUint16(0, CTRL_MAGIC, true);
  dv.setUint32(2, (sendControl._seq++ & 0xFFFF), true);
  dv.setUint8(6, dir);
  dv.setFloat32(7, Math.max(0, Math.min(1, speed)), true);
  ws.send(buf);
}

// wire up
connectBtn.addEventListener('click', ()=>{ resetBackoff(); connectWS(); });
document.addEventListener('visibilitychange', ()=>{ if (document.visibilityState==='visible') { resetBackoff(); connectWS(); }});
window.addEventListener('online', ()=>{ resetBackoff(); connectWS(); });
window.addEventListener('offline', ()=> setWSState(false, 'WS: offline'));

// autostart
connectWS();
