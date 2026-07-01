// js/shared/ws.js
// WebSocket client for the BugBot firmware WebSocket server (port 80, /ws).
// Handles connection, exponential-backoff reconnect, and binary packet parsing.
// All parsed packets are broadcast on window via emit() so any panel can subscribe.
//
// Packet types decoded here (see WebWS.h for byte layouts):
//   Pose    (20 bytes, no magic)         → emit('pose', {...})
//   3D scan (magic 0x5343)               → emit('scan3d', {...})
//   Map     (magic 0x4D41)               → emit('mapgrid', {...})
//   MapDelta(magic 0x4D44)               → emit('mapgrid', {...})  (delta applied to localMap)
//   Snapshot(magic 0x494D)               → emit('snapshot', {...})
//   LiDAR4/8(no magic, fixed sizes)      → emit('lidar', {...})
//   AprilTag(magic 0x4154)               → emit('apriltag', {...}) per detection
import { defaultWsURL, emit } from './utils.js';

const wsStatus  = document.getElementById('wsStatus');
const statsEl   = document.getElementById('stats');
const batteryEl = document.getElementById('batteryStatus');

let currentWsUrl = defaultWsURL();

let ws = null;
let reconnectTimer = null;
let countdownTimer = null;
let backoffMs = 1000;
const BO_MIN = 1000, BO_MAX = 15000, BO_FACTOR = 1.7;

let posePkts = 0, lidarPkts = 0, atPkts = 0;

// Persistent cell buffer for delta map updates — populated by full MAP, mutated by DMAP.
const localMap = { cells: null, nx: 0, ny: 0, origin_x: 0, origin_y: 0, cell_mm: 0 };
function resetLocalMap() { localMap.cells = null; }

function setWSState(connected, msg) {
  wsStatus.textContent = msg || (connected ? 'WS: connected' : 'WS: disconnected');
  wsStatus.classList.toggle('ok', connected);
  wsStatus.classList.toggle('bad', !connected);
}

function setBattery(volts) {
  if (!batteryEl) return;

  if (typeof volts !== 'number' || !isFinite(volts)) {
    batteryEl.textContent = 'BAT: --.- V';
    batteryEl.classList.remove('ok', 'warn', 'bad');
    return;
  }

  batteryEl.textContent = `BAT: ${volts.toFixed(2)} V`;
  batteryEl.classList.remove('ok', 'warn', 'bad');

  if (volts >= 3.8) batteryEl.classList.add('ok');
  else if (volts >= 3.5) batteryEl.classList.add('warn');
  else batteryEl.classList.add('bad');
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
  if (ws) {
    ws.onopen = null; ws.onclose = null; ws.onerror = null; ws.onmessage = null;
    try { ws.close(); } catch {}
  }
  ws = new WebSocket(currentWsUrl);
  ws.binaryType = 'arraybuffer';

  ws.onopen = ()=>{ localStorage.setItem('bugbot_ws_url', currentWsUrl); resetBackoff(); setWSState(true, 'WS: connected'); resetLocalMap(); };

  ws.onclose = ()=>{
    setWSState(false, 'WS: disconnected');
    setBattery(NaN);
    if (document.visibilityState !== 'hidden') scheduleReconnect('reconnecting');
  };

  ws.onerror = ()=>{
    setWSState(false, 'WS: error');
    setBattery(NaN);
  };

  ws.onmessage = (ev)=>{
    if (typeof ev.data === 'string') {
      try {
        const msg = JSON.parse(ev.data);

        if (msg.battery_volts !== undefined) {
          setBattery(msg.battery_volts);
        }
        if (msg.calib_status !== undefined) {
          emit('calib_status', msg);
        }
        if (msg.loc_sweep_status !== undefined) {
          emit('loc_sweep_status', msg);
        }
      } catch (_) {
        // ignore non-JSON text messages
      }
      return;
      }


    const buf = ev.data, n = buf.byteLength;

    const PKT_POSE     = 20;
    const HDR_A_SZ     = 12, LIDAR4 = HDR_A_SZ + 32, LIDAR8 = HDR_A_SZ + 64;
    const AT_MAGIC     = 0x4154, AT_HDR = 11, AT_HIT = 21;
    const SCAN3D_MAGIC = 0x5343, SCAN3D_HDR = 11;
    const MAP_MAGIC    = 0x4D41, MAP_HDR  = 33;
    const DMAP_MAGIC   = 0x4D44, DMAP_HDR = 35;
    const SNAP_MAGIC   = 0x494D, SNAP_HDR = 15;

    if (n === PKT_POSE){
      const dv = new DataView(buf);
      const pkt = { seq:dv.getUint32(0,true), t_ms:dv.getUint32(4,true), x_mm:dv.getFloat32(8,true), y_mm:dv.getFloat32(12,true), yaw_d:dv.getFloat32(16,true) };
      posePkts++; if (statsEl) statsEl.textContent = `pose ${posePkts} | lidar ${lidarPkts} | at ${atPkts}`;
      emit('pose', pkt);
      return;
    }
    // 3-D scan packet (magic 0x5343): header(11) + n×12 bytes
    if (n >= SCAN3D_HDR) {
      const dv3 = new DataView(buf);
      if (dv3.getUint16(0, true) === SCAN3D_MAGIC) {
        const npts = dv3.getUint8(10);
        const pts = [];
        for (let i = 0; i < npts; i++) {
          const base = SCAN3D_HDR + i * 12;
          if (base + 12 > n) break;
          pts.push({
            x: dv3.getFloat32(base,     true),
            y: dv3.getFloat32(base + 4, true),
            z: dv3.getFloat32(base + 8, true),
          });
        }
        emit('scan3d', { seq: dv3.getUint32(2, true), t_ms: dv3.getUint32(6, true), pts });
        lidarPkts++; if (statsEl) statsEl.textContent = `pose ${posePkts} | lidar ${lidarPkts} | at ${atPkts}`;
        return;
      }
    }

    // Full occupancy map packet — stores into localMap so delta packets can be applied
    if (n >= MAP_HDR) {
      const dvm = new DataView(buf);
      if (dvm.getUint16(0, true) === MAP_MAGIC) {
        const nx = dvm.getUint8(31), ny = dvm.getUint8(32);
        if (n >= MAP_HDR + nx * ny) {
          const origin_x = dvm.getInt32(22, true);
          const origin_y = dvm.getInt32(26, true);
          const cell_mm  = dvm.getUint8(30);
          if (!localMap.cells || localMap.cells.length !== nx * ny) {
            localMap.cells = new Uint8Array(nx * ny);
          }
          localMap.cells.set(new Uint8Array(buf, MAP_HDR, nx * ny));
          localMap.nx = nx; localMap.ny = ny;
          localMap.origin_x = origin_x; localMap.origin_y = origin_y;
          localMap.cell_mm  = cell_mm;
          emit('mapgrid', {
            seq: dvm.getUint32(2, true), t_ms: dvm.getUint32(6, true),
            robot_x: dvm.getFloat32(10, true), robot_y: dvm.getFloat32(14, true),
            robot_yaw: dvm.getFloat32(18, true),
            origin_x, origin_y, cell_mm, nx, ny,
            cells: localMap.cells,
          });
          lidarPkts++; if (statsEl) statsEl.textContent = `pose ${posePkts} | lidar ${lidarPkts} | at ${atPkts}`;
        }
        return;
      }
    }

    // Delta map packet — applies changed cells to localMap, then re-emits mapgrid
    if (n >= DMAP_HDR) {
      const dvd = new DataView(buf);
      if (dvd.getUint16(0, true) === DMAP_MAGIC) {
        const n_cells  = dvd.getUint16(33, true);
        const origin_x = dvd.getInt32(22, true);
        const origin_y = dvd.getInt32(26, true);
        if (n >= DMAP_HDR + n_cells * 3 && localMap.cells &&
            origin_x === localMap.origin_x && origin_y === localMap.origin_y) {
          for (let i = 0; i < n_cells; i++) {
            const base = DMAP_HDR + i * 3;
            localMap.cells[dvd.getUint8(base) * localMap.nx + dvd.getUint8(base + 1)] = dvd.getUint8(base + 2);
          }
          emit('mapgrid', {
            seq: dvd.getUint32(2, true), t_ms: dvd.getUint32(6, true),
            robot_x: dvd.getFloat32(10, true), robot_y: dvd.getFloat32(14, true),
            robot_yaw: dvd.getFloat32(18, true),
            origin_x, origin_y,
            cell_mm: localMap.cell_mm, nx: localMap.nx, ny: localMap.ny,
            cells: localMap.cells,
          });
          lidarPkts++; if (statsEl) statsEl.textContent = `pose ${posePkts} | lidar ${lidarPkts} | at ${atPkts}`;
        }
        return;
      }
    }

    // Snapshot packet — check before LIDAR4/8 (those have no magic guard)
    if (n >= SNAP_HDR) {
      const dvs = new DataView(buf);
      if (dvs.getUint16(0, true) === SNAP_MAGIC) {
        const img_len = dvs.getUint32(11, true);
        if (n >= SNAP_HDR + img_len) {
          const jpeg = new Uint8Array(buf, SNAP_HDR, img_len);
          const blob = new Blob([jpeg], { type: 'image/jpeg' });
          emit('snapshot', {
            seq:    dvs.getUint32(2, true),
            t_ms:   dvs.getUint32(6, true),
            tag_id: dvs.getUint8(10),
            blob,
          });
        }
        return;
      }
    }

    if (n === LIDAR4 || n === LIDAR8){
      const dv = new DataView(buf);
      const pts = (n===LIDAR4)?4:8;
      const arrF = new Float32Array(buf, HDR_A_SZ, pts*2);
      const pkt = { seq:dv.getUint32(0,true), t_ms:dv.getUint32(4,true), row:dv.getUint8(8), vmask:dv.getUint8(9), hz:dv.getUint16(10,true), xw:arrF.slice(0,pts), yw:arrF.slice(pts,pts*2), pts };
      lidarPkts++; if (statsEl) statsEl.textContent = `pose ${posePkts} | lidar ${lidarPkts} | at ${atPkts}`;
      emit('lidar', pkt);
      return;
    }
    // AprilTag packet: [uint16 magic=0x4154][uint32 seq][uint32 t_ms][uint8 count]
    //                  [per hit: uint8 id, float cx_px, cy_px, az_deg, el_deg, dist_mm]
    if (n >= AT_HDR) {
      const dv = new DataView(buf);
      if (dv.getUint16(0, true) === AT_MAGIC) {
        const t_ms = dv.getUint32(6, true);
        const count = dv.getUint8(10);
        for (let i = 0; i < count; i++) {
          const base = AT_HDR + i * AT_HIT;
          if (base + AT_HIT > n) break;
          emit('apriltag', {
            id:     dv.getUint8(base),
            cx:     dv.getFloat32(base + 1,  true),
            cy:     dv.getFloat32(base + 5,  true),
            azDeg:  dv.getFloat32(base + 9,  true),
            elDeg:  dv.getFloat32(base + 13, true),
            dist_m: dv.getFloat32(base + 17, true) / 1000,
            W: 160, H: 120,
            t_ms,
          });
        }
        atPkts++; if (statsEl) statsEl.textContent = `pose ${posePkts} | lidar ${lidarPkts} | at ${atPkts}`;
        return;
      }
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

function clampSigned(v) {
  const n = Number(v);
  if (!Number.isFinite(n)) return 0;
  return Math.max(-1, Math.min(1, n));
}

export function sendMotionVector(log, lat, rot) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return false;
  // Use the new firmware text protocol for easy testing/debugging.
  ws.send(JSON.stringify({ log: clampSigned(log), lat: clampSigned(lat), rot: clampSigned(rot) }));
  return true;
}

export function sendText(text) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return false;
  ws.send(text);
  return true;
}

export function sendJson(obj) {
  return sendText(JSON.stringify(obj));
}

function disconnectWS() {
  clearTimers();
  if (ws) {
    ws.onopen = null; ws.onclose = null; ws.onerror = null; ws.onmessage = null;
    try { ws.close(); } catch {}
    ws = null;
  }
  setWSState(false, 'WS: disconnected');
  setBattery(NaN);
}

// wire up
document.addEventListener('visibilitychange', () => {
  if (document.visibilityState === 'hidden') {
    disconnectWS();
  } else {
    resetBackoff(); connectWS();
  }
});
window.addEventListener('online',  () => { resetBackoff(); connectWS(); });
window.addEventListener('offline', () => disconnectWS());

// autostart
connectWS();
