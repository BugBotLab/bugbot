export function defaultWsURL() {
  const host = "192.168.4.1"; // adjust if needed
  return `ws://${host}/ws`;
}
export function hostFromWs(urlStr) {
  try { const u = new URL(urlStr); return u.hostname; } catch { return ""; }
}
export function defaultCamURLFromWS(wsUrl) {
  const host = hostFromWs(wsUrl) || "192.168.4.1";
  return `http://${host}:81/mjpeg`;
}

// tiny event bus for cross-panel comms
const bus = new EventTarget();
export const on   = (t, fn) => bus.addEventListener(t, fn);
export const off  = (t, fn) => bus.removeEventListener(t, fn);
export const emit = (t, detail) => bus.dispatchEvent(new CustomEvent(t, { detail }));
