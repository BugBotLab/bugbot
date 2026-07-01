// js/shared/utils.js
// Shared utilities used across all UI panels.
// Provides a lightweight cross-panel event bus (on/emit/off) backed by
// window.dispatchEvent so events are visible to all module scopes.
// Also exports URL helpers and common math utilities.

export function defaultWsURL() {
  const saved = localStorage.getItem('bugbot_ws_url');
  if (saved) return saved;
  // When served from the robot itself, auto-derive WS URL from the page host.
  const host = window.location.hostname;
  if (host && host !== 'localhost' && host !== '127.0.0.1') {
    return `ws://${host}/ws`;
  }
  return 'ws://bugbot.local/ws';
}
export function hostFromWs(urlStr) {
  try { const u = new URL(urlStr); return u.hostname; } catch { return ""; }
}
export function defaultCamURLFromWS(wsUrl) {
  const host = hostFromWs(wsUrl) || "bugbot.local";
  return `http://${host}:81/mjpeg`;
}

// Cross-panel event bus: all three functions use window so listeners and emitters
// are in the same scope regardless of which module registers them.
export const on   = (type, handler) => window.addEventListener(type, handler);
export const off  = (type, handler) => window.removeEventListener(type, handler);
export const emit = (type, detail)  => window.dispatchEvent(new CustomEvent(type, { detail }));

export const clamp  = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
export const rad2deg = r => (r * 180) / Math.PI;
export const deg2rad = d => (d * Math.PI) / 180;

export const nowMs = () => performance.now();
