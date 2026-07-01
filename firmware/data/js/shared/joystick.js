// js/shared/joystick.js
// Reusable virtual analog joystick backed by a canvas element.
export class Joystick {
  constructor(canvas, { onInput } = {}) {
    this.canvas  = canvas;
    this.onInput = onInput;
    this.x = 0;
    this.y = 0;
    this._ptId = null;

    canvas.addEventListener('pointerdown',   e => this._down(e));
    canvas.addEventListener('pointermove',   e => this._move(e));
    canvas.addEventListener('pointerup',     e => this._up(e));
    canvas.addEventListener('pointercancel', e => this._up(e));

    new ResizeObserver(() => this._resize()).observe(canvas);
    this._resize();
    requestAnimationFrame(() => this._frame());
  }

  _resize() {
    const c = this.canvas;
    c.width  = c.clientWidth  || 200;
    c.height = c.clientHeight || 200;
  }

  _R()  { return Math.min(this.canvas.width, this.canvas.height) * 0.42; }
  _cx() { return this.canvas.width  / 2; }
  _cy() { return this.canvas.height / 2; }

  _toCanvas(e) {
    const r = this.canvas.getBoundingClientRect();
    return [
      (e.clientX - r.left) * (this.canvas.width  / r.width),
      (e.clientY - r.top)  * (this.canvas.height / r.height),
    ];
  }

  _apply(e) {
    const [px, py] = this._toCanvas(e);
    const R = this._R();
    let dx = px - this._cx(), dy = py - this._cy();
    const d = Math.hypot(dx, dy);
    if (d > R) { dx = dx / d * R; dy = dy / d * R; }
    this.x = dx / R;
    this.y = dy / R;
    this.onInput?.(this.x, this.y);
  }

  _down(e) {
    e.preventDefault();
    if (this._ptId !== null) return;
    this._ptId = e.pointerId;
    this.canvas.setPointerCapture(e.pointerId);
    this._apply(e);
  }

  _move(e) { if (e.pointerId === this._ptId) this._apply(e); }

  _up(e) {
    if (e.pointerId !== this._ptId) return;
    this._ptId = null;
    this.x = 0; this.y = 0;
    this.onInput?.(0, 0);
  }

  _frame() { this._draw(); requestAnimationFrame(() => this._frame()); }

  _draw() {
    const { canvas, x, y, _ptId: ptId } = this;
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    const cx = this._cx(), cy = this._cy();
    const R  = this._R();

    ctx.clearRect(0, 0, W, H);

    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, Math.PI * 2);
    ctx.fillStyle   = 'rgba(162,197,148,0.18)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(90,138,106,0.40)';
    ctx.lineWidth   = 2;
    ctx.stroke();

    ctx.strokeStyle = 'rgba(90,138,106,0.20)';
    ctx.lineWidth   = 1;
    ctx.beginPath();
    ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
    ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R);
    ctx.stroke();

    const kx = cx + x * R;
    const ky = cy + y * R;
    const kr = R * 0.30;

    ctx.beginPath();
    ctx.arc(kx, ky, kr, 0, Math.PI * 2);
    ctx.fillStyle   = ptId !== null ? 'rgba(60,130,80,0.82)' : 'rgba(90,150,110,0.48)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(40,100,60,0.65)';
    ctx.lineWidth   = 1.5;
    ctx.stroke();
  }
}
