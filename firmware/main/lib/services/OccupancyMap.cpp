#include "OccupancyMap.h"
#include <esp_heap_caps.h>
#include <stdlib.h>  // abs()

// ── Bresenham ray cast ─────────────────────────────────────────────────────────
// Traverses cells from (r0,c0) to (r1,c1) inclusive.
// Cells before the hit: FREE_DEC (confirms free space, erases dynamic obstacles).
// Hit cell: HIT_INC (accumulates evidence; real walls saturate, noise doesn't).
// Only marks dirty when the value actually changes (keeps delta packets small).
void OccupancyMap::markRay(float rx_mm, float ry_mm, float hx_mm, float hy_mm) {
  const int r0 = (int)((rx_mm - (float)ox_mm_) / CELL_MM);
  const int c0 = (int)((ry_mm - (float)oy_mm_) / CELL_MM);
  const int r1 = (int)((hx_mm - (float)ox_mm_) / CELL_MM);
  const int c1 = (int)((hy_mm - (float)oy_mm_) / CELL_MM);

  const int dr  = abs(r1 - r0);
  const int dc  = abs(c1 - c0);
  const int sr  = (r1 > r0) ? 1 : -1;
  const int sc  = (c1 > c0) ? 1 : -1;
  int err = dr - dc;
  int r = r0, c = c0;

  // Bresenham terminates in at most dr+dc steps; cap for safety.
  const int max_steps = dr + dc + 1;
  for (int step = 0; step <= max_steps; step++) {
    const bool is_hit = (r == r1 && c == c1);

    if ((unsigned)r < (unsigned)NY && (unsigned)c < (unsigned)NX) {
      const int    idx = r * NX + c;
      uint8_t&     v   = cells_[idx];
      const uint8_t old = v;
      if (is_hit) {
        v = (v + HIT_INC <= LOG_MAX) ? v + HIT_INC : LOG_MAX;
      } else {
        v = (v >= FREE_DEC) ? v - FREE_DEC : 0;
      }
      if (v != old) setDirty_(idx);
    }

    if (is_hit) break;

    const int e2 = 2 * err;
    if (e2 > -dc) { err -= dc; r += sr; }
    if (e2 <  dr) { err += dr; c += sc; }
  }
}

// ── Recenter ───────────────────────────────────────────────────────────────────
void OccupancyMap::recenter(float robot_x_mm, float robot_y_mm) {
  const int MARGIN = 8;
  int rrow = (int)((robot_x_mm - (float)ox_mm_) / CELL_MM);
  int rcol = (int)((robot_y_mm - (float)oy_mm_) / CELL_MM);
  if (rrow >= MARGIN && rrow < NY - MARGIN &&
      rcol >= MARGIN && rcol < NX - MARGIN) return;

  int32_t new_ox = (int32_t)(robot_x_mm) - (NY / 2) * CELL_MM;
  int32_t new_oy = (int32_t)(robot_y_mm) - (NX / 2) * CELL_MM;
  int     dr     = (int)((new_ox - ox_mm_) / CELL_MM);
  int     dc     = (int)((new_oy - oy_mm_) / CELL_MM);

  // PSRAM-backed static buffer — avoids 4 KB in internal BSS.
  // recenter() is only ever called from one task (net_tx), so no races.
  static uint8_t* tmp = (uint8_t*)heap_caps_malloc(NY * NX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!tmp) return;
  memset(tmp, 0, NY * NX);
  for (int r = 0; r < NY; r++) {
    for (int c = 0; c < NX; c++) {
      int sr = r + dr, sc = c + dc;
      if ((unsigned)sr < (unsigned)NY && (unsigned)sc < (unsigned)NX)
        tmp[r * NX + c] = cells_[sr * NX + sc];
    }
  }
  memcpy(cells_, tmp, NY * NX);
  ox_mm_ = new_ox;
  oy_mm_ = new_oy;

  originChanged_ = true;
  memset(dirty_, 0, sizeof(dirty_));
}
