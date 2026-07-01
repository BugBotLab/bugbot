#pragma once
#include <stdint.h>
#include <string.h>

// 64 × 64 log-odds occupancy grid, CELL_MM mm/cell → 3.2 m × 3.2 m coverage.
// Index: row = x_mm bucket (forward axis), col = y_mm bucket (lateral axis).
// Flat layout: cells_[row * NX + col].
//
// Log-odds encoding (uint8, unsigned):
//   0         = unobserved / confirmed free (not rendered by client)
//   1..LOG_MAX = occupied evidence (higher = more confirmed)
//
// markRay() traces each ToF beam with Bresenham:
//   free cells along the ray → -= FREE_DEC (dynamic obstacles erase over time)
//   hit cell at end of ray   → += HIT_INC  (clamp to LOG_MAX)
// Walls accumulate hits and never receive free decrements (the beam ends there).
// Moving objects accumulate hits but then receive free decrements once they move,
// clearing in roughly (peak_value / FREE_DEC) subsequent scans.
class OccupancyMap {
public:
  static constexpr int     NX       = 64;
  static constexpr int     NY       = 64;
  static constexpr int     CELL_MM  = 50;

  // Log-odds update weights — tuned so a wall saturates in ~20 hits and a
  // dynamic obstacle with 10 hits clears in ~25 subsequent free-ray passes.
  static constexpr uint8_t LOG_MAX  = 100;  // cell value cap
  static constexpr uint8_t HIT_INC  = 5;   // added per confirmed hit
  static constexpr uint8_t FREE_DEC = 2;   // removed per free-ray pass through cell

  OccupancyMap() { clear(); }

  // Trace a ray from the robot to the hit point using Bresenham.
  // Cells along the ray get FREE_DEC subtracted (erases stale obstacles).
  // The hit cell gets HIT_INC added (confirms walls).
  void markRay(float rx_mm, float ry_mm, float hx_mm, float hy_mm);

  void recenter(float robot_x_mm, float robot_y_mm);

  void clear() {
    memset(cells_, 0, sizeof(cells_));
    memset(dirty_, 0, sizeof(dirty_));
    originChanged_ = false;
  }

  int32_t        originX() const { return ox_mm_; }
  int32_t        originY() const { return oy_mm_; }
  const uint8_t* cells()   const { return cells_; }

  // Dirty-cell tracking — set in markRay(), cleared after delta send
  bool isDirty(int row, int col) const {
    const int idx = row * NX + col;
    return (dirty_[idx >> 3] >> (idx & 7)) & 1;
  }
  bool anyDirty() const {
    for (int i = 0; i < (int)sizeof(dirty_); i++)
      if (dirty_[i]) return true;
    return false;
  }
  void clearDirty() { memset(dirty_, 0, sizeof(dirty_)); }

  // Set after recenter() shifts the origin; cleared after a full-map send
  bool originChanged()      const { return originChanged_; }
  void clearOriginChanged()       { originChanged_ = false; }
  void markOriginChanged()        { originChanged_ = true; }

private:
  void setDirty_(int idx) { dirty_[idx >> 3] |= (1u << (idx & 7)); }

  int32_t ox_mm_ = 0, oy_mm_ = 0;
  uint8_t cells_[NY * NX]{};
  uint8_t dirty_[(NY * NX + 7) / 8]{};
  bool    originChanged_ = false;
};
