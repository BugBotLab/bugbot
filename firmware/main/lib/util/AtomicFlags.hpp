// AtomicFlags.hpp
// Lock-free boolean (AtomicFlag) and bitmask (AtomicBits) types wrapping std::atomic.
// Flags:: constants are the system-wide status bits shared between services.
#pragma once
#include <Arduino.h>
#include <atomic>
class AtomicFlag {
public:
  explicit AtomicFlag(bool v=false) : v_(v) {}
  inline void set()                 { v_.store(true,  std::memory_order_release); }
  inline void clear()               { v_.store(false, std::memory_order_release); }
  inline bool test()          const { return v_.load(std::memory_order_acquire); }
  inline bool testAndClear()        { return v_.exchange(false, std::memory_order_acq_rel); }
private:
  std::atomic<bool> v_;
};
class AtomicBits {
public:
  explicit AtomicBits(uint32_t init=0) : bits_(init) {}
  inline void set(uint32_t mask)         { bits_.fetch_or(mask,  std::memory_order_acq_rel); }
  inline void clear(uint32_t mask)       { bits_.fetch_and(~mask, std::memory_order_acq_rel); }
  inline bool any(uint32_t mask) const   { return (bits_.load(std::memory_order_acquire) & mask) != 0; }
  inline uint32_t value() const          { return bits_.load(std::memory_order_acquire); }
  inline uint32_t exchange(uint32_t v)   { return bits_.exchange(v, std::memory_order_acq_rel); }
private:
  std::atomic<uint32_t> bits_;
};
namespace Flags {
  constexpr uint32_t WIFI_STA_OK = 1u << 0;
  constexpr uint32_t WIFI_AP_ON  = 1u << 1;
  constexpr uint32_t LIDAR_OK    = 1u << 2;
  constexpr uint32_t MOTION_RDY  = 1u << 3;
}
