#pragma once

// Minimal compatibility layer so algorithmic code (PID, steam assist, etc.)
// can be unit-tested on PlatformIO's `native` platform (no Arduino SDK).

#ifdef ARDUINO
  #include <Arduino.h>
#else
  #include <cstdint>
  #include <algorithm>

  using uint8_t  = std::uint8_t;
  using uint16_t = std::uint16_t;
  using uint32_t = std::uint32_t;
  using int8_t   = std::int8_t;

  template <typename T>
  static inline T constrain(T x, T a, T b) {
    return std::min(std::max(x, a), b);
  }

  static inline uint32_t millis() {
    // Unit tests should pass explicit timestamps to controllers.
    // This stub exists only to satisfy code paths that *might* call millis().
    return 0;
  }
#endif
