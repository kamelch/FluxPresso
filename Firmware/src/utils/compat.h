#pragma once

// Minimal compatibility layer so algorithmic code (PID, steam assist, etc.)
// can be unit-tested on PlatformIO's `native` platform (no Arduino SDK).

#ifdef ARDUINO
  #include <Arduino.h>
#else
  #include <cstdint>
  #include <algorithm>
  #include <type_traits>

  using uint8_t  = std::uint8_t;
  using uint16_t = std::uint16_t;
  using uint32_t = std::uint32_t;
  using int8_t   = std::int8_t;


// Arduino-like min/max/constrain that work with mixed integer types (e.g., uint8_t + int).
template <typename A, typename B>
static inline constexpr auto min(A a, B b) -> typename std::common_type<A,B>::type {
  using C = typename std::common_type<A,B>::type;
  return (C)a < (C)b ? (C)a : (C)b;
}

template <typename A, typename B>
static inline constexpr auto max(A a, B b) -> typename std::common_type<A,B>::type {
  using C = typename std::common_type<A,B>::type;
  return (C)a > (C)b ? (C)a : (C)b;
}

template <typename X, typename A, typename B>
static inline constexpr auto constrain(X x, A a, B b) -> typename std::common_type<X,A,B>::type {
  using C = typename std::common_type<X,A,B>::type;
  const C xc = (C)x;
  const C ac = (C)a;
  const C bc = (C)b;
  return min(max(xc, ac), bc);
}

  static inline uint32_t millis() {
    // Unit tests should pass explicit timestamps to controllers.
    // This stub exists only to satisfy code paths that *might* call millis().
    return 0;
  }

  // GPIO stubs (DreamSteam can optionally use a valve switch).
  static constexpr int INPUT_PULLUP = 0;

  static inline void pinMode(uint8_t, int) {
    // no-op in native tests
  }

  static inline bool digitalRead(uint8_t) {
    return false;
  }

#endif