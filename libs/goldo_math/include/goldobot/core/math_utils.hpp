#pragma once

namespace goldobot {
constexpr double c_pi = 3.14159265358979323846264338327950288;

template <typename T>
constexpr T clamp(T val, T min, T max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

template <typename T>
constexpr T clampAngle(T angle) {
  while (angle > c_pi) {
    angle -= static_cast<T>(2 * c_pi);
  };
  while (angle < -c_pi) {
    angle += static_cast<T>(2 * c_pi);
  }
  return angle;
}

template <typename T>
inline float angleDiff(T a, T b) {
  T diff = a - b;
  return clampAngle(diff);
}
}  // namespace goldobot
