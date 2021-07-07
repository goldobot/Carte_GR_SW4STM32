#pragma once

namespace goldobot {

// A simple low pass filter
// A derivative of x sampled at a fixed period can be computed as d = (x - x_prev) / period
// To reduce noise, it is filtered by a first order low pass filter

class LowPassFilter {
 public:
  void setConfig(float period, float frequency);
  void reset(float value);
  float step(float value);
  float value() const noexcept {return m_value;};

 private:
  float m_coeffs[2]{0, 0};
  float m_value;
};
}  // namespace goldobot
