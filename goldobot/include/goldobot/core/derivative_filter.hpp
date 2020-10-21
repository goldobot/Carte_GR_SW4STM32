#pragma once

namespace goldobot {

// A derivative filter
// A derivative of x sampled at a fixed period can be computed as d = (x - x_prev) / period
// To reduce noise, it is filtered by a first order low pass filter

class DerivativeFilter {
 public:
  void setConfig(float period, float frequency);
  void reset(float value);
  float step(float delta);

 private:
  float m_coeffs[2]{0, 0};
  float m_value;
};
}  // namespace goldobot
