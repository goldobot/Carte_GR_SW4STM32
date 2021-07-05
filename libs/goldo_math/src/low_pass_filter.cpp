#include "goldobot/core/low_pass_filter.hpp"

#include "goldobot/core/math_utils.hpp"

#include <cmath>

namespace goldobot {

void LowPassFilter::setConfig(float period, float frequency) {
  m_coeffs[0] = expf(-period * frequency * c_pi * 2);
  m_coeffs[1] = (1.0f - m_coeffs[0]);
}

void LowPassFilter::reset(float value) { m_value = value; };

float LowPassFilter::step(float value) {
  m_value = m_value * m_coeffs[0] + value * m_coeffs[1];
  return m_value;
}

}  // namespace goldobot
