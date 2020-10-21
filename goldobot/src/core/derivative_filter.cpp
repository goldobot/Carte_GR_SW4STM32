#include "goldobot/core/derivative_filter.hpp"

#include "goldobot/core/math_utils.hpp"

#include <cmath>

namespace goldobot {

void DerivativeFilter::setConfig(float period, float frequency) {
  m_coeffs[0] = expf(-period * frequency * c_pi * 2);
  m_coeffs[1] = (1.0f - m_coeffs[0]) / period;
}

void DerivativeFilter::reset(float value) { m_value = value; };

float DerivativeFilter::step(float delta) {
  m_value = m_value * m_coeffs[0] + delta * m_coeffs[1];
  return m_value;
}

}  // namespace goldobot
