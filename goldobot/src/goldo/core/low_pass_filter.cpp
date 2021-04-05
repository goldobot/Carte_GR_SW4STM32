#include <goldo/core/low_pass_filter.hpp>

using namespace goldobot;

LowPassFilter::LowPassFilter()
{
  m_alpha = 1.0;
  reset();
}

LowPassFilter::LowPassFilter(float _alpha)
{
  m_alpha = _alpha;
  reset();
}

void LowPassFilter::set_alpha(float _new_alpha)
{
  m_alpha = _new_alpha;
  reset();
}

void LowPassFilter::reset()
{
  m_out = 0.0;
  m_x_n = 0.0;
  m_y_n = 0.0;
  m_y_n_1 = 0.0;
}

float LowPassFilter::step(float _new_x)
{
  m_x_n = _new_x;
  m_out = (2.0*m_x_n - (1.0-m_alpha)*m_y_n_1) / (1.0+m_alpha);
  m_y_n_1 = m_y_n;
  m_y_n = m_out;
  return m_out;
}

