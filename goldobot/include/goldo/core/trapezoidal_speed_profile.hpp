#pragma once
#include "goldo/core/trapezoidal_speed_profile.hpp"

namespace goldobot
{
  class TrapezoidalSpeedProfile
  {
  public:
    TrapezoidalSpeedProfile();
    void update(float distance, float speed, float accel, float deccel);
    float begin_time();
    float end_time();
    void compute(float t, float* val, float* deriv, float* accel);

  private:
    float m_c0[8];
    float m_c1[8];
    float m_c2[8];
    float m_c3[8];
    float m_t[8];
    unsigned m_num_points;
  };
}
