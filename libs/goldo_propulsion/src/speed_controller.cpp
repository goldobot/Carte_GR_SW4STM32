#include "goldobot/propulsion/speed_controller.hpp"
#include "goldobot/core/math_utils.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

namespace goldobot {

SpeedController::SpeedController()
{
};

void SpeedController::setPeriod(float update_period) { m_period = update_period; }

void SpeedController::setParameterRange(float min_parameter, float max_parameter, bool reset_parameter) {
  m_min_parameter = min_parameter;
  m_max_parameter = max_parameter;
  if (reset_parameter)
  {
    m_parameter = m_min_parameter;
  }
  else
  {
    m_parameter = clamp(m_parameter, m_min_parameter, m_max_parameter);
  }
  recompute();
}

void SpeedController::setRequestedSpeed(float speed) {
  m_requested_speed = speed;
  recompute();
}

void SpeedController::setFinalSpeed(float final_speed) { m_final_speed = final_speed; }

void SpeedController::setAccelerationLimits(float accel, float deccel) {
  m_acceleration_limit = accel;
  m_decceleration_limit = deccel;
}

void SpeedController::update() {
  if (!finished()) {
    m_time += m_period;

    auto t = m_time;
    unsigned int index = m_index;

    while (index + 1 < m_num_points && t > m_t[index + 1]) {
      index++;
    };
    m_index = index;

    float u = t - m_t[index];
    float c0 = m_c0[index];
    float c1 = m_c1[index];
    float c2 = m_c2[index];
    float c3 = m_c3[index];

    m_parameter = c0 + u * (c1 + u * (c2 + u * c3));
    m_speed = c1 + u * (2 * c2 + u * 3 * c3);
    m_acceleration = 2 * c2 + 6 * u * c3;

    m_parameter = clamp(m_parameter, m_min_parameter, m_max_parameter);

    if (finished()) {
      m_speed = m_final_speed;
      m_acceleration = 0;
    }
  }
}

void SpeedController::reset(float current_parameter, float current_speed,
                            float current_acceleration) {
  m_parameter = current_parameter;
  m_speed = current_speed;
  m_acceleration = current_acceleration;
  m_parameter = clamp(m_parameter, m_min_parameter, m_max_parameter);
  recompute();
}

float SpeedController::parameter() const noexcept { return m_parameter; }

float SpeedController::speed() const noexcept { return m_speed; }

float SpeedController::acceleration() const noexcept { return m_acceleration; }

bool SpeedController::finished() const noexcept { return m_parameter == m_max_parameter; }

bool SpeedController::not_feasible(float dist, float speed, float acc, float dec)
{
  /* foolproof.. */
  dist = fabsf(dist);
  speed = fabsf(speed);
  float a1 = fabsf(acc);
  float a2 = fabsf(dec);

  float t_a = speed / a1;
  float t_d = speed / a2;

  float d_a = t_a * (speed + 0.5f * a1 * t_a);
  float d_d = t_d * (speed + 0.5f * a2 * t_d);

  return (d_a>dist) || (d_d>dist);
}

void SpeedController::recompute() {
  m_time = 0;
  m_index = 0;

  if (m_max_parameter - m_parameter < std::numeric_limits<float>::epsilon()) {
    m_num_points = 1;
    m_t[0] = 0;
    m_c0[0] = m_parameter;
    m_c1[0] = m_requested_speed;
    m_c2[0] = 0;
    m_c3[0] = 0;
    return;
  }

  // trapezoidal profile in 3 phases
  // phase 1: from current speed to target_speed
  // phase 2: constant speed
  // phase 3: from target_speed to zero

  float target_speed = m_requested_speed;
  float delta_v_1 = target_speed - m_speed;
  float delta_v_2 = m_final_speed - target_speed;
  float distance = m_max_parameter - m_parameter;

  if (not_feasible(distance,m_speed,m_acceleration_limit,m_decceleration_limit))
  {
    /* FIXME : TODO : refactor! */
    m_num_points = 1;
    m_t[0] = 0;
    m_c0[0] = m_parameter;
    m_c1[0] = m_requested_speed;
    m_c2[0] = 0;
    m_c3[0] = 0;
    return;
  }

  float a1 = delta_v_1 >= 0 ? m_acceleration_limit : -m_decceleration_limit;
  float a2 = delta_v_2 >= 0 ? m_decceleration_limit : -m_decceleration_limit;

  float t_a = delta_v_1 / a1;
  float t_d = delta_v_2 / a2;

  float d_a = t_a * (m_speed + 0.5f * a1 * t_a);
  float d_d = t_d * (target_speed + 0.5f * a2 * t_d);
  float d_c = distance - d_a - d_d;

  while (d_c < 0) {
    target_speed *= 0.95;

    delta_v_1 = target_speed - m_speed;
    delta_v_2 = 0 - target_speed;

    a1 = delta_v_1 >= 0 ? m_acceleration_limit : -m_acceleration_limit;
    a2 = delta_v_2 >= 0 ? m_decceleration_limit : -m_decceleration_limit;

    t_a = delta_v_1 / a1;
    t_d = delta_v_2 / a2;

    d_a = t_a * (m_speed + 0.5f * a1 * t_a);
    d_d = t_d * (target_speed + 0.5f * a2 * t_d);
    d_c = distance - d_a - d_d;
  }

  float t_c = d_c / target_speed;

  m_t[0] = 0;
  m_t[1] = t_a;
  m_t[2] = t_a + t_c;
  m_t[3] = t_a + t_c + t_d;

  m_c0[0] = m_parameter;
  m_c0[1] = m_parameter + d_a;
  m_c0[2] = m_parameter + d_a + d_c;
  m_c0[3] = m_max_parameter;

  m_c1[0] = m_speed;
  m_c1[1] = target_speed;
  m_c1[2] = target_speed;
  m_c1[3] = m_final_speed;

  m_c2[0] = 0.5 * a1;
  m_c2[1] = 0;
  m_c2[2] = 0.5 * a2;
  m_c2[3] = 0;

  m_c3[0] = 0;
  m_c3[1] = 0;
  m_c3[2] = 0;
  m_c3[3] = 0;

  m_num_points = 4;
}

}  // namespace goldobot
