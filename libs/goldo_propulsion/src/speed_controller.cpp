#include "goldobot/propulsion/speed_controller.hpp"
#include "goldobot/core/math_utils.hpp"

namespace goldobot {

SpeedController::SpeedController(){};

void SpeedController::setParameterRange(float min_parameter, float max_parameter) {
  m_max_parameter = max_parameter;
  m_parameter = clamp(m_parameter, m_min_parameter, m_max_parameter);
  /*/
m_command_begin_time = m_time_base_ms;
m_command_end_time =
m_time_base_ms + static_cast<uint32_t>(ceilf(1000 * m_speed_profile.end_time()));
m_speed_profile.update(m_trajectory_buffer.max_parameter(), speed, accel, deccel);*/
}

void SpeedController::setRequestedSpeed(float speed) { m_requested_speed = speed; }

void SpeedController::setAccelerationLimits(float accel, float deccel) {
  m_acceleration_limit = accel;
  m_decceleration_limit = deccel;
}

void SpeedController::update() {
  if (!finished()) {
    // todo: support other update rates than 1kHz
    m_time += 1e-3f;
    m_parameter = clamp(m_parameter, m_min_parameter, m_max_parameter);
    m_speed_profile.compute(m_time, &m_parameter, &m_speed, &m_acceleration);
    if (finished()) {
      m_speed = 0;
      m_acceleration = 0;
    }
  }
}

void SpeedController::reset(float current_parameter, float current_speed,
                            float current_acceleration) {
  m_speed_profile.update(m_max_parameter, m_requested_speed, m_acceleration_limit,
                         m_decceleration_limit);
  m_time = m_speed_profile.begin_time();
  m_parameter = current_parameter;
  m_speed = current_speed;
  m_acceleration = current_acceleration;

  m_parameter = clamp(m_parameter, m_min_parameter, m_max_parameter);
  m_time = 0;
}

float SpeedController::parameter() const noexcept { return m_parameter; }

float SpeedController::speed() const noexcept { return m_speed; }

float SpeedController::acceleration() const noexcept { return m_acceleration; }

bool SpeedController::finished() const noexcept { return m_parameter == m_max_parameter; }

}  // namespace goldobot
