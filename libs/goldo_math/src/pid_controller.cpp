#include <goldobot/core/pid_controller.hpp>

using namespace goldobot;

PIDController::PIDController() {}

PIDController::PIDController(const PIDConfig& config) : m_config(config) {}

void PIDController::setPeriod(float period) { m_period = period; }

const PIDConfig& PIDController::config() const { return m_config; }

void PIDController::setConfig(const PIDConfig& config) { m_config = config; }

float PIDController::output() const { return m_output; }

void PIDController::reset() { m_first_run = true; }

float PIDController::step(float error) {
  // Compute proportional term
  float prop_term = m_config.kp * error;

  // Update integral term and clamp to configured range
  m_integral_term += error * m_config.ki * m_period;
  m_integral_term = clamp(m_integral_term, -m_config.lim_iterm, m_config.lim_iterm);

  // Compute and clamp derivative term, set integral term to zero
  float derivative_term = 0;
  float derivative = (error - m_previous_error) / m_period;
  if (!m_first_run) {
    derivative_term = m_config.kd * derivative;
    derivative_term = clamp(derivative_term, -m_config.lim_dterm, m_config.lim_dterm);
  } else {
    m_first_run = false;
    m_integral_term = 0.0f;
  }
  m_previous_error = error;

  if (m_config.max_output != m_config.min_output) {
    // Anti integral windup protection
    // Change the integral term so that the sum of proportional and integral term stays in the
    // output range
    if (prop_term + m_integral_term > m_config.max_output) {
      m_integral_term = m_config.max_output - prop_term;
    }

    if (prop_term + m_integral_term < m_config.min_output) {
      m_integral_term = m_config.min_output - prop_term;
    }
    m_output = clamp(prop_term + m_integral_term + derivative_term, m_config.min_output,
                     m_config.max_output);
  } else {
    m_output = clamp(prop_term + m_integral_term + derivative_term, m_config.min_output,
                     m_config.max_output);
  }

  return m_output;
}

float PIDController::clamp(float val, float min_val, float max_val) const {
  if (val < min_val) {
    return min_val;
  }
  if (val > max_val) {
    return max_val;
  }
  return val;
}
