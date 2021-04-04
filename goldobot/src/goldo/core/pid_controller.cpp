#include <goldo/core/low_pass_filter.hpp>
#include <goldo/core/pid_controller.hpp>

using namespace goldobot;

#if 1 /* FIXME : DEBUG */
#define TEST_ALPHA 4.0
extern float g_dbg_deriv_filter_alpha;
#endif

PIDController::PIDController()
{
#if 1 /* FIXME : DEBUG */
  g_dbg_deriv_filter_alpha = TEST_ALPHA;
  m_lpf.set_alpha(g_dbg_deriv_filter_alpha);
#endif
}

PIDController::PIDController(const PIDConfig& config):
  m_config(config)
{
#if 1 /* FIXME : DEBUG */
  g_dbg_deriv_filter_alpha = TEST_ALPHA;
  m_lpf.set_alpha(g_dbg_deriv_filter_alpha);
#endif
}

const PIDConfig& PIDController::config() const
{
  return m_config;
}

void PIDController::set_config(const PIDConfig& config)
{
  m_config = config;
}

void PIDController::set_kp(float kp)
{
  m_config.kp = kp;
}

void PIDController::set_kd(float kd)
{
  m_config.kd = kd;
}

void PIDController::set_ki(float ki)
{
  m_config.ki = ki;
}

void PIDController::set_feedforward(float feedforward)
{
  /* FIXME : TODO */
}

float PIDController::output() const
{
  return m_output;
}

void PIDController::set_target(float target, float target_derivative)
{
  m_target = target;
  m_target_derivative = target_derivative;
}

void PIDController::reset()
{
  m_first_run = true;
#if 1 /* FIXME : DEBUG */
  m_lpf.set_alpha(g_dbg_deriv_filter_alpha);
#endif
}

float PIDController::update(float current_value)
{
  // Compute difference between setpoint and current value
  float error = m_target - current_value;

  // Compute proportional term
  float prop_term = m_config.feed_forward * m_target + m_config.kp * (m_target - current_value);

  // Update integral term and clamp to configured range
  m_integral_term += error * m_config.ki * m_config.period;
  m_integral_term = clamp(m_integral_term, -m_config.lim_iterm, m_config.lim_iterm);

  // Compute and clamp derivative term, set integral term to zero
  float derivative_term = 0;
  if (!m_first_run)
  {
#if 0 /* FIXME : DEBUG */
    derivative_term = (m_target_derivative - (current_value - m_previous_value) / m_config.period) * m_config.kd;
#else
    float current_derivative = (current_value - m_previous_value) / m_config.period;
    float filtered_derivative = m_lpf.update(current_derivative);
    derivative_term = (m_target_derivative - filtered_derivative) * m_config.kd;
#endif
    derivative_term = clamp(derivative_term, -m_config.lim_dterm, m_config.lim_dterm);
    //m_integral_term = 0; /* FIXME : TODO : WTF!? */
  }
  else
  {
    m_first_run = false;
    m_integral_term = 0;
  }
  m_previous_value = current_value;

  if (m_config.max_output != m_config.min_output)
  {
    // Anti integral windup protection
    // Change the integral term so that the sum of proportional and integral term stays in the output range
    if (prop_term + m_integral_term > m_config.max_output)
    {
      m_integral_term = m_config.max_output - prop_term;
    }

    if (prop_term + m_integral_term < m_config.min_output)
    {
      m_integral_term = m_config.min_output - prop_term;
    }  
    m_output = clamp(prop_term + m_integral_term + derivative_term, m_config.min_output, m_config.max_output);
  }
  else
  {
    m_output = clamp(prop_term + m_integral_term + derivative_term, m_config.min_output, m_config.max_output);
  }

  return m_output;
}

float PIDController::clamp(float val, float min_val, float max_val) const
{
  if (val < min_val)
  {
    return min_val;
  }
  if (val > max_val)
  {
    return max_val;
  }
  return val;
}

