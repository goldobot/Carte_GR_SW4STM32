#include <goldo/core/low_pass_filter.hpp>
#include <goldo/core/pid_controller.hpp>

using namespace goldobot;

#if 1 /* FIXME : DEBUG : EXPERIMENTAL */
#define TEST_ALPHA 4.0
float g_dbg_deriv_filter_alpha;
#endif

PIDController::PIDController()
{
#if 1 /* FIXME : DEBUG : EXPERIMENTAL */
  g_dbg_deriv_filter_alpha = TEST_ALPHA;
  m_derivative_filter.set_alpha(g_dbg_deriv_filter_alpha);
#endif
}

PIDController::PIDController(const PIDConfig& config):
  m_config(config)
{
#if 1 /* FIXME : DEBUG : EXPERIMENTAL */
  g_dbg_deriv_filter_alpha = TEST_ALPHA;
  m_derivative_filter.set_alpha(g_dbg_deriv_filter_alpha);
  m_backup_feed_forward = config.feed_forward;
#endif
#if 1 /* FIXME : TODO : remove later */
  m_period = config.period;
#endif
}

void PIDController::setPeriod(float period)
{
  m_period = period;
}

const PIDConfig& PIDController::config() const
{
  return m_config;
}

#if 1 /* FIXME : DEBUG : EXPERIMENTAL */
void PIDController::tweakFeedForward(float new_feed_forward)
{
  m_config.feed_forward = new_feed_forward;
}
#endif

void PIDController::set_config(const PIDConfig& config)
{
  m_config = config;
#if 1 /* FIXME : DEBUG : EXPERIMENTAL */
  m_backup_feed_forward = config.feed_forward;
#endif
#if 1 /* FIXME : TODO : remove later */
  m_period = config.period;
#endif
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

float PIDController::output() const
{
  return m_output;
}

void PIDController::reset()
{
  m_first_run = true;
#if 1 /* FIXME : DEBUG : EXPERIMENTAL */
  m_derivative_filter.set_alpha(g_dbg_deriv_filter_alpha);
  m_config.feed_forward = m_backup_feed_forward;
#endif
}

float PIDController::step(float error)
{
  // Compute proportional term
  float prop_term = m_config.kp * error;

  // Update integral term and clamp to configured range
  m_integral_term += error * m_config.ki * m_period;
  m_integral_term = clamp(m_integral_term, -m_config.lim_iterm, m_config.lim_iterm);

  // Compute and clamp derivative term, set integral term to zero
  float derivative_term = 0;
  float derivative = (error - m_previous_error) / m_period;
  if (!m_first_run)
  {
    /* FIXME : DEBUG : EXPERIMENTAL */
    /*  - filter out high frequency noise produced by derivative operator  */
    /*  - to restore the previous (non filtered) behaviour set the "alpha" */
    /*    coefficient of the filter to 1                                   */
    float filtered_derivative = m_derivative_filter.step(derivative);
    derivative_term = m_config.kd * filtered_derivative;
    derivative_term = clamp(derivative_term, -m_config.lim_dterm, m_config.lim_dterm);
  }
  else
  {
    m_first_run = false;
    m_integral_term = 0;
  }
  m_previous_error = error;

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

