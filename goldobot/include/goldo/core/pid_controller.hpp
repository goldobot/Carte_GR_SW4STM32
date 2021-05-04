#pragma once
#include "goldo/core/pid_config.hpp"
#include "goldo/core/low_pass_filter.hpp"

namespace goldobot
{
  class PIDController
  {
  public:
    PIDController();
    PIDController(const PIDConfig& config);

    void setPeriod(float period);
    void tweakFeedForward(float new_feed_forward);

    const PIDConfig& config() const;
    void set_config(const PIDConfig& config);

    void set_kp(float kp);
    void set_ki(float ki);
    void set_kd(float kd);

    float output() const;
    void reset();
    float step(float error);

    LowPassFilter& derivative_filter() {return m_derivative_filter;};

  private:
    float clamp(float val, float min_val, float max_val) const;

    PIDConfig m_config;
    float m_period{0.001f};
    float m_backup_feed_forward{0.0f};

    LowPassFilter m_derivative_filter;

    float m_previous_error{0};
    float m_integral_term{0};
    float m_output{0};
    bool m_first_run{true};
  };
}
