#pragma once
#include "goldobot/core/pid_config.hpp"
#include "goldobot/core/derivative_filter.hpp"

namespace goldobot {
class PIDController {
 public:
  PIDController();
  PIDController(const PIDConfig& config);

  void setPeriod(float period);

  const PIDConfig& config() const;
  void setConfig(const PIDConfig& config);

  float output() const;
  void reset();
  float step(float error);

 private:
  float clamp(float val, float min_val, float max_val) const;

  PIDConfig m_config;
  float m_period{1.0f};

  DerivativeFilter m_derivative_filter;
  float m_previous_error{0};
  float m_integral_term{0};
  float m_output{0};
  bool m_first_run{true};
};
}  // namespace goldobot
