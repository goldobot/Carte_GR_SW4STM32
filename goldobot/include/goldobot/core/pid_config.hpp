#pragma once

namespace goldobot {
struct PIDConfig {
  float kp{0};
  float ki{0};
  float kd{0};
  float lim_iterm{0};
  float lim_dterm{0};
  float d_filter_frequency{0};
  float min_output{0};
  float max_output{0};
};
}  // namespace goldobot
