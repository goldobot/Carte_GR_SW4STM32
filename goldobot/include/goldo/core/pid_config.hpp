#pragma once

namespace goldobot
{
  struct PIDConfig
  {
    float period{1};
    float kp{0};
    float ki{0};
    float kd{0};
    float feed_forward{0};
    float lim_iterm{0};
    float lim_dterm{0};
    float min_output{0};
    float max_output{0};
  };
}
