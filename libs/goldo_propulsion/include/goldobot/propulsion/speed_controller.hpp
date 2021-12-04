#pragma once
#include "goldobot/core/trapezoidal_speed_profile.hpp"

#include <cstdint>

namespace goldobot {

class SpeedController {
 public:
  SpeedController();

  // Set the control loop's update period in seconds.
  void setPeriod(float update_period);

  //! Called at every control loop step, updates the parameter, speed, acceleration and finished
  //! outputs.
  void update();

  // Reset the controller, the current speed and acceleration are set to given value.
  void reset(float current_parameter, float current_speed, float current_acceleration);

  //! Set the current trajectory parameter and the valid parameter range for the demanded move.
  //! Called when starting a rotation or trajectory, or when dynamically updating the trajectory in
  //! trajectory following mode
  void setParameterRange(float min_parameter, float max_parameter);

  void setRequestedSpeed(float speed);
  void setFinalSpeed(float final_speed);
  void setAccelerationLimits(float accel, float deccel);

  float maxParameter() const noexcept;
  float parameter() const noexcept;
  float speed() const noexcept;
  float acceleration() const noexcept;
  bool finished() const noexcept;

 private:
  void recompute();
  float m_min_parameter{0};
  float m_max_parameter{0};
  float m_parameter{0};
  float m_speed{0};
  float m_acceleration{0};

  float m_requested_speed{0};
  float m_final_speed{0};
  float m_acceleration_limit{1};
  float m_decceleration_limit{1};

  // temporary
  float m_time{0};
  float m_period{1e-3f};
  int m_index{0};

  // polynomial parameters
  float m_c0[8];
  float m_c1[8];
  float m_c2[8];
  float m_c3[8];
  float m_t[8];
  unsigned m_num_points{0};
};
}  // namespace goldobot
