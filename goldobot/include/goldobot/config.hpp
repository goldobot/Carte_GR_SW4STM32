#pragma once
#include "goldobot/enums.hpp"

namespace goldobot {

struct ServoConfig {
  uint8_t id;
  ServoType type;
  uint16_t cw_limit;
  uint16_t ccw_limit;
  uint16_t max_speed;
};

struct ServosConfig {
  uint16_t num_servos;
  ServoConfig servos[16];
};

struct ArmConfig {
  uint16_t num_servos;
  ServoConfig servos[8];
  uint16_t num_positions;
  uint16_t num_torques;
  uint16_t* positions_ptr;
  uint16_t* torques_ptr;
};

struct RobotConfig {
  //! \brief distance from wheels axis to front of the robot
  float front_length;

  //! \brief distance from wheels axis to back of the robot
  float back_length;
};

}  // namespace goldobot
