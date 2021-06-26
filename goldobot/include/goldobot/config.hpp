#pragma once
#include "goldobot/enums.hpp"

namespace goldobot {

struct ServoConfig {
  ServoType type;
  uint8_t id;
  uint16_t cw_limit;
  uint16_t ccw_limit;
  uint16_t max_speed;
  uint8_t max_torque;
  uint8_t reserved;
};

struct ServosConfig {
  uint16_t num_servos;
  ServoConfig servos[32];
};

struct SensorConfig {
  uint8_t type{0};
  uint8_t id{0};
};

struct SensorsConfig {
  uint8_t num_sensors;
  SensorConfig sensors[32];
};

struct RobotGeometryConfig {
  //! \brief distance from wheels axis to front of the robot
  float front_length;

  //! \brief distance from wheels axis to back of the robot
  float back_length;
};

struct RobotConfig {
  //! \brief distance from wheels axis to front of the robot
  float front_length;

  //! \brief distance from wheels axis to back of the robot
  float back_length;

  //! \brief use odrive uart interface for propulsion instead of pwm
  bool use_odrive_uart;
  bool use_simulator;

  SensorsConfig sensors;
};

}  // namespace goldobot
