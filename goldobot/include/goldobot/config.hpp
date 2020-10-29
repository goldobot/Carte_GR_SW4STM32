#pragma once
#include "goldobot/enums.hpp"

namespace goldobot {

struct ServoConfig {
  ServoType type;
  uint8_t id;
  uint16_t cw_limit;
  uint16_t ccw_limit;
  uint16_t max_speed;
  uint16_t max_torque;
};

struct ServosConfig {
  uint16_t num_servos;
  ServoConfig servos[32];
};

struct SensorConfig
{
	int8_t type; // 0 unknown, 1: gpio, 2: fpga
	int8_t id;
};

struct SensorsConfig
{
	int8_t num_sensors;
	SensorConfig ensors[32];
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
