#include "goldobot/robot.hpp"

#include <cstring>

#include "goldobot/utils/crc.hpp"

using namespace goldobot;

Robot Robot::s_instance;
unsigned char Robot::s_config_area[16384];

Robot& Robot::instance() { return s_instance; }

void Robot::init() {
  m_comm_task.init();
  m_main_exchange_out.pushMessage(CommMessageType::Reset, nullptr, 0);
  m_main_task.init();
  m_debug_task.init();
  m_heartbeat_task.init();
}

void Robot::start() { m_propulsion_task.start(); }

SimpleOdometry& Robot::odometry() { return m_propulsion_task.odometry(); }

PropulsionControllerConfig Robot::defaultPropulsionControllerConfig() {
  return *m_propulsion_controller_config;
}
OdometryConfig Robot::odometryConfig() { return *m_odometry_config; }

const RobotConfig& Robot::robotConfig() const { return *m_robot_config; }

const RobotSimulatorConfig& Robot::robotSimulatorConfig() const {
  return *m_robot_simulator_config;
}

void Robot::setOdometryConfig(const OdometryConfig& config) {
  *m_odometry_config = config;
  odometry().setConfig(config);
}

ServosConfig* Robot::servosConfig() { return m_servos_config; }

PropulsionController::State Robot::propulsionState() {
  return m_propulsion_task.controller().state();
}

void Robot::beginLoadConfig() {
  m_load_config_ptr = s_config_area;
  m_load_config_crc = 0;
}

void Robot::loadConfig(char* buffer, size_t size) {
  if (m_load_config_ptr == nullptr ||
      m_load_config_ptr + size >= s_config_area + sizeof(s_config_area)) {
    return;
  }
  std::memcpy(m_load_config_ptr, buffer, size);
  m_load_config_ptr += size;
  m_load_config_crc = update_crc16((unsigned char*)buffer, size, m_load_config_crc);
}

bool Robot::endLoadConfig(uint16_t crc) {
  // Config binary starts with array of 16 bits offsets to:
  // HalConfig
  // RobotConfig
  // RobotSimulatorConfig
  // OdometryConfig
  // PropulsionControllerConfig
  // ArmConfig
  // Arm positions
  // ServosConfig

  if (crc != m_load_config_crc) {
    return false;
  }
  uint16_t* offsets = (uint16_t*)s_config_area;
  int i = 0;

  hal::configure(s_config_area + offsets[i++]);
  m_robot_config = (RobotConfig*)(s_config_area + offsets[i++]);
  m_robot_simulator_config = (RobotSimulatorConfig*)(s_config_area + offsets[i++]);
  m_odometry_config = (OdometryConfig*)(s_config_area + offsets[i++]);
  m_propulsion_controller_config = (PropulsionControllerConfig*)(s_config_area + offsets[i++]);
  m_arms_task.m_config = *(ArmConfig*)(s_config_area + offsets[i++]);
  m_servos_config = (ServosConfig*)(s_config_area + offsets[i++]);
  m_arms_task.m_config.positions_ptr = (uint16_t*)(s_config_area + offsets[i++]);
  m_arms_task.m_config.torques_ptr = (uint16_t*)(s_config_area + offsets[i++]);
  m_main_task.sequenceEngine().setBuffer(s_config_area + offsets[i++]);

  m_main_task.sequenceEngine().endLoad();

  odometry().setConfig(*m_odometry_config);
  m_propulsion_task.controller().setConfig(defaultPropulsionControllerConfig());
  m_propulsion_task.init();
  if (m_robot_config->use_odrive_uart) {
    m_odrive_comm_task.init();
  }
  m_arms_task.init();
  m_fpga_task.init();

  start();
  m_match_state = MatchState::Idle;
  return true;
}
