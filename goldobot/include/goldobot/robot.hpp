#pragma once
#include "goldobot/enums.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/platform/config.hpp"
#include "goldobot/core/message_exchange.hpp"
#include "goldobot/propulsion/robot_simulator.hpp"
#include "goldobot/tasks/servos.hpp"
#include "goldobot/tasks/fpga.hpp"
#include "goldobot/tasks/main.hpp"
#include "goldobot/tasks/odrive_comm.hpp"
#include "goldobot/tasks/propulsion.hpp"
#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/tasks/dynamixels_comm.hpp"

#include <cstdint>

namespace goldobot {

class Robot {
 public:
  enum class ConfigSection : uint8_t {
    Hal,
    RobotGeometry,
    Sensors,
    RobotSimulator,
    Odometry,
    PropulsionController,
    Servos,
    PropulsionTask,
    TasksEnable,
    Lifts
  };

 public:
  static Robot& instance();
  void init();
  void start();

  MatchState matchState() const noexcept { return m_match_state; };
  void setMatchState(MatchState state) noexcept { m_match_state = state; };

  uint32_t sensorsState() const { return m_sensors_state; };
  void setSensorsState(uint32_t state) { m_sensors_state = state; };

  void setStartMatchTime(uint32_t time) { m_start_match_time = time; };

  int remainingMatchTime() const { return m_remaining_match_time; };
  void setRemainingMatchTime(int t) { m_remaining_match_time = t; };

  SimpleOdometry& odometry();

  MessageExchange& mainExchangeIn() { return m_main_exchange_in; };
  MessageExchange& mainExchangeOut() { return m_main_exchange_out; };
  MessageExchange& mainExchangeOutPrio() { return m_main_exchange_out_prio; };
  MessageExchange& exchangeOutFtdi() { return m_exchange_out_ftdi; };
  MessageExchange& exchangeInternal() { return m_exchange_internal; };

  const RobotGeometryConfig& robotGeometry() const;

  ServosConfig* servosConfig();
  LiftsConfig* liftsConfig();
  const SensorsConfig& sensorsConfig() const noexcept;

  PropulsionController::State propulsionState();

  // Synopsis: beginLoad, multiple calls to loadConfig to write config buffer, endLoadConfig to
  // check
  void beginLoadConfig();
  void loadConfig(char* buffer, size_t size);
  bool endLoadConfig(uint16_t crc);
  LiftsConfig* m_lifts_config{nullptr};

 private:
  SensorsConfig m_sensors_config;
  MatchState m_match_state{MatchState::Unconfigured};

  std::atomic<int> m_start_match_time{0};
  std::atomic<int> m_remaining_match_time{0};
  uint32_t m_sensors_state{0};

  MainTask m_main_task;
  PropulsionTask m_propulsion_task;
  ServosTask m_servos_task;
  FpgaTask m_fpga_task;
  UARTCommTask m_comm_task;
  ODriveCommTask m_odrive_comm_task;
  DynamixelsCommTask m_dynamixels_comm_task;

  RobotGeometryConfig* m_robot_geometry_config;
  ServosConfig* m_servos_config;

  unsigned char* m_load_config_ptr{nullptr};
  uint16_t m_load_config_crc{0};

  MessageExchange m_main_exchange_in;
  MessageExchange m_main_exchange_out;
  MessageExchange m_main_exchange_out_prio;
  MessageExchange m_exchange_out_ftdi;
  MessageExchange m_exchange_internal;

  static unsigned char s_config_area[16384];
  static Robot s_instance;
};
}  // namespace goldobot
