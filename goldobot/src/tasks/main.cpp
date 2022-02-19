#include "goldobot/tasks/main.hpp"

#include "goldobot/version.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/tasks/uart_comm.hpp"

#include <cmath>
#include <cstring>

using namespace goldobot;

unsigned char g_temp_buffer[32];

#define MY_FIRMWARE_VER_SZ 64

unsigned char __attribute__((section(".ccmram"))) MainTask::s_message_queue_buffer[2048];

MainTask::MainTask() : m_message_queue(s_message_queue_buffer, sizeof(s_message_queue_buffer)) {}

const char* MainTask::name() const { return "main"; }

int MainTask::remainingMatchTime() {
  int match_duration = 100;
  if (!m_match_timer_running) {
    return match_duration;
  }
  int elapsed_time = (hal::get_tick_count() - m_start_of_match_time) / 1000;

  return elapsed_time < match_duration ? match_duration - elapsed_time : 0;
}

void MainTask::taskFunction() {
  Robot::instance().mainExchangeIn().subscribe({5, 5, &m_message_queue});
  Robot::instance().mainExchangeIn().subscribe({10, 12, &m_message_queue});
  Robot::instance().exchangeInternal().subscribe({34, 34, &m_message_queue});
  Robot::instance().mainExchangeIn().subscribe({200, 205, &m_message_queue});

  // Config loop
  while (Robot::instance().matchState() == MatchState::Unconfigured) {
    while (m_message_queue.message_ready()) {
      process_message_config();
    }
    delay(1);
  }

  // Main loop
  while (1) {
    while (m_message_queue.message_ready()) {
      process_message();
    }

    checkSensorsState();

    // Check match timer
    auto remaining_time = remainingMatchTime();
    if (m_match_timer_running && remaining_time == 0) {
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::MatchEnd,
                                                      (unsigned char*)nullptr, 0);
      Robot::instance().exchangeInternal().pushMessage(CommMessageType::MatchEnd,
                                                       (unsigned char*)nullptr, 0);
      m_match_timer_running = false;
    }

    m_cnt++;
    if (m_cnt == 100) {
      uint8_t watchdog_id = 0;
      Robot::instance().exchangeInternal().pushMessage(CommMessageType::WatchdogReset, &watchdog_id,
                                                       1);
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::MatchTimer,
                                                      (unsigned char*)&remaining_time, 4);
      Robot::instance().exchangeInternal().pushMessage(CommMessageType::MatchTimer,
                                                       (unsigned char*)&remaining_time, 4);
      m_cnt = 0;
    }
    delay_periodic(1);
  }
}

void MainTask::process_message_config() {
  int msg_size = m_message_queue.message_size();
  switch (m_message_queue.message_type()) {
    case CommMessageType::GetNucleoFirmwareVersion: {
      m_message_queue.pop_message(nullptr, 0);
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::GetNucleoFirmwareVersion,
                                                      (unsigned char*)goldobot::c_git_commit_id,
                                                      strlen(goldobot::c_git_commit_id));
    } break;
    case CommMessageType::RobotConfigLoadBegin: {
      m_message_queue.pop_message(nullptr, 0);
      Robot::instance().beginLoadConfig();
    } break;

    case CommMessageType::RobotConfigLoadChunk: {
      m_message_queue.pop_message(m_scratchpad, msg_size);
      Robot::instance().loadConfig((char*)m_scratchpad, msg_size);
    } break;

    case CommMessageType::RobotConfigLoadEnd: {
      uint16_t crc;
      m_message_queue.pop_message((unsigned char*)&crc, 2);
      uint8_t status = Robot::instance().endLoadConfig(crc) ? 0 : 1;
      Robot::instance().mainExchangeOutPrio().pushMessage(CommMessageType::RobotConfigLoadStatus,
                                                          (unsigned char*)&status, 1);
    } break;
    default:
      m_message_queue.pop_message(nullptr, 0);
      break;
  }
}
void MainTask::process_message() {
  int msg_size = m_message_queue.message_size();
  switch (m_message_queue.message_type()) {
    case CommMessageType::GetNucleoFirmwareVersion: {
      m_message_queue.pop_message(nullptr, 0);
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::GetNucleoFirmwareVersion,
                                                      (unsigned char*)goldobot::c_git_commit_id,
                                                      strlen(goldobot::c_git_commit_id));
    } break;

    case CommMessageType::MatchTimerStart: {
      m_start_of_match_time = hal::get_tick_count();
      m_match_timer_running = true;
      m_message_queue.pop_message(nullptr, 0);
    } break;
    case CommMessageType::FpgaGpioState:
      m_message_queue.pop_message((unsigned char*)&m_fpga_gpio_state, sizeof(m_fpga_gpio_state));
      break;
    default:
      m_message_queue.pop_message(nullptr, 0);
      break;
  }
}

void MainTask::checkSensorsState() {
  uint32_t sensors_state{0};
  auto& robot = Robot::instance();
  auto& sensors_config = robot.sensorsConfig();
  for (unsigned i = 0; i < sensors_config.num_sensors; i++) {
    const auto& sensor = sensors_config.sensors[i];
    switch (sensor.type) {
      case 1:
        sensors_state |= hal::gpio_get(sensor.id) ? (1 << i) : 0;
        break;
      case 2:
        sensors_state |= (m_fpga_gpio_state & (1 << sensor.id)) != 0 ? (1 << i) : 0;
        break;
      default:
        break;
    }
  }

  if (sensors_state != m_sensors_state) {
    m_sensors_state_changed = true;
  }

  auto timestamp = hal::get_tick_count();

  m_sensors_state = sensors_state;

  if (timestamp >= m_sensors_state_next_ts) {
    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::SensorsState,
                                                    (unsigned char*)&m_sensors_state, 4);
    Robot::instance().exchangeInternal().pushMessage(CommMessageType::SensorsState,
                                                        (unsigned char*)&m_sensors_state, 4);
    m_sensors_state_next_ts = std::max(m_sensors_state_next_ts + 200, timestamp);
    m_sensors_state_changed = false;
  }

  if (m_sensors_state_changed && timestamp >= m_sensors_state_next_ts_min) {
    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::SensorsState,
                                                    (unsigned char*)&m_sensors_state, 4);
    Robot::instance().exchangeInternal().pushMessage(CommMessageType::SensorsState,
                                                        (unsigned char*)&m_sensors_state, 4);
    m_sensors_state_next_ts_min = std::max(m_sensors_state_next_ts_min + 50, timestamp);
    m_sensors_state_changed = false;
  }
}
