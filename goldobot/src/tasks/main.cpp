#include "goldobot/tasks/main.hpp"

#include "goldobot/hal.hpp"
#include "goldobot/messages.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/tasks/uart_comm.hpp"

#include <cmath>
#include <cstring>

using namespace goldobot;

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

unsigned char g_temp_buffer[32];

#ifdef GOLDO_GIT_VERSION
#define _STRINGIFY(A) #A
#define STRINGIFY(A) _STRINGIFY(A)
#define MY_GIT_VERSION STRINGIFY(GOLDO_GIT_VERSION)
#else
#define MY_GIT_VERSION "GOLDO HACK"
#endif

#define MY_FIRMWARE_VER_SZ 64
const char my_firmware_ver[MY_FIRMWARE_VER_SZ] = MY_GIT_VERSION;

unsigned char __attribute__((section(".ccmram"))) MainTask::s_message_queue_buffer[2048];

MainTask::MainTask() : m_message_queue(s_message_queue_buffer, sizeof(s_message_queue_buffer)) {}

const char* MainTask::name() const { return "main"; }

int MainTask::remainingMatchTime() {
  int elapsed_time = (hal::get_tick_count() - m_start_of_match_time) / 1000;
  int match_duration = 100;
  return elapsed_time < match_duration ? match_duration - elapsed_time : 0;
}

void MainTask::taskFunction() {
  Robot::instance().mainExchangeIn().subscribe({10, 12, &m_message_queue});
  Robot::instance().mainExchangeIn().subscribe({200, 205, &m_message_queue});

  // Config loop
  while (Robot::instance().matchState() == MatchState::Unconfigured) {
    while (m_message_queue.message_ready()) {
      process_message_config();
    }
    delay(1);
  }

  while (1) {
    while (m_message_queue.message_ready()) {
      process_message();
    }

    auto remaining_time = remainingMatchTime();
    if(m_match_timer_running && remaining_time == 0)
    {
    	Robot::instance().mainExchangeOut().pushMessage(CommMessageType::MatchEnd,
    	                                                      (unsigned char*)nullptr, 0);
    	Robot::instance().exchangeInternal().pushMessage(CommMessageType::MatchEnd,
    	                                                      (unsigned char*)nullptr, 0);
    }

    m_cnt++;
    if(m_cnt == 100)
    {
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
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::RobotConfigLoadStatus,
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
      /*case CommMessageType::GetNucleoFirmwareVersion: {
        // std::memset(my_firmware_ver,0,MY_FIRMWARE_VER_SZ);
        // std::strncpy(my_firmware_ver, GOLDO_GIT_VERSION, MY_FIRMWARE_VER_SZ);
        m_message_queue.pop_message(nullptr, 0);
        Robot::instance().mainExchangeOut().pushMessage(CommMessageType::GetNucleoFirmwareVersion,
                                                        (unsigned char*)my_firmware_ver,
                                                        MY_FIRMWARE_VER_SZ);
      } break;*/

          case CommMessageType::MatchTimerStart: {
        	m_start_of_match_time = hal::get_tick_count();
        	m_match_timer_running = true;
            m_message_queue.pop_message(nullptr, 0);
          } break;
          /*
          case CommMessageType::MainSequenceBeginLoad:
            m_sequence_engine.beginLoad();
            m_message_queue.pop_message(nullptr, 0);
            break;

          case CommMessageType::MainSequenceEndLoad:
            m_sequence_engine.endLoad();
            m_message_queue.pop_message(nullptr, 0);
            break;

          case CommMessageType::MainSequenceLoadData:
            m_message_queue.pop_message(g_temp_buffer, 32);
            m_sequence_engine.loadData(g_temp_buffer, msg_size);
            break;
          case CommMessageType::MainSequenceStartSequence:
            uint16_t seq_id;
            m_message_queue.pop_message((unsigned char*)(&seq_id), 2);
            m_sequence_engine.startSequence(seq_id);
            break;
          case CommMessageType::MainSequenceAbortSequence:
            m_message_queue.pop_message(nullptr, 0);
            m_sequence_engine.abortSequence();
            break;
          case CommMessageType::PropulsionStateChanged: {
            uint8_t buff[2];
            m_message_queue.pop_message((unsigned char*)(&buff), 2);
            m_sequence_engine.updatePropulsionState((PropulsionState)buff[0]);
          } break;
          case CommMessageType::ArmsStateChange: {
            uint8_t buff[2];
            m_message_queue.pop_message((unsigned char*)(&buff), 2);
            m_sequence_engine.updateArmState((ArmState)buff[1]);
          } break;
          case CommMessageType::FpgaServoState: {
            uint8_t buff[2];
            m_message_queue.pop_message((unsigned char*)(&buff), 2);
            m_sequence_engine.updateServoState(buff[0], buff[1]);
          } break;*/

    default:
      m_message_queue.pop_message(nullptr, 0);
      break;
  }
}
