#include "goldobot/tasks/odrive_comm.hpp"

#include "goldobot/hal.hpp"
#include "goldobot/propulsion/odometry_config.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/tasks/main.hpp"

#include <stdarg.h>
#include <stdio.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cassert>

using namespace goldobot;

unsigned char __attribute__((section(".ccmram"))) ODriveCommTask::s_message_queue_buffer[1024];
unsigned char __attribute__((section(".ccmram"))) ODriveCommTask::s_write_buffer[512];
unsigned char __attribute__((section(".ccmram"))) ODriveCommTask::s_parse_buffer[512];
unsigned char __attribute__((section(".ccmram"))) ODriveCommTask::s_scratchpad[128];

ODriveCommTask::ODriveCommTask()
    : m_message_queue(s_message_queue_buffer, sizeof(s_message_queue_buffer)),
      m_stream_writer(s_write_buffer, sizeof(s_write_buffer)),
      m_stream_parser(s_parse_buffer, sizeof(s_parse_buffer)) {}

const char* ODriveCommTask::name() const { return "odrive_comm"; }

void ODriveCommTask::taskFunction() {
  set_priority(5);

  Robot::instance().mainExchangeIn().subscribe({50, 59, &m_message_queue});  // odrive messages

  while (1) {
    {
      // Send serialized data
      uint8_t* ptr;
      size_t space_available = hal::io_map_write(1, &ptr);

      if (space_available == 0) {
        m_no_write_cnt++;
      } else {
        m_no_write_cnt = 0;
      }
      size_t dtlen = m_stream_writer.popData(ptr, space_available);
      hal::io_unmap_write(1, ptr, dtlen);
    }
    // Parse received data
    {
      uint8_t* ptr;
      size_t bytes_available = hal::io_map_read(1, &ptr);
      size_t dtlen = m_stream_parser.pushData((unsigned char*)ptr, bytes_available);
      hal::io_unmap_read(1, ptr, dtlen);

      if (bytes_available == 0) {
        m_no_read_cnt++;
      } else {
        m_no_read_cnt = 0;
      }
    }
    // try to reset communication when it crashes
    if (m_no_read_cnt > 100 || m_no_write_cnt > 100) {
      m_no_read_cnt = 0;
      m_no_write_cnt = 0;
      hal::io_reset(1);
    }

    if (m_stream_parser.packetReady()) {
      size_t packet_size = m_stream_parser.packetSize();
      m_stream_parser.popPacket(s_scratchpad, sizeof(s_scratchpad));
      uint16_t sequence_number = *(uint16_t*)s_scratchpad;
      if (!(sequence_number & 0x4000)) {
        Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ODriveResponsePacket,
                                                        s_scratchpad, packet_size);
      } else {
        Robot::instance().exchangeInternal().pushMessage(CommMessageType::ODriveResponsePacket,
                                                         s_scratchpad, packet_size);
      }
    }

    // Process messages
    while (processMessage()) {
    }
    auto current_time = hal::get_tick_count();

    if (current_time >= m_next_statistics_ts) {
      sendStatistics();
      m_next_statistics_ts = std::max(m_next_statistics_ts + 1000, current_time);
    }

    // Wait for next tick
    delay_periodic(1);
  }
}

bool ODriveCommTask::processMessage() {
  if (!m_message_queue.message_ready()) {
    return false;
  }
  auto message_type = (CommMessageType)m_message_queue.message_type();

  switch (message_type) {
    case CommMessageType::ODriveRequestPacket: {
      auto packet_size = m_message_queue.message_size();
      if (packet_size > m_stream_writer.availableSpace() || packet_size > sizeof(s_scratchpad)) {
        m_message_queue.pop_message(nullptr, 0);
        return false;
      };
      m_message_queue.pop_message(s_scratchpad, 128);
      m_stream_writer.pushPacket((unsigned char*)s_scratchpad, packet_size);
    } break;
    default:
      m_message_queue.pop_message(nullptr, 0);
      break;
  }
  return true;
}

void ODriveCommTask::sendStatistics() {
  ODriveCommStats statistics;
  statistics.writer = m_stream_writer.statistics();
  statistics.parser = m_stream_parser.statistics();
  statistics.queue = m_message_queue.statistics();

  Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ODriveCommTaskStatistics,
                                                  (unsigned char*)&statistics, sizeof(statistics));
}
