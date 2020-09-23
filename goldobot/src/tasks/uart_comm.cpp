#include "goldobot/tasks/uart_comm.hpp"

#include <stdarg.h>
#include <stdio.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "goldobot/hal.hpp"
#include "goldobot/platform/message_queue.hpp"
#include "goldobot/propulsion/odometry_config.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/tasks/main.hpp"

using namespace goldobot;

UARTCommTask::UARTCommTask()
    : m_serializer(m_serialize_buffer, sizeof(m_serialize_buffer)),
      m_deserializer(m_deserialize_buffer, sizeof(m_deserialize_buffer)),
      m_out_queue(m_out_buffer, sizeof(m_out_buffer)) {}

const char* UARTCommTask::name() const { return "uart_comm"; }

void UARTCommTask::init() {
  Robot::instance().mainExchangeOut().subscribe({0, 1000, &m_out_queue});
  Task::init();
}

void UARTCommTask::taskFunction() {
  set_priority(5);

  m_last_timestamp = hal::get_tick_count();

  while (1) {
    {
      uint16_t space_available = hal::io_write_space_available(0);
      if (space_available > sizeof(m_scratch_buffer)) {
        space_available = sizeof(m_scratch_buffer);
      }
      size_t dtlen = m_serializer.pop_data((unsigned char*)m_scratch_buffer, space_available);
      if (dtlen) {
        hal::io_write(0, m_scratch_buffer, dtlen);
      }
    }

    // Parse received data
    size_t bytes_read = hal::io_read(0, (unsigned char*)m_scratch_buffer, sizeof(m_scratch_buffer));
    m_deserializer.push_data((unsigned char*)m_scratch_buffer, bytes_read);

    // Copy waiting messages in serializer if needed
    while (m_out_queue.message_ready() &&
           m_out_queue.message_size() < m_serializer.availableSize()) {
      auto msg_type = m_out_queue.message_type();
      auto msg_size = m_out_queue.message_size();
      m_out_queue.pop_message(m_scratch_buffer, msg_size);
      m_serializer.push_message((uint16_t)msg_type, m_scratch_buffer, msg_size);
    }

    // Process received message if needed
    while (m_deserializer.message_ready()) {
      uint16_t message_type = m_deserializer.message_type();
      if (message_type != 0) {
        process_message(message_type);
      } else {
        m_deserializer.pop_message(nullptr, 0);
      }
    }

    uint32_t timestamp = hal::get_tick_count();
    if (timestamp - m_last_timestamp >= 1000) {
      uint8_t msg[sizeof(CommDeserializer::Statistics) + sizeof(CommSerializer::Statistics)];
      auto deserializer_statistics = m_deserializer.statistics();
      auto serializer_statistics = m_serializer.statistics();
      memcpy(msg, &deserializer_statistics, sizeof(CommDeserializer::Statistics));
      memcpy(msg + sizeof(CommDeserializer::Statistics), &serializer_statistics,
             sizeof(CommSerializer::Statistics));
      send_message(CommMessageType::CommStats, (char*)msg, sizeof(msg));
      m_last_timestamp = timestamp;
    }
    // Wait for next tick
    delay(1);
  }
}

bool UARTCommTask::send_message(CommMessageType type, const char* buffer, uint16_t size) {
  m_serializer.push_message((uint16_t)type, (const unsigned char*)(buffer), size);
  return true;
}

void UARTCommTask::process_message(uint16_t message_type) {
  uint16_t msg_type = m_deserializer.message_type();
  size_t msg_size = m_deserializer.message_size();
  m_deserializer.pop_message(m_scratch_buffer, msg_size);
  if (msg_type == static_cast<uint16_t>(CommMessageType::Ping)) {
    m_serializer.push_message(msg_type, (const unsigned char*)(m_scratch_buffer), msg_size);
  } else {
    Robot::instance().mainExchangeIn().pushMessage((CommMessageType)msg_type, m_scratch_buffer,
                                                   msg_size);
  };
}
