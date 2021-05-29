#include "goldobot/tasks/uart_comm.hpp"

#include "goldobot/hal.hpp"
#include "goldobot/propulsion/odometry_config.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/tasks/main.hpp"

// for measuring computing time, should be in hal
#include "stm32f3xx_hal.h"
#include "core_cm4.h"

#include <stdarg.h>
#include <stdio.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

using namespace goldobot;

unsigned char __attribute__((section(".ccmram"))) UARTCommTask::s_serialize_buffer[1024];
unsigned char __attribute__((section(".ccmram"))) UARTCommTask::s_deserialize_buffer[1024];
unsigned char __attribute__((section(".ccmram"))) UARTCommTask::s_out_buffer[512];
unsigned char __attribute__((section(".ccmram"))) UARTCommTask::s_out_prio_buffer[512];
unsigned char __attribute__((section(".ccmram"))) UARTCommTask::s_scratch_buffer[1024];

UARTCommTask::UARTCommTask()
    : m_serializer(s_serialize_buffer, sizeof(s_serialize_buffer)),
      m_deserializer(s_deserialize_buffer, sizeof(s_deserialize_buffer)),
      m_out_queue(s_out_buffer, sizeof(s_out_buffer)),
	  m_out_prio_queue(s_out_buffer, sizeof(s_out_buffer)){}

const char* UARTCommTask::name() const { return "uart_comm"; }

void UARTCommTask::init() {
  Robot::instance().mainExchangeOut().subscribe({0, 1000, &m_out_queue});
  Robot::instance().mainExchangeOutPrio().subscribe({0, 1000, &m_out_prio_queue});
  Task::init();
}

void UARTCommTask::taskFunction() {
  set_priority(5);

  m_last_timestamp = hal::get_tick_count();

  // flush uart to ensure first message is well received
  memset(s_scratch_buffer, 0, sizeof(s_scratch_buffer));
  for(int i = 0; i < 32; i++)
  {
	  hal::io_write(0, s_scratch_buffer, 32);
	  delay(10);
  }


  while (1) {
	uint32_t cyccnt_begin = DWT->CYCCNT;
    {
      uint16_t space_available = hal::io_write_space_available(0);
      if (space_available > sizeof(s_scratch_buffer)) {
        space_available = sizeof(s_scratch_buffer);
      }
      size_t dtlen = m_serializer.pop_data((unsigned char*)s_scratch_buffer, space_available);
      if (dtlen) {
        hal::io_write(0, s_scratch_buffer, dtlen);
      }
    }

    // Parse received data
    size_t bytes_read = hal::io_read(0, (unsigned char*)s_scratch_buffer, sizeof(s_scratch_buffer));
    m_deserializer.push_data((unsigned char*)s_scratch_buffer, bytes_read);

    // Copy waiting messages in serializer if needed, starting with high priority queue
    while (m_out_prio_queue.message_ready() &&
    		m_out_prio_queue.message_size() < m_serializer.availableSize()) {
      auto msg_type = m_out_prio_queue.message_type();
      auto msg_size = m_out_prio_queue.message_size();
      m_out_prio_queue.pop_message(s_scratch_buffer, msg_size);
      m_serializer.push_message((uint16_t)msg_type, s_scratch_buffer, msg_size);
    }

    if(!m_out_prio_queue.message_ready())
    {
	  while (m_out_queue.message_ready() &&
	   	     m_out_queue.message_size() + 64 < m_serializer.availableSize()) {
	    auto msg_type = m_out_queue.message_type();
	    auto msg_size = m_out_queue.message_size();
	    m_out_queue.pop_message(s_scratch_buffer, msg_size);
	    m_serializer.push_message((uint16_t)msg_type, s_scratch_buffer, msg_size);
	  }
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

    // send task statistics every second
    if (timestamp - m_last_timestamp >= 1000) {
      sendStatistics();



      m_last_timestamp = timestamp;
    }

    uint32_t cyccnt_end = DWT->CYCCNT;
    uint32_t cycles_count = cyccnt_end - cyccnt_begin;

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
  m_deserializer.pop_message(s_scratch_buffer, msg_size);
  if (msg_type == static_cast<uint16_t>(CommMessageType::CommUartPing)) {
    m_serializer.push_message(msg_type, (const unsigned char*)(s_scratch_buffer), msg_size);
  } else {
    Robot::instance().mainExchangeIn().pushMessage((CommMessageType)msg_type, s_scratch_buffer,
                                                   msg_size);
  };
}

void UARTCommTask::sendStatistics()
{
  uint8_t msg[sizeof(CommDeserializer::Statistics) + sizeof(CommSerializer::Statistics)];
  auto deserializer_statistics = m_deserializer.statistics();
  auto serializer_statistics = m_serializer.statistics();
  memcpy(msg, &deserializer_statistics, sizeof(CommDeserializer::Statistics));
  memcpy(msg + sizeof(CommDeserializer::Statistics), &serializer_statistics,
		 sizeof(CommSerializer::Statistics));
  send_message(CommMessageType::CommUartStats, (char*)msg, sizeof(msg));

  m_statistics.out_queue = m_out_queue.statistics();
  m_statistics.out_prio_queue = m_out_prio_queue.statistics();

  send_message(CommMessageType::UartCommTaskStatistics, (char*)&m_statistics, sizeof(m_statistics));
  m_statistics = Statistics();


}
