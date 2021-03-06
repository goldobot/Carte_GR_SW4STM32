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

unsigned char __attribute__((section(".ccmram"))) UARTCommTask::s_serialize_buffer[512];
unsigned char __attribute__((section(".ccmram"))) UARTCommTask::s_deserialize_buffer[512];

unsigned char __attribute__((section(".ccmram"))) UARTCommTask::s_serialize_ftdi_buffer[512];
unsigned char __attribute__((section(".ccmram"))) UARTCommTask::s_deserialize_ftdi_buffer[512];

unsigned char __attribute__((section(".ccmram"))) UARTCommTask::s_out_buffer[512];
unsigned char __attribute__((section(".ccmram"))) UARTCommTask::s_out_prio_buffer[1024];
unsigned char __attribute__((section(".ccmram"))) UARTCommTask::s_out_ftdi_buffer[512];
unsigned char __attribute__((section(".ccmram"))) UARTCommTask::s_scratch_buffer[512];

UARTCommTask::UARTCommTask()
    : m_serializer(s_serialize_buffer, sizeof(s_serialize_buffer)),
      m_deserializer(s_deserialize_buffer, sizeof(s_deserialize_buffer)),
      m_serializer_ftdi(s_serialize_ftdi_buffer, sizeof(s_serialize_ftdi_buffer)),
      m_deserializer_ftdi(s_deserialize_ftdi_buffer, sizeof(s_deserialize_ftdi_buffer)),
      m_out_queue(s_out_buffer, sizeof(s_out_buffer)),
      m_out_prio_queue(s_out_buffer, sizeof(s_out_buffer)),
      m_out_ftdi_queue(s_out_ftdi_buffer, sizeof(s_out_ftdi_buffer)) {}

const char* UARTCommTask::name() const { return "uart_comm"; }

void UARTCommTask::init() { Task::init(); }

void UARTCommTask::taskFunction() {
  set_priority(5);

  Robot::instance().mainExchangeOut().subscribe({0, 1000, &m_out_queue});
  Robot::instance().mainExchangeOutPrio().subscribe({0, 1000, &m_out_prio_queue});
  Robot::instance().exchangeOutFtdi().subscribe({0, 1000, &m_out_ftdi_queue});

  Robot::instance().mainExchangeOut().pushMessage(CommMessageType::Reset, nullptr, 0);

  // flush uart to ensure first message is well received
  memset(s_scratch_buffer, 0, sizeof(s_scratch_buffer));
  for (int i = 0; i < 256; i++) {
    hal::io_write(0, s_scratch_buffer, 32);
    delay(1);
  }

  while (1) {
    uint32_t cyccnt_begin = DWT->CYCCNT;

    // write serialized data to uart buffer
    {
      auto space_available = hal::io_write_space_available(0);
      if (space_available > sizeof(s_scratch_buffer)) {
        space_available = sizeof(s_scratch_buffer);
      }
      size_t dtlen = m_serializer.pop_data((unsigned char*)s_scratch_buffer, space_available);
      if (dtlen) {
        hal::io_write(0, s_scratch_buffer, dtlen);
      }
    }

    // write serialized data to ftdi uart buffer
    if (m_ftdi_enable) {
      auto space_available = hal::io_write_space_available(3);
      if (space_available > sizeof(s_scratch_buffer)) {
        space_available = sizeof(s_scratch_buffer);
      }
      size_t dtlen = m_serializer_ftdi.pop_data((unsigned char*)s_scratch_buffer, space_available);
      if (dtlen) {
        hal::io_write(3, s_scratch_buffer, dtlen);
      }
    }

    // Parse received data from uart
    {
      // uint8_t* ptr;
      size_t bytes_read =
          hal::io_read(0, (unsigned char*)s_scratch_buffer, sizeof(s_scratch_buffer));
      m_deserializer.push_data((unsigned char*)s_scratch_buffer, bytes_read);
      // auto dtlen = m_deserializer.push_data((unsigned char*)ptr, bytes_read);
      // hal::io_unmap_read(0, ptr, dtlen);
    }

    // Parse received data from ftdi uart
    if (m_ftdi_enable) {
      uint8_t* ptr;
      size_t bytes_read = hal::io_map_read(0, &ptr);
      auto dtlen = m_deserializer.push_data((unsigned char*)ptr, bytes_read);
      hal::io_unmap_read(0, ptr, dtlen);
    }

    // Copy waiting messages in serializer if needed, starting with high priority queue
    while (m_out_prio_queue.message_ready() &&
           m_out_prio_queue.message_size() < m_serializer.availableSize()) {
      auto msg_type = m_out_prio_queue.message_type();
      auto msg_size = m_out_prio_queue.message_size();
      m_out_prio_queue.pop_message(s_scratch_buffer, msg_size);
      m_serializer.push_message((uint16_t)msg_type, s_scratch_buffer, msg_size);
    }

    if (!m_out_prio_queue.message_ready()) {
      while (m_out_queue.message_ready() &&
             m_out_queue.message_size() + 64 < m_serializer.availableSize()) {
        auto msg_type = m_out_queue.message_type();
        auto msg_size = m_out_queue.message_size();
        m_out_queue.pop_message(s_scratch_buffer, msg_size);
        m_serializer.push_message((uint16_t)msg_type, s_scratch_buffer, msg_size);
      }
    }

    // Copy waiting messages in ftdi serializer if needed
    while (m_out_ftdi_queue.message_ready() &&
           m_out_ftdi_queue.message_size() < m_serializer_ftdi.availableSize()) {
      auto msg_type = m_out_ftdi_queue.message_type();
      auto msg_size = m_out_ftdi_queue.message_size();
      m_out_ftdi_queue.pop_message(s_scratch_buffer, msg_size);
      m_serializer_ftdi.push_message((uint16_t)msg_type, s_scratch_buffer, msg_size);
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
    if (timestamp >= m_next_statistics_timestamp) {
      sendStatistics();
      m_next_statistics_timestamp = m_next_statistics_timestamp + 1000;
    }

    // send heartbeat every 100ms
    if (timestamp >= m_next_heartbeat_timestamp) {
      sendHeartbeat(timestamp);
      m_next_heartbeat_timestamp = m_next_heartbeat_timestamp + 100;
    }

    uint32_t cyccnt_end = DWT->CYCCNT;
    uint32_t cycles_count = cyccnt_end - cyccnt_begin;
    m_statistics.max_cycles = std::max(m_statistics.max_cycles, cycles_count);

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

void UARTCommTask::sendHeartbeat(uint32_t timestamp) {
  Robot::instance().mainExchangeOutPrio().pushMessage(
      CommMessageType::Heartbeat, (unsigned char*)&timestamp, sizeof(timestamp));
}

void UARTCommTask::sendStatistics() {
  m_statistics.serializer = m_serializer.statistics();
  m_statistics.deserializer = m_deserializer.statistics();
  m_statistics.serializer_ftdi = m_serializer_ftdi.statistics();
  m_statistics.deserializer_fdti = m_deserializer_ftdi.statistics();
  m_statistics.out_queue = m_out_queue.statistics();
  m_statistics.out_prio_queue = m_out_prio_queue.statistics();
  m_statistics.out_ftdi_queue = m_out_ftdi_queue.statistics();

  m_out_prio_queue.push_message(CommMessageType::UartCommTaskStatistics,
                                (unsigned char*)&m_statistics, sizeof(m_statistics));
  memset(&m_statistics, 0, sizeof(m_statistics));

  // HeapStats_t heap_stats;
  //       vPortGetHeapStats(&heap_stats);
  //       exchange.pushMessage(CommMessageType::HeapStats, (unsigned char*)&heap_stats,
  //                            sizeof(heap_stats));
}
