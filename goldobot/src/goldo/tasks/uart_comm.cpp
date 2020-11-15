#include "hal/generic/hal.hpp"
#include "goldo/tasks/uart_comm.hpp"
#include "goldo/core/message_queue.hpp"
#include "goldo/tasks/main.hpp"
#include "goldo/propulsion/odometry_config.hpp"
#include "goldo/robot.hpp"
#include <stdio.h>
#include <stdarg.h>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include "stm32f3xx.h"
#include "FreeRTOS.h"
#include "task.h"

using namespace goldobot;


UARTCommTask::UARTCommTask() :
  m_serializer(m_serialize_buffer, sizeof(m_serialize_buffer)),
  m_deserializer(m_deserialize_buffer, sizeof(m_deserialize_buffer)),
  m_out_queue(m_out_buffer, sizeof(m_out_buffer))
{
}

const char* UARTCommTask::name() const
{
  return "uart_comm";
}

void UARTCommTask::init()
{
  Robot::instance().mainExchangeOut().subscribe({0,1000, &m_out_queue});
  Task::init();
}

void UARTCommTask::taskFunction()
{
  set_priority(5);

  m_last_timestamp = xTaskGetTickCount();
  m_bytes_sent = 0;
  m_serialize_buffer_high_watermark = 0;

  Hal::uart_receive(0, m_recv_buffer, sizeof(m_recv_buffer), false);
  while(1)
  {
    // If current transmission is finished, send next chunk of data from ring buffer
    if(Hal::uart_transmit_finished(0))
    {
      auto watermark =  m_serializer.size();
      if(watermark > m_serialize_buffer_high_watermark)
      {
        m_serialize_buffer_high_watermark = watermark;
      }
      size_t dtlen = m_serializer.pop_data((unsigned char*)m_send_buffer, sizeof(m_send_buffer));
      if(dtlen)
      {
        Hal::uart_transmit(0, m_send_buffer, dtlen, false);
      }
      m_bytes_sent += dtlen;
    }

    // Parse received data
    if(Hal::uart_receive_finished(0))
    {
      m_deserializer.push_data((unsigned char*)m_recv_buffer, sizeof(m_recv_buffer));
    }
    else
    {
      // Abort reception if previous was not finished
      uint16_t bytes_received = Hal::uart_receive_abort(0);
      if(bytes_received)
      {
        m_deserializer.push_data((unsigned char*)m_recv_buffer, bytes_received);
      }
    }

    // Launch new receive command
    Hal::uart_receive(0, m_recv_buffer, sizeof(m_recv_buffer), false);

    // Copy waiting messages in serializer if needed
    while(m_out_queue.message_ready() && m_out_queue.message_size() < m_serializer.availableSize())
    {
      auto msg_type = m_out_queue.message_type();
      auto msg_size = m_out_queue.message_size();
      m_out_queue.pop_message(m_tmp_buffer, msg_size);
      m_serializer.push_message((uint16_t)msg_type, m_tmp_buffer, msg_size);
    }

    // Process received message if needed
    if(m_deserializer.message_ready())
    {
      uint16_t message_type = m_deserializer.message_type();
      if(message_type != 0)
      {
        process_message(message_type);
      }
      else
      {
        m_deserializer.pop_message(nullptr,0);
      }
    }

    uint32_t timestamp = xTaskGetTickCount();
    if(timestamp - m_last_timestamp >= 1000)
    {
      uint16_t msg[2];
      msg[0] = (m_bytes_sent * 1000) / (timestamp - m_last_timestamp);
      msg[1] =  m_serialize_buffer_high_watermark;
      send_message(CommMessageType::CommStats, (char*)msg, 4);
      m_last_timestamp = timestamp;
      m_bytes_sent = 0;
      m_serialize_buffer_high_watermark = 0;
    }
    // Wait for next tick
    delay(1);
  }
}

bool UARTCommTask::send_message(CommMessageType type, const char* buffer, uint16_t size)
{
  m_serializer.push_message((uint16_t)type, (const unsigned char*)(buffer), size);
  return true;
}

void UARTCommTask::process_message(uint16_t message_type)
{
  uint16_t msg_type = m_deserializer.message_type();
  size_t msg_size = m_deserializer.message_size();
  m_deserializer.pop_message(m_tmp_buffer, msg_size);
  Robot::instance().mainExchangeIn().pushMessage((CommMessageType)msg_type, m_tmp_buffer, msg_size);
}
