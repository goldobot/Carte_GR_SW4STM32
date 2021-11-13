#include "goldo/core/message_queue.hpp"

using namespace goldobot;


MessageQueue::MessageQueue(unsigned char* buffer, size_t size):
  m_buffer(buffer),
  m_buffer_size(size),
  m_begin_index(0),
  m_end_index(0),
  m_message_ready(false)
{
  m_mutex = xSemaphoreCreateMutex();
}

bool MessageQueue::push_message(uint16_t message_type, const unsigned char* buffer, size_t msg_size, uint16_t seq)
{
  // Reject message if buffer is full
  if(msg_size > available_capacity())
  {
    return false;
  }

  while(xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE)
  {
  };

  // Set current message type if empty
  if(m_begin_index == m_end_index)
  {
    m_message_size = msg_size;
    m_message_type = message_type;
    m_message_seq = seq;
    m_message_ready = true;
  } 
  else
  {
    uint16_t header[] = {message_type, (uint16_t)msg_size, seq};
    push_data((unsigned char*)header, 6);
  }

  push_data(buffer, msg_size);
  xSemaphoreGive(m_mutex);
  return true;
}

void MessageQueue::push_data(const unsigned char* buffer, size_t size)
{
  for(unsigned i=0; i< size; i++)
  {
    m_buffer[m_end_index] = buffer[i];
    m_end_index++;
    if(m_end_index == m_buffer_size)
    {
      m_end_index = 0;
    }
  }
}

bool MessageQueue::message_ready() const
{
  return m_message_ready;
}

CommMessageType MessageQueue::message_type() const
{
  return (CommMessageType)m_message_type;
}

size_t MessageQueue::message_size() const
{
  return m_message_size;
}

uint16_t MessageQueue::message_seq() const
{
  return m_message_seq;
}

void MessageQueue::read_data(size_t start_index, unsigned char* buffer, size_t size)
{
  size_t i = 0;
  size_t idx = start_index;

  while (i < size)
  {
    buffer[i] = m_buffer[idx];
    i++;
    idx++;
    if (idx == m_buffer_size)
    {
      idx = 0;
    }
  }
}

void MessageQueue::pop_data(size_t size)
{
  m_begin_index += size;
  if (m_begin_index >= m_buffer_size)
  {
    m_begin_index -= m_buffer_size;
  }
}

void MessageQueue::pop_message(unsigned char* buffer, size_t size)
{
  while(xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE)
  {
  };

  if(!m_message_ready)
  {
    xSemaphoreGive(m_mutex);
    return;
  }
  if(buffer)
  {
    read_data(m_begin_index, buffer, m_message_size);
  }
  pop_data(m_message_size);
  if(m_begin_index != m_end_index)
  {
    uint16_t buff[2];
    read_data(m_begin_index, (unsigned char*)buff, 6);
    pop_data(6);
    m_message_type = buff[0];
    m_message_size = buff[1];
    m_message_seq  = buff[2];
  } 
  else
  {
    m_message_ready = false;
    m_message_size = 0;
    m_message_type = 0;
    m_message_seq  = 0xffff;
  }
  xSemaphoreGive(m_mutex);
}

size_t MessageQueue::available_capacity() const
{
  while(xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE)
  {
  };
  size_t size =  m_end_index >= m_begin_index ? m_end_index - m_begin_index : m_end_index - m_begin_index + m_buffer_size;
  auto retval = m_buffer_size > size + 6 ? m_buffer_size - size - 6 : 0;
  xSemaphoreGive(m_mutex);
  return retval;
}
