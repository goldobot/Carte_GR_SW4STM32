#include "goldobot/core/message_queue.hpp"

#include <mutex>
#include <cassert>
#include <cstring>

namespace goldobot {

MessageQueue::MessageQueue(unsigned char* buffer, size_t size)
    : m_buffer(buffer),
      m_buffer_size(size),
      m_begin_index(0),
      m_end_index(0),
      m_message_ready(false) {
  m_statistics.min_available_capacity = available_capacity();
}

bool MessageQueue::push_message(CommMessageType message_type, const unsigned char* buffer,
                                size_t msg_size) {
  std::unique_lock<detail::LockerMutex> lock(m_mutex);
  // Reject message if buffer is full
  if (msg_size > available_capacity()) {
    return false;
  }

  // Set current message type if empty
  if (m_begin_index == m_end_index) {
    m_message_size = msg_size;
    m_message_type = message_type;
    m_message_ready = true;
  } else {
    uint16_t header[] = {static_cast<uint16_t>(message_type), (uint16_t)msg_size};
    push_data((unsigned char*)header, 4);
  }

  push_data(buffer, msg_size);
  m_statistics.min_available_capacity =
      std::min(m_statistics.min_available_capacity, available_capacity());
  m_statistics.messages_pushed++;
  return true;
}

void MessageQueue::push_data(const unsigned char* buffer, size_t size) {
  for (unsigned i = 0; i < size; i++) {
    m_buffer[m_end_index] = buffer[i];
    m_end_index++;
    if (m_end_index == m_buffer_size) {
      m_end_index = 0;
    }
  }
  m_statistics.bytes_pushed += size;
}

bool MessageQueue::message_ready() const { return m_message_ready; }

CommMessageType MessageQueue::message_type() const { return m_message_type; }

size_t MessageQueue::message_size() const { return m_message_size; }

MessageQueue::Statistics MessageQueue::statistics() {
  std::unique_lock<detail::LockerMutex> lock(m_mutex);
  auto retval = m_statistics;
  m_statistics.min_available_capacity = available_capacity();
  m_statistics.bytes_pushed = 0;
  m_statistics.messages_pushed = 0;
  return retval;
}

void MessageQueue::read_data(size_t start_index, unsigned char* buffer, size_t size) {
  size_t i = 0;
  size_t idx = start_index;
  size_t idx_end = start_index + size;

  if (idx_end < m_buffer_size) {
    std::memcpy(buffer, &m_buffer[idx], size);
  } else {
    std::memcpy(buffer, &m_buffer[idx], m_buffer_size - idx);
    std::memcpy(buffer + m_buffer_size - idx, m_buffer, idx_end - m_buffer_size);
  }
}

void MessageQueue::pop_data(size_t size) {
  m_begin_index += size;
  if (m_begin_index >= m_buffer_size) {
    m_begin_index -= m_buffer_size;
  }
}

size_t MessageQueue::pop_message(unsigned char* buffer, size_t size) {
  std::unique_lock<detail::LockerMutex> lock(m_mutex);

  if (!m_message_ready) {
    return 0;
  }

  if (size > m_message_size) {
    size = m_message_size;
  }

  if (buffer) {
    read_data(m_begin_index, buffer, size);
  }

  pop_data(m_message_size);

  if (m_begin_index != m_end_index) {
    uint16_t buff[2];
    read_data(m_begin_index, (unsigned char*)buff, 4);
    pop_data(4);
    m_message_type = static_cast<CommMessageType>(buff[0]);
    m_message_size = buff[1];
  } else {
    m_message_ready = false;
    m_message_size = 0;
    m_message_type = static_cast<CommMessageType>(0);
  }
  return size;
}

size_t MessageQueue::buffer_capacity() const noexcept {
  size_t size = m_end_index >= m_begin_index ? m_end_index - m_begin_index
                                             : m_end_index - m_begin_index + m_buffer_size;
  return m_buffer_size - size - 1;
}

size_t MessageQueue::available_capacity() const {
  auto buff_capacity = buffer_capacity();
  size_t retval = buff_capacity > 4 ? buff_capacity - 4 : 0;
  return retval;
}
}  // namespace goldobot
