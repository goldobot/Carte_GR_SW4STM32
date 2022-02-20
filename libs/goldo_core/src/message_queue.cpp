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
  auto available_cap = available_capacity();
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
  size_t idx = m_end_index;
  size_t idx_end = m_end_index + size;

  if (idx_end < m_buffer_size) {
    std::memcpy(m_buffer + idx, buffer, size);
    m_end_index = idx_end;
  } else {
    auto s1 = m_buffer_size - idx;
    auto s2 = idx_end - m_buffer_size;
    std::memcpy(m_buffer + idx, buffer, s1);
    std::memcpy(m_buffer, buffer + s1, s2);
    m_end_index = s2;
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
  size_t idx = start_index;
  size_t idx_end = start_index + size;

  if (idx_end < m_buffer_size) {
    std::memcpy(buffer, &m_buffer[idx], size);
  } else {
    size_t s1 = m_buffer_size - idx;
    size_t s2 = idx_end - m_buffer_size;

    std::memcpy(buffer, &m_buffer[idx], s1);
    std::memcpy(buffer + m_buffer_size - idx, m_buffer, s2);
  }
}

void MessageQueue::pop_data(size_t size) {
  size_t index = m_begin_index + size;
  while (index >= m_buffer_size) {
    index -= m_buffer_size;
  }
  m_begin_index = index;
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
  load_next_message();
  return size;
}

bool MessageQueue::pop_message(unsigned char** buffers, size_t* sizes, int num_buffers) {
  std::unique_lock<detail::LockerMutex> lock(m_mutex);

  if (!m_message_ready) {
    return false;
  }

  size_t remaining_size = m_message_size;

  for (int i = 0; i < num_buffers; i++) {
    auto ptr = buffers[i];
    size_t size = sizes[i];
    if (size > remaining_size) {
      size = remaining_size;
    }
    remaining_size -= size;
    sizes[i] = size;
    if (size > 0) {
      read_data(m_begin_index, ptr, size);
      pop_data(size);
    }
  }
  pop_data(remaining_size);
  load_next_message();
  return true;
}

void MessageQueue::load_next_message() {
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
}

size_t MessageQueue::buffer_capacity() const noexcept {
  size_t size = m_end_index >= m_begin_index ? m_end_index - m_begin_index
                                             : m_end_index - m_begin_index + m_buffer_size;
  return m_buffer_size - size - 1;
}

size_t MessageQueue::available_capacity() const {
  auto buff_capacity = buffer_capacity();
  return buff_capacity > 4 ? buff_capacity - 4 : 0;
}
}  // namespace goldobot
