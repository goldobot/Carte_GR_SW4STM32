#ifndef __GOLDO_COMM_MSG_QUEUE_HPP__
#define __GOLDO_COMM_MSG_QUEUE_HPP__

#include "goldo_comm/circular_buffer.hpp"

namespace goldo_comm {
template <size_t BufferSize, size_t MaxMessages>
class MessageQueue {
 public:
  MessageQueue();

  bool empty() const noexcept;
  bool full() const noexcept;
  size_t available_message_size() const noexcept;

  bool push_message(const uint8_t* buffer, size_t buffer_size);
  uint8_t* lock_write(size_t& buffer_size);
  void unlock_write(size_t written_size);
  bool end_message();

  size_t pop_message(uint8_t* buffer, size_t buffer_size);
  const uint8_t* lock_read(size_t& buffer_size);
  void unlock_read(size_t written_size);
  //! \brief reset read pointer to start of message
  void restart_read();
  void pop_message();
  size_t message_size();

  // private:
  using buffer = CircularBuffer<BufferSize>;
  buffer m_buffer;
  std::array<typename buffer::iterator, MaxMessages> m_msg_idx;
  size_t m_cur_msg_already_read{0};
  int m_write_idx{0};
  int m_read_idx{0};
  bool m_full{false};
};

template <size_t BufferSize, size_t MaxMessages>
MessageQueue<BufferSize, MaxMessages>::MessageQueue() {
  m_msg_idx[m_write_idx] = m_buffer.m_write_ptr;
}

template <size_t BufferSize, size_t MaxMessages>
bool MessageQueue<BufferSize, MaxMessages>::empty() const noexcept {
  return m_read_idx == m_write_idx && !m_full;
}

template <size_t BufferSize, size_t MaxMessages>
bool MessageQueue<BufferSize, MaxMessages>::full() const noexcept {
  return m_full || m_buffer.full();
}

template <size_t BufferSize, size_t MaxMessages>
const uint8_t* MessageQueue<BufferSize, MaxMessages>::lock_read(
    size_t& buffer_size) {
  return m_buffer.lock_read(buffer_size);
}

template <size_t BufferSize, size_t MaxMessages>
void MessageQueue<BufferSize, MaxMessages>::unlock_read(size_t written_size) {
  m_buffer.unlock_read(written_size);
  m_cur_msg_already_read += written_size;
}

template <size_t BufferSize, size_t MaxMessages>
uint8_t* MessageQueue<BufferSize, MaxMessages>::lock_write(
    size_t& buffer_size) {
  return m_buffer.lock_write(buffer_size);
}

template <size_t BufferSize, size_t MaxMessages>
void MessageQueue<BufferSize, MaxMessages>::unlock_write(size_t written_size) {
  m_buffer.unlock_write(written_size);
}

template <size_t BufferSize, size_t MaxMessages>
size_t MessageQueue<BufferSize, MaxMessages>::available_message_size()
    const noexcept {
  return m_buffer.available_space();
}

template <size_t BufferSize, size_t MaxMessages>
bool MessageQueue<BufferSize, MaxMessages>::push_message(const uint8_t* buffer,
                                                         size_t buffer_size) {
  if (buffer_size > available_message_size()) {
    return false;
  }
  m_buffer.push(buffer, buffer_size);
  end_message();
  return true;
}

template <size_t BufferSize, size_t MaxMessages>
bool MessageQueue<BufferSize, MaxMessages>::end_message() {
  if (m_full) {
    return false;
  }
  m_write_idx++;
  if (m_write_idx == MaxMessages) {
    m_write_idx = 0;
  }
  m_msg_idx[m_write_idx] = m_buffer.m_write_ptr;
  if (m_write_idx == m_read_idx) {
    m_full = true;
  }
  return true;
}

template <size_t BufferSize, size_t MaxMessages>
size_t MessageQueue<BufferSize, MaxMessages>::pop_message(uint8_t* buffer,
                                                          size_t buffer_size) {
  if (empty()) {
    return 0;
  }

  buffer_size = std::min(buffer_size, message_size());
  m_buffer.pop(buffer, buffer_size);
  pop_message();
  return buffer_size;
}

template <size_t BufferSize, size_t MaxMessages>
void MessageQueue<BufferSize, MaxMessages>::pop_message() {
  if (empty()) {
    return;
  }
  m_read_idx++;
  if (m_read_idx == MaxMessages) {
    m_read_idx = 0;
  }
  m_full = false;
  m_cur_msg_already_read = 0;
  m_buffer.m_read_ptr = m_msg_idx[m_read_idx];
}

template <size_t BufferSize, size_t MaxMessages>
size_t MessageQueue<BufferSize, MaxMessages>::message_size() {
  auto it1 = m_msg_idx[m_read_idx];
  auto it2 = m_msg_idx[m_read_idx + 1 == MaxMessages ? 0 : m_read_idx + 1];
  if (it2 > it1) {
    return it2 - it1 - m_cur_msg_already_read;
  } else {
    return BufferSize - (it1 - it2) - m_cur_msg_already_read;
  }
}
}  // namespace goldo_comm

#endif  // __GOLDO_COMM_MSG_QUEUE_HPP__
