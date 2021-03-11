#pragma once
#include "goldobot/platform/lockers.hpp"

#include <cstdint>
#include <cstring>
namespace goldobot {

struct LockerNull {
  void lock(){};
  void unlock(){};
};

template <typename Locker = detail::LockerNull>
class CircularBuffer {
 public:
  void init(uint8_t* buffer, size_t buffer_size) {
    m_buffer = buffer;
    m_buffer_end = buffer + buffer_size;
    m_head = buffer;
    m_tail = buffer;
    m_full = 0;
  }

  // Returns the amount of data
  size_t size() const noexcept {
    m_locker.lock();
    auto full = m_full;
    auto head = m_head;
    auto tail = m_tail;
    m_locker.unlock();

    if (full) {
      return m_buffer_end - m_buffer;
    } else if (head >= tail) {
      return head - tail;
    } else {
      return (m_buffer_end - tail) + (head - m_buffer);
    }
  }

  // Returns the capacity of the queue
  size_t maxSize() const noexcept { return m_buffer_end - m_buffer; }

  // Returns the space available  for pushing data
  size_t spaceAvailable() const noexcept { return maxSize() - size(); };

  size_t push(const uint8_t* buffer, size_t buffer_size) noexcept {
    // Return immediately if nothing to push
    if (buffer_size == 0 || buffer == nullptr) {
      return 0;
    }

    // Number of bytes written is the minimum of buffer_size and sapce avaialable in the circular
    // buffer
    uint16_t bytes_to_write = spaceAvailable();
    if (buffer_size < bytes_to_write) {
      bytes_to_write = buffer_size;
    }

    uint8_t* head = m_head;
    if (head + bytes_to_write < m_buffer_end) {
      std::memcpy(head, buffer, bytes_to_write);
      head = head + bytes_to_write;

    } else {
      size_t size_1 = m_buffer_end - head;
      size_t size_2 = bytes_to_write - size_1;
      std::memcpy(head, buffer, size_1);
      std::memcpy(m_buffer, buffer + size_1, size_2);
      head = m_buffer + size_2;
    }

    m_locker.lock();
    m_head = head;
    if (head == m_tail) {
      m_full = true;
    }
    m_locker.unlock();

    return bytes_to_write;
  }

  size_t pop(uint8_t* buffer, size_t buffer_size) {
    if (buffer_size == 0 || buffer == nullptr) {
      return 0;
    }

    uint16_t bytes_to_read = size();
    if (bytes_to_read > buffer_size) {
      bytes_to_read = buffer_size;
    }

    if (bytes_to_read == 0) {
      return 0;
    }

    uint8_t* tail = m_tail;

    if (tail + bytes_to_read < m_buffer_end) {
      std::memcpy(buffer, tail, bytes_to_read);
      tail = tail + bytes_to_read;
    } else {
      size_t size_1 = m_buffer_end - tail;
      size_t size_2 = bytes_to_read - size_1;

      std::memcpy(buffer, tail, size_1);
      std::memcpy(buffer + size_1, m_buffer, size_2);
      tail = m_buffer + size_2;
    }

    m_locker.lock();
    m_tail = tail;
    m_full = false;
    m_locker.unlock();

    return bytes_to_read;
  }

  // Use mapPush to access the next area of the queue to write and unmap_push to commit the write
  // without copy
  size_t mapPush(uint8_t** buffer) noexcept {
    m_locker.lock();
    if (m_full) {
      m_locker.unlock();
      *buffer = nullptr;
      return 0;
    }

    *buffer = m_head;
    size_t retval = 0;

    if (m_tail > m_head) {
      retval = m_tail - m_head;
    } else {
      // Read data up to end of the buffer
      retval = m_buffer_end - m_head;
    }
    m_locker.unlock();
    return retval;
  }

  void unmapPush(uint8_t* buffer, size_t size) {
    if (size == 0) {
      return;
    }

    uint8_t* head = buffer + size;
    // If the push filled the buffer to the end, go back to the beginning
    if (head == m_buffer_end) {
      head = m_buffer;
    }

    // If head after pushing is equal to tail, the queue is full
    m_locker.lock();
    m_full = (head == m_tail);
    m_head = head;
    m_locker.unlock();
  }

  size_t mapPop(uint8_t** buffer) noexcept {
    m_locker.lock();
    // empty queue
    if (m_head == m_tail && !m_full) {
      m_locker.unlock();
      *buffer = nullptr;
      return 0;
    }

    size_t retval = 0;
    *buffer = m_tail;
    if (m_head > m_tail) {
      retval = m_head - m_tail;
    } else {
      retval = m_buffer_end - m_tail;
    }
    m_locker.unlock();
    return retval;
  }

  void unmapPop(uint8_t* buffer, size_t size) {
    if (size == 0) {
      return;
    }

    uint8_t* tail = buffer + size;
    // If the push emptied the buffer to the end, go back to the beginning
    if (tail == m_buffer_end) {
      tail = m_buffer;
    }
    m_locker.lock();
    m_full = false;
    m_tail = tail;
    m_locker.unlock();
  }

 private:
  uint8_t* m_buffer;
  uint8_t* m_buffer_end;
  uint8_t* m_head;
  uint8_t* m_tail;
  bool m_full;
  mutable Locker m_locker;
};

}  // namespace goldobot
