#include "goldobot/platform/hal_io_device_queue.hpp"

#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

namespace goldobot {
namespace hal {
namespace platform {

void IODeviceQueue::init(uint8_t* buffer, size_t buffer_size) {
  m_buffer = buffer;
  m_buffer_end = buffer + buffer_size;
  m_head = buffer;
  m_tail = buffer;
  m_full = 0;
}

size_t IODeviceQueue::size() const {
  taskENTER_CRITICAL();
  auto full = m_full;
  auto head = m_head;
  auto tail = m_tail;
  taskEXIT_CRITICAL();

  if (m_full) {
    return m_buffer_end - m_buffer;
  } else if (head >= tail) {
    return head - tail;
  } else {
    return (m_buffer_end - tail) + (head - m_buffer);
  }
}

size_t IODeviceQueue::max_size() const { return m_buffer_end - m_buffer; }

size_t IODeviceQueue::space_available() const { return max_size() - size(); }

size_t IODeviceQueue::push(const uint8_t* buffer, size_t buffer_size) {
  if (buffer_size == 0 || buffer == nullptr) {
    return 0;
  }

  uint16_t bytes_to_write = space_available();
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

  taskENTER_CRITICAL();
  m_head = head;
  if (head == m_tail) {
    m_full = true;
  }
  taskEXIT_CRITICAL();

  return bytes_to_write;
}

size_t IODeviceQueue::pop(uint8_t* buffer, size_t buffer_size) {
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

  taskENTER_CRITICAL();
  m_tail = tail;
  m_full = false;
  taskEXIT_CRITICAL();

  return bytes_to_read;
}

size_t IODeviceQueue::map_push(uint8_t** buffer) {
  taskENTER_CRITICAL();
  if (m_full) {
    taskEXIT_CRITICAL();
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
  taskEXIT_CRITICAL();
  return retval;
}

void IODeviceQueue::unmap_push(uint8_t* buffer, size_t size) {
  if (size == 0) {
    return;
  }

  uint8_t* head = buffer + size;
  // If the push filled the buffer to the end, go back to the beginning
  if (head == m_buffer_end) {
    head = m_buffer;
  }

  // If head after pushing is equal to tail, the queue is full
  taskENTER_CRITICAL();
  m_full = (head == m_tail);
  m_head = head;
  taskEXIT_CRITICAL();
}

size_t IODeviceQueue::map_pop(uint8_t** buffer) {
  taskENTER_CRITICAL();
  // empty queue
  if (m_head == m_tail && !m_full) {
    taskEXIT_CRITICAL();
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
  taskEXIT_CRITICAL();
  return retval;
}

void IODeviceQueue::unmap_pop(uint8_t* buffer, size_t size) {
  if (size == 0) {
    return;
  }

  uint8_t* tail = buffer + size;
  // If the push emptied the buffer to the end, go back to the beginning
  if (tail == m_buffer_end) {
    tail = m_buffer;
  }
  taskENTER_CRITICAL();
  m_full = false;
  m_tail = tail;
  taskEXIT_CRITICAL();
}

}  // namespace platform
};  // namespace hal
}  // namespace goldobot
