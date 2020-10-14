#include "goldobot/platform/hal_io_device_queue.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <cstring>

namespace goldobot {
namespace hal {
namespace platform {

void IODeviceQueue::init(uint8_t* buffer, size_t buffer_size) {
  m_buffer = buffer;
  m_buffer_end = buffer + buffer_size;
  m_head = buffer;
  m_tail = buffer;
}

size_t IODeviceQueue::size() const {
  taskENTER_CRITICAL();
  auto head = m_head;
  auto tail = m_tail;
  taskEXIT_CRITICAL();

  if (head >= tail) {
    return head - tail;
  } else {
    return (m_buffer_end - tail) + (head - m_buffer);
  }
}

size_t IODeviceQueue::max_size() const { return m_buffer_end - m_buffer - 1; }

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

  m_head = head;
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

  m_tail = tail;

  return bytes_to_read;
}

size_t IODeviceQueue::map_push(uint8_t** buffer) {
  auto tail = m_tail;
  auto head = m_head;

  *buffer = head;
  size_t retval = 0;

  if (tail > head) {
    retval = tail - head - 1;
  } else if (tail == m_buffer) {
    // Read data up to end of the buffer
    retval = m_buffer_end - head - 1;
  } else {
    retval = m_buffer_end - head;
  }
  return retval;
}

size_t IODeviceQueue::map_push_2(uint8_t** buffer, uint8_t* head) {
  if (head == nullptr) {
    head = m_head;
  }
  if (head == m_buffer_end) {
    head = m_buffer;
  }

  auto tail = m_tail;
  if (tail == m_buffer) {
    tail = m_buffer_end;
  }

  auto buffer_size = m_buffer_end - m_buffer;
  auto last_quarter = m_buffer + buffer_size * 3 / 4;

  *buffer = head;
  size_t retval = 0;

  if (tail > head) {
    if (tail >= last_quarter) {
      tail = last_quarter;
    }
    retval = tail - head - 1;
    if (retval < buffer_size / 4) {
      retval = 0;
    }
  } else {
    retval = m_buffer_end - head;
  }

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

  assert(buffer >= m_buffer);
  assert(head < m_buffer_end);

  // If head after pushing is equal to tail, the queue is full
  m_head = head;
}

size_t IODeviceQueue::map_pop(uint8_t** buffer) {
  auto tail = m_tail;
  auto head = m_head;
  size_t retval = 0;
  *buffer = tail;
  if (head >= tail) {
    retval = head - m_tail;
  } else {
    retval = m_buffer_end - tail;
  }
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

  assert(buffer >= m_buffer);
  assert(tail < m_buffer_end);

  m_tail = tail;
}

}  // namespace platform
};  // namespace hal
}  // namespace goldobot
