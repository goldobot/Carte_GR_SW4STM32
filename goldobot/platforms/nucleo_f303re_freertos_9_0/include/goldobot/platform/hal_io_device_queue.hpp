#pragma once
#include <assert.h>

#include "goldobot/hal.hpp"

namespace goldobot {
namespace hal {
namespace platform {

class IODeviceQueue {
 public:
  void init(uint8_t* buffer, size_t buffer_size);

  // Returns the amount of data
  size_t size() const;

  // Returns the capacity of the queue
  size_t max_size() const;

  // Returns the space available  for pushing data
  size_t space_available() const;

  size_t push(const uint8_t* buffer, size_t buffer_size);
  size_t pop(uint8_t* buffer, size_t buffer_size);

  // Use map_push to access the next area of the queue to write and unmap_push to commit the write
  // without copy
  size_t map_push(uint8_t** buffer);
  size_t map_push_2(uint8_t** buffer,
                    uint8_t* head);  // dirty hack, map on 1/4 of queue size boundaries
  void unmap_push(uint8_t* buffer, size_t size);

  size_t map_pop(uint8_t** buffer);
  void unmap_pop(uint8_t* buffer, size_t size);

 private:
  uint8_t* m_buffer;
  uint8_t* m_buffer_end;
  uint8_t* m_head;
  uint8_t* m_tail;
};

}  // namespace platform
}  // namespace hal
}  // namespace goldobot
