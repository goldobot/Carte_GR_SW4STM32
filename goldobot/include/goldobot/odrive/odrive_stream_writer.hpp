#pragma once
#include <cstdint>

#include "goldobot/core/circular_buffer.hpp"

namespace goldobot {

class ODriveStreamWriter {
 public:
  ODriveStreamWriter(uint8_t* buffer, size_t size);
  bool pushPacket(const uint8_t* buffer, size_t size);
  size_t availableSpace() const;
  size_t popData(uint8_t* buffer, size_t size);

 private:
  CircularBuffer<> m_buffer;
};

}  // namespace goldobot
