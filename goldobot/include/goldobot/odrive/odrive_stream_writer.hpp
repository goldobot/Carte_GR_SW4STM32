#pragma once
#include "goldobot/core/circular_buffer.hpp"

#include <cstdint>

namespace goldobot {

class ODriveStreamWriter {
 public:
  struct Statistics {
    uint32_t bytes_sent{0};
    uint16_t messages_sent{0};
    uint16_t tx_highwater{0};
  };

 public:
  ODriveStreamWriter(uint8_t* buffer, size_t size);
  bool pushPacket(const uint8_t* buffer, size_t size);
  size_t size() const;
  size_t availableSpace() const;
  size_t popData(uint8_t* buffer, size_t size);

  // Return send statistics for the last period and reset them to zero.
  Statistics statistics();

 private:
  CircularBuffer<> m_buffer;
  Statistics m_statistics;
};

}  // namespace goldobot
