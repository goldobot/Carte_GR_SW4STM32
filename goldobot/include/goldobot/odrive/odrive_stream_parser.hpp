#pragma once
#include "goldobot/core/circular_buffer.hpp"

namespace goldobot {

class ODriveStreamParser {
 public:
  ODriveStreamParser(uint8_t* buffer, size_t buffer_size);
  size_t pushData(unsigned char* buffer, size_t size);

  bool packetReady() const;
  size_t packetSize() const;
  void popPacket(uint8_t* buffer, size_t size);

 private:
  uint8_t m_packet_body[130];
  uint8_t m_state{0};
  uint8_t m_packet_length;
  uint8_t m_bytes_read;
};

}  // namespace goldobot
