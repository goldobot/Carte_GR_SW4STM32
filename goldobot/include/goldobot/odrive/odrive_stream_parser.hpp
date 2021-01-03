#pragma once
#include "goldobot/core/circular_buffer.hpp"

namespace goldobot {

class ODriveStreamParser {
 public:
  struct Statistics {
    uint32_t bytes_received{0};
    uint16_t messages_received{0};
    uint16_t rx_errors{0};
  };

 public:
  ODriveStreamParser(uint8_t* buffer, size_t buffer_size);
  size_t pushData(unsigned char* buffer, size_t size);

  bool packetReady() const;
  size_t packetSize() const;
  size_t popPacket(uint8_t* buffer, size_t size);

  // Return send statistics for the last period and reset them to zero.
  Statistics statistics();

 private:
  enum class State : uint8_t {
    SearchMagic,
    ReadHeader,
    CheckHeaderCrc,
    ReadPayload,
    PacketReady,
  };

  static constexpr size_t c_max_packet_size = 128;

  uint8_t m_packet_body[130];
  State m_state{State::SearchMagic};
  uint8_t m_packet_length{0};
  uint8_t m_bytes_read{0};
  Statistics m_statistics;
};

}  // namespace goldobot
