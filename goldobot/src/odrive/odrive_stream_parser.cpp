#include "goldobot/odrive/odrive_stream_parser.hpp"

#include "goldobot/odrive/odrive_utils.hpp"

#include <cstring>

namespace goldobot {

ODriveStreamParser::ODriveStreamParser(uint8_t* buffer, size_t size) {
  // m_buffer.init(buffer, size);
}

bool ODriveStreamParser::packetReady() const { return (m_state == 4); }

size_t ODriveStreamParser::packetSize() const { return m_packet_length; }

void ODriveStreamParser::popPacket(uint8_t* buffer, size_t size) {
  if (size < m_packet_length) {
    size = m_packet_length;
  }
  memcpy(buffer, m_packet_body, size);
  m_state = 0;
}

size_t ODriveStreamParser::pushData(uint8_t* buffer, size_t size) {
  if (m_state == 4) {
    return 0;
  }
  size_t idx = 0;
  while (idx < size) {
    if (m_state == 0) {
      // Search start of packet marker
      while (idx < size) {
        if (buffer[idx++] == CANONICAL_PREFIX) {
          m_state = 1;
          break;
        }
      }
    } else if (m_state == 1) {
      // Read packet size
      m_packet_length = buffer[idx++];
      m_state = 2;
    } else if (m_state == 2) {
      // check crc8 of header
      uint8_t header[2] = {CANONICAL_PREFIX, m_packet_length};
      auto crc8 = calc_crc8<CANONICAL_CRC8_POLYNOMIAL>(CANONICAL_CRC8_INIT, header, 2);
      if (crc8 == buffer[idx++]) {
        m_state = 3;
        m_bytes_read = 0;
      } else {
        m_state = 0;
      }
    } else if (m_state == 3) {
      size_t bytes_to_read = size - idx;
      if (bytes_to_read > m_packet_length + 2 - m_bytes_read) {
        bytes_to_read = m_packet_length + 2 - m_bytes_read;
      }
      std::memcpy(m_packet_body + m_bytes_read, buffer + idx, bytes_to_read);
      idx += bytes_to_read;
      m_bytes_read += bytes_to_read;
      if (m_bytes_read == m_packet_length + 2) {
        uint16_t crc16 = calc_crc16<CANONICAL_CRC16_POLYNOMIAL>(CANONICAL_CRC16_INIT, m_packet_body,
                                                                m_packet_length + 2);
        // crc16 in big endian
        if (crc16 == 0) {
          m_state = 4;
          return idx;
        } else {
          m_state = 0;
        }
      }
    }
  }
  return idx;
}

};  // namespace goldobot
