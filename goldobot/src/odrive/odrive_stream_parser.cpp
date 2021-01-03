#include "goldobot/odrive/odrive_stream_parser.hpp"

#include "goldobot/odrive/odrive_utils.hpp"

#include <cstring>

namespace goldobot {

ODriveStreamParser::ODriveStreamParser(uint8_t* buffer, size_t size) {}

bool ODriveStreamParser::packetReady() const { return (m_state == State::PacketReady); }

size_t ODriveStreamParser::packetSize() const { return m_packet_length; }

size_t ODriveStreamParser::popPacket(uint8_t* buffer, size_t size) {
  if (m_state != State::PacketReady) {
    return 0;
  }

  if (size > m_packet_length) {
    size = m_packet_length;
  }
  memcpy(buffer, m_packet_body, size);
  m_state = State::SearchMagic;
  return size;
}

size_t ODriveStreamParser::pushData(uint8_t* buffer, size_t size) {
  if (m_state == State::PacketReady) {
    return 0;
  }
  size_t idx = 0;
  while (idx < size) {
    switch (m_state) {
      case State::SearchMagic:
        while (idx < size) {
          if (buffer[idx++] == CANONICAL_PREFIX) {
            m_state = State::ReadHeader;
            break;
          }
        }
        break;

      case State::ReadHeader:
        m_packet_length = buffer[idx++];
        m_state = State::CheckHeaderCrc;
        break;
      case State::CheckHeaderCrc: {
        uint8_t header[2] = {CANONICAL_PREFIX, m_packet_length};
        auto crc8 = calc_crc8<CANONICAL_CRC8_POLYNOMIAL>(CANONICAL_CRC8_INIT, header, 2);
        if (crc8 == buffer[idx++] && m_packet_length <= c_max_packet_size) {
          m_state = State::ReadPayload;
          m_bytes_read = 0;
        } else {
          m_state = State::SearchMagic;
          m_statistics.rx_errors += 1;
        }
      } break;
      case State::ReadPayload: {
        size_t bytes_to_read = size - idx;
        if (bytes_to_read + m_bytes_read > m_packet_length + 2) {
          bytes_to_read = m_packet_length + 2 - m_bytes_read;
        }
        std::memcpy(m_packet_body + m_bytes_read, buffer + idx, bytes_to_read);
        idx += bytes_to_read;
        m_bytes_read += bytes_to_read;
        if (m_bytes_read == m_packet_length + 2) {
          uint16_t crc16 = calc_crc16<CANONICAL_CRC16_POLYNOMIAL>(
              CANONICAL_CRC16_INIT, m_packet_body, m_packet_length + 2);
          // crc16 in big endian
          if (crc16 == 0) {
            m_state = State::PacketReady;
            m_statistics.messages_received += 1;
            m_statistics.bytes_received += idx;
            return idx;
          } else {
            m_statistics.rx_errors += 1;
            m_state = State::SearchMagic;
          }
        }
        break;
      }
    }
  }
  m_statistics.bytes_received += idx;
  return idx;
}

ODriveStreamParser::Statistics ODriveStreamParser::statistics() {
  auto statistics = m_statistics;
  m_statistics = Statistics();
  return statistics;
}
};  // namespace goldobot
