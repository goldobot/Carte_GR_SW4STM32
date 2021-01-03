#include "goldobot/odrive/odrive_stream_writer.hpp"

#include "goldobot/odrive/odrive_utils.hpp"

#include <algorithm>

namespace goldobot {

ODriveStreamWriter::ODriveStreamWriter(uint8_t* buffer, size_t size) {
  m_buffer.init(buffer, size);
}

bool ODriveStreamWriter::pushPacket(const uint8_t* buffer, size_t size) {
  uint8_t header[3];
  header[0] = CANONICAL_PREFIX;
  header[1] = size;
  header[2] = calc_crc8<CANONICAL_CRC8_POLYNOMIAL>(CANONICAL_CRC8_INIT, header, 2);

  uint8_t footer[2];
  uint16_t crc16 = calc_crc16<CANONICAL_CRC16_POLYNOMIAL>(CANONICAL_CRC16_INIT, buffer, size);
  footer[0] = (uint8_t)((crc16 >> 8) & 0xff);  // crc16 in big endian
  footer[1] = (uint8_t)((crc16 >> 0) & 0xff);

  m_buffer.push(header, 3);
  m_buffer.push(buffer, size);
  m_buffer.push(footer, 2);

  m_statistics.messages_sent += 1;
  m_statistics.bytes_sent += size + 5;
  m_statistics.tx_highwater = std::max<uint16_t>(m_statistics.tx_highwater, this->size());
  return true;
}

size_t ODriveStreamWriter::availableSpace() const { return m_buffer.spaceAvailable() - 5; }

size_t ODriveStreamWriter::size() const { return m_buffer.size(); }

size_t ODriveStreamWriter::popData(uint8_t* buffer, size_t size) {
  return m_buffer.pop(buffer, size);
}

ODriveStreamWriter::Statistics ODriveStreamWriter::statistics() {
  auto statistics = m_statistics;
  m_statistics = Statistics();
  return statistics;
}

}  // namespace goldobot
