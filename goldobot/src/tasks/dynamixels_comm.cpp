#include "goldobot/tasks/dynamixels_comm.hpp"

#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#include <string.h>

#include <algorithm>

using namespace goldobot;

struct DynamixelPacketHeaderV1 {
  uint16_t magic;  // 0xFFFF
  uint8_t id;
  uint8_t length;
  uint8_t command;
};

enum class DynamixelParseState { Transmit, ReceiveHeader, ReceiveBody };

bool g_dynamixels_has_status;
volatile DynamixelParseState g_dynamixels_parse_state;
unsigned char* g_dynamixels_buff;
size_t g_dynamixels_bytes_received{0};

bool goldo_dynamixels_callback(hal::IORequest* req, hal::IORequestStatus status) {
  auto& dynamixels_task = *reinterpret_cast<DynamixelsCommTask*>(req->userdata);

  switch (g_dynamixels_parse_state) {
    case DynamixelParseState::Transmit:
      req->rx_ptr = g_dynamixels_buff;
      req->tx_ptr = nullptr;
      req->size = 4;
      g_dynamixels_parse_state = DynamixelParseState::ReceiveHeader;
      g_dynamixels_bytes_received = 0;
      return true;
    case DynamixelParseState::ReceiveHeader:
      g_dynamixels_bytes_received += (req->size - req->remaining);
      g_dynamixels_parse_state = DynamixelParseState::ReceiveBody;
      req->rx_ptr = g_dynamixels_buff + g_dynamixels_bytes_received;
      req->size = g_dynamixels_buff[3];
      return true;
    case DynamixelParseState::ReceiveBody:
      g_dynamixels_bytes_received += (req->size - req->remaining);
      return false;
    default:
      break;
  }
  return false;
}

DynamixelsCommTask::DynamixelsCommTask()
    : m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer)) {}

const char* DynamixelsCommTask::name() const { return "dynamixels_comm"; }

void DynamixelsCommTask::taskFunction() {
  Robot::instance().mainExchangeIn().subscribe({60, 79, &m_message_queue});
  int cnt = 0;
  while (1) {
    while (m_message_queue.message_ready()) {
      processMessage();
    }

    cnt++;
    if (cnt == 100) {
      uint8_t watchdog_id = 4;
      Robot::instance().exchangeInternal().pushMessage(CommMessageType::WatchdogReset, &watchdog_id,
                                                       1);
      cnt = 0;
    }
    delay_periodic(1);
  }
}

bool DynamixelsCommTask::read(uint8_t id, uint8_t address, uint8_t* buffer, size_t size) {
  unsigned char tmp_buff[2];
  tmp_buff[0] = address;
  tmp_buff[1] = size;
  transmitPacket(id, DynamixelCommand::Read, tmp_buff, 2);
  auto received = receivePacket();
  if (received == DynamixelStatusError::Ok) {
    memcpy(buffer, m_dynamixels_buffer + 5, size);
  }
  return received == DynamixelStatusError::Ok;
}

bool DynamixelsCommTask::write(uint8_t id, uint8_t address, uint8_t* buffer, size_t size) {
  unsigned char tmp_buff[16];
  tmp_buff[0] = address;
  memcpy(tmp_buff + 1, buffer, size);
  transmitPacket(id, DynamixelCommand::Write, tmp_buff, size + 1);
  bool received = receivePacket() == DynamixelStatusError::Ok && m_dynamixels_receive_error == 0;
  return received;
}

bool DynamixelsCommTask::regWrite(uint8_t id, uint8_t address, uint8_t* buffer, size_t size) {
  unsigned char tmp_buff[32];
  tmp_buff[0] = address;
  memcpy(tmp_buff + 1, buffer, size);
  transmitPacket(id, DynamixelCommand::RegWrite, tmp_buff, size + 1);
  bool received = receivePacket() == DynamixelStatusError::Ok && m_dynamixels_receive_error == 0;
  return received;
}

void DynamixelsCommTask::action() { transmitPacket(0xFE, DynamixelCommand::Action, nullptr, 0); }

void DynamixelsCommTask::processMessage() {
  auto message_size = m_message_queue.message_size();
  switch (m_message_queue.message_type()) {
    case CommMessageType::DynamixelsRequest: {
      onRequest();

    } break;

    default:
      m_message_queue.pop_message(nullptr, 0);
      break;
  }
}

void DynamixelsCommTask::onRequest() {
  unsigned char buff[3];
  unsigned char data_read[64];
  auto message_size = m_message_queue.message_size();
  m_message_queue.pop_message(m_scratchpad, 256);
  // uint16_t sequence_id, uint8_t protocol version, uint8_t flags, payload
  // flags: 0x01
  uint16_t sequence_id = *reinterpret_cast<uint16_t*>(m_scratchpad);
  uint8_t proto_version = m_scratchpad[2];
  uint8_t _id = m_scratchpad[3];
  m_response_ok = true;
  transmitPacket(m_scratchpad[3], (DynamixelCommand)m_scratchpad[4], &m_scratchpad[5],
                 message_size - 5);

  if (m_response_ok) {
    *reinterpret_cast<uint16_t*>(m_scratchpad) = sequence_id;
    m_scratchpad[2] = proto_version;
    m_scratchpad[3] = _id;
    memcpy(m_scratchpad + 4, m_dynamixels_buffer + 4, m_response_num_parameters + 1);
    if (sequence_id & 0x8000) {
      Robot::instance().exchangeInternal().pushMessage(CommMessageType::DynamixelsResponse,
                                                       (unsigned char*)m_scratchpad,
                                                       m_response_num_parameters + 5);
    } else {
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DynamixelsResponse,
                                                      (unsigned char*)m_scratchpad,
                                                      m_response_num_parameters + 5);
    }
  }
}
void DynamixelsCommTask::transmitPacket(uint8_t id, DynamixelCommand command, uint8_t* parameters,
                                        size_t num_parameters) {
  // protocol v1
  DynamixelPacketHeaderV1& header =
      *reinterpret_cast<DynamixelPacketHeaderV1*>(m_dynamixels_buffer);
  header.magic = 0xFFFF;
  header.id = id;
  header.length = num_parameters + 2;
  header.command = static_cast<uint8_t>(command);
  memcpy(m_dynamixels_buffer + 5, parameters, num_parameters);

  uint8_t checksum = 0;
  for (unsigned i = 2; i < 5 + num_parameters; i++) {
    checksum += m_dynamixels_buffer[i];
  }
  checksum = ~checksum;
  m_dynamixels_buffer[5 + num_parameters] = checksum;

  hal::IORequest req;
  req.tx_ptr = m_dynamixels_buffer;
  req.size = 6 + num_parameters;
  req.callback = &goldo_dynamixels_callback;
  req.userdata = this;

  g_dynamixels_has_status = false;
  m_response_ok = false;
  m_response_num_parameters = 0;

  switch (command) {
    case DynamixelCommand::Read:
      g_dynamixels_has_status = true;
      break;
    case DynamixelCommand::Write:
      g_dynamixels_has_status = true;
      break;
    case DynamixelCommand::Ping:
      g_dynamixels_has_status = true;
      break;
    default:
      g_dynamixels_has_status = false;
      break;
  }

  g_dynamixels_parse_state = DynamixelParseState::Transmit;
  g_dynamixels_buff = m_dynamixels_buffer;
  g_dynamixels_bytes_received = 0;
  hal::io_execute(2, &req, 2);
  delay(1);

  if (g_dynamixels_has_status) {
    // minimal status packet is 2 sync bytes, id, length, error, crc, 6 bytes
    if (g_dynamixels_bytes_received < 6) {
      m_response_ok = false;
      return;
    }
    // check sync bytes
    if (m_dynamixels_buffer[0] != 0xff || m_dynamixels_buffer[1] != 0xff) {
      m_response_ok = false;
      return;
    }

    uint8_t len = m_dynamixels_buffer[3];
    m_response_num_parameters = len - 2;  // response = error + parameters + crc

    // check that length is matching
    if (g_dynamixels_bytes_received != len + 4) {
      m_response_ok = false;
      return;
    }

    // check checksum
    uint8_t rx_checksum = 0;
    for (unsigned i = 2; i < len + 3; i++) {
      rx_checksum += m_dynamixels_buffer[i];
    }
    rx_checksum = ~rx_checksum;

    if (rx_checksum == m_dynamixels_buffer[len + 3]) {
      // ok
      m_response_ok = true;
    } else {
      m_response_ok = false;
    }
  }
}

