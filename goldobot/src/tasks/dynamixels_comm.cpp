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

DynamixelsCommTask::DynamixelsCommTask()
    : m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer)) {}

const char* DynamixelsCommTask::name() const { return "dynamixels_comm"; }

void DynamixelsCommTask::taskFunction() {
  Robot::instance().mainExchangeIn().subscribe({60,79, &m_message_queue});
  int cnt = 0;
  while (1) {
    while (m_message_queue.message_ready()) {
      processMessage();
    }

    cnt++;
    if(cnt == 100)
    {
    	uint8_t watchdog_id = 4;
    	    Robot::instance().exchangeInternal().pushMessage(CommMessageType::WatchdogReset,&watchdog_id, 1);
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
      unsigned char buff[3];
      unsigned char data_read[64];
      auto message_size = m_message_queue.message_size();
      m_message_queue.pop_message(m_scratchpad, 256);
      // uint16_t sequence_id, uint8_t protocol version, uint8_t flags, payload
      // flags: 0x01
      uint16_t sequence_id = *reinterpret_cast<uint16_t*>(m_scratchpad);
      uint8_t proto_version = m_scratchpad[2];
      uint8_t flags = m_scratchpad[3];
      transmitPacket(m_scratchpad[4], (DynamixelCommand)m_scratchpad[5], &m_scratchpad[6], message_size - 6);
      if(flags & 0x01)
      {
    	  receivePacket();
    	  *reinterpret_cast<uint16_t*>(m_scratchpad) = m_dynamixels_receive_size;
    	  memcpy(m_scratchpad, m_dynamixels_buffer, m_dynamixels_receive_size);
    	  Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DynamixelsResponse,
    	                                                          (unsigned char*)m_scratchpad, m_dynamixels_receive_size + 2);

      }
    } break;

    default:
      m_message_queue.pop_message(nullptr, 0);
      break;
  }
}

void DynamixelsCommTask::transmitPacket(uint8_t id, DynamixelCommand command, uint8_t* parameters,
                                        size_t num_parameters) {
  //protocol v1
  DynamixelPacketHeaderV1& header = *reinterpret_cast<DynamixelPacketHeaderV1*>(m_dynamixels_buffer);
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
  hal::io_write(2, m_dynamixels_buffer, 6 + num_parameters);
}

DynamixelStatusError DynamixelsCommTask::receivePacket() {
  memset(m_dynamixels_buffer, 0, 255);
  delay(1);
  m_dynamixels_receive_size = hal::io_read(2, m_dynamixels_buffer, 256);
  return DynamixelStatusError::Ok;
  uint16_t bytes_received = 0;
  for (unsigned i = 0; i < 10; i++) {
    // bytes_received = Hal::uart_bytes_received(1);

    // search for magic 0xFF
    if (bytes_received >= 4 && bytes_received >= m_dynamixels_buffer[3] + 4) {
      // Hal::uart_receive_abort(1);

      // Check checksum
      m_dynamixels_receive_id = m_dynamixels_buffer[2];
      m_dynamixels_receive_num_parameters = m_dynamixels_buffer[3] - 2;
      m_dynamixels_receive_error = m_dynamixels_buffer[4];
      m_dynamixels_receive_params = m_dynamixels_buffer + 5;

      uint8_t checksum = 0;
      for (unsigned i = 2; i < 5 + m_dynamixels_receive_num_parameters; i++) {
        checksum += m_dynamixels_buffer[i];
      }
      checksum = ~checksum;

      if (checksum == m_dynamixels_buffer[5 + m_dynamixels_receive_num_parameters]) {
        return DynamixelStatusError::Ok;
      } else {
        return DynamixelStatusError::ChecksumError;
      }
    }
    delay(1);
  }
  // Hal::uart_receive_abort(1);
  return DynamixelStatusError::TimeoutError;
}
