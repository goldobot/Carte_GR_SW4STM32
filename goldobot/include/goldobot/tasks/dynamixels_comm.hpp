#pragma once
#include "goldobot/config.hpp"
#include "goldobot/core/message_queue.hpp"
#include "goldobot/platform/task.hpp"

#include <cstdint>

namespace goldobot {

enum class DynamixelStatusError { Ok, ChecksumError, TimeoutError };

enum class DynamixelCommand : uint8_t {
  Ping = 0x01,
  Read = 0x02,
  Write = 0x03,
  RegWrite = 0x04,
  Action = 0x05,
  FactoryReset = 0x06,
  Reboot = 0x08,
  Clear = 0x10,
  Status = 0x55,
  SyncRead = 0x82,
  SyncWrite = 0x83,
  BulkRead = 0x92,
  BulkWrite = 0x93
};

class DynamixelsCommTask : public Task {
 public:
  DynamixelsCommTask();
  const char* name() const override;

  bool read(uint8_t id, uint8_t address, uint8_t* buffer, size_t size);
  bool write(uint8_t id, uint8_t address, uint8_t* buffer, size_t size);

  bool regWrite(uint8_t id, uint8_t address, uint8_t* buffer, size_t size);
  void action();

  void transmitPacket(uint8_t id, DynamixelCommand command, uint8_t* parameters, size_t size);
  DynamixelStatusError receivePacket();

  void taskFunction() override;

  unsigned char m_dynamixels_buffer[256];
  unsigned char m_scratchpad[256];
  size_t m_dynamixels_receive_size{0};
  bool m_dynamixels_receive_ok;
  uint8_t m_dynamixels_receive_id;
  uint8_t m_dynamixels_receive_num_parameters;
  uint8_t m_dynamixels_receive_error;
  uint8_t* m_dynamixels_receive_params;

  MessageQueue m_message_queue;
  unsigned char m_message_queue_buffer[256];

  void processMessage();
  void onRequest();
};
}  // namespace goldobot
