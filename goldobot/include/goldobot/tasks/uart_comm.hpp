#pragma once
#include <cstdint>

#include "goldobot/comm_deserializer.hpp"
#include "goldobot/comm_serializer.hpp"
#include "goldobot/platform/message_queue.hpp"
#include "goldobot/platform/task.hpp"

namespace goldobot {
class UARTCommTask : public Task {
 public:
  UARTCommTask();
  const char* name() const override;
  void init();

 private:
  void taskFunction() override;

  void process_message(uint16_t message_type);
  bool send_message(CommMessageType msg_type, const char* buffer, uint16_t size);

  uint32_t m_last_timestamp;

  unsigned char m_scratch_buffer[1024];
  unsigned char m_serialize_buffer[1024];
  unsigned char m_deserialize_buffer[1024];
  unsigned char m_out_buffer[1024];
  CommSerializer m_serializer;
  CommDeserializer m_deserializer;
  MessageQueue m_out_queue;
};
}  // namespace goldobot
