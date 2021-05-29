#pragma once
#include "goldobot/comm_deserializer.hpp"
#include "goldobot/comm_serializer.hpp"
#include "goldobot/core/message_queue.hpp"
#include "goldobot/platform/task.hpp"

#include <cstdint>

namespace goldobot {
class UARTCommTask : public Task {
 public:
  struct Statistics
  {
	  uint32_t max_cycles{0};
	  MessageQueue::Statistics out_queue;
	  MessageQueue::Statistics out_prio_queue;
  };
 public:
  UARTCommTask();
  const char* name() const override;
  void init();

 private:
  void taskFunction() override;

  void process_message(uint16_t message_type);
  bool send_message(CommMessageType msg_type, const char* buffer, uint16_t size);

  void sendStatistics();

  uint32_t m_last_timestamp;

  static unsigned char s_scratch_buffer[1024];
  static unsigned char s_serialize_buffer[1024];
  static unsigned char s_deserialize_buffer[1024];
  static unsigned char s_out_buffer[512];
  static unsigned char s_out_prio_buffer[512];
  CommSerializer m_serializer;
  CommDeserializer m_deserializer;
  MessageQueue m_out_queue;
  MessageQueue m_out_prio_queue;

  Statistics m_statistics;
};
}  // namespace goldobot
