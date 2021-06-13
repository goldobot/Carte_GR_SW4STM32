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
	  CommSerializer::Statistics serializer;
	  CommDeserializer::Statistics deserializer;
	  CommSerializer::Statistics serializer_ftdi;
	  CommDeserializer::Statistics deserializer_fdti;
	  MessageQueue::Statistics out_queue;
	  MessageQueue::Statistics out_prio_queue;
	  MessageQueue::Statistics out_ftdi_queue;
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
  void sendHeartbeat(uint32_t timestamp);

  std::atomic<bool> m_ftdi_enable{false};

  uint32_t m_next_heartbeat_timestamp{0};
  uint32_t m_next_statistics_timestamp{0};

  static unsigned char s_scratch_buffer[512];

  static unsigned char s_serialize_buffer[512];
  static unsigned char s_deserialize_buffer[512];

  static unsigned char  s_serialize_ftdi_buffer[512];
  static unsigned char s_deserialize_ftdi_buffer[512];

  static unsigned char s_out_buffer[512];
  static unsigned char s_out_prio_buffer[1024];
  static unsigned char s_out_ftdi_buffer[512];

  CommSerializer m_serializer;
  CommDeserializer m_deserializer;

  CommSerializer m_serializer_ftdi;
  CommDeserializer m_deserializer_ftdi;

  MessageQueue m_out_queue;
  MessageQueue m_out_prio_queue;
  MessageQueue m_out_ftdi_queue;

  Statistics m_statistics;
};
}  // namespace goldobot
