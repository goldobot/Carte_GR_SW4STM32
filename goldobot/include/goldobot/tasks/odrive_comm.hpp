#pragma once
#include "goldobot/odrive/odrive_stream_parser.hpp"
#include "goldobot/odrive/odrive_stream_writer.hpp"
#include "goldobot/core/message_queue.hpp"
#include "goldobot/platform/task.hpp"

#include <cstdint>

namespace goldobot {

struct ODriveCommStats {
  uint16_t bytes_sent{0};
  uint16_t requests_sent{0};
  uint16_t bytes_received{0};
  uint16_t requests_received{0};
  uint16_t tx_highwater{0};
  uint16_t rx_highwater{0};
};

class ODriveCommTask : public Task {
 public:
  ODriveCommTask();
  const char* name() const override;

 private:
  void taskFunction() override;

  bool processMessage();

  MessageQueue m_message_queue;
  ODriveStreamParser m_stream_parser;
  ODriveStreamWriter m_stream_writer;

  size_t m_bytes_sent{0};
  size_t m_requests_sent{0};
  int m_cnt{0};
  ODriveCommStats m_comm_stats;

  static unsigned char s_message_queue_buffer[1024];
  static unsigned char s_write_buffer[512];
  static unsigned char s_parse_buffer[512];
  static unsigned char s_scratchpad[128];
};
}  // namespace goldobot
