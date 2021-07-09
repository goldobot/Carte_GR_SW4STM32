#pragma once
#include "goldobot/odrive/odrive_stream_parser.hpp"
#include "goldobot/odrive/odrive_stream_writer.hpp"
#include "goldobot/core/message_queue.hpp"
#include "goldobot/platform/task.hpp"

#include <cstdint>

namespace goldobot {

struct ODriveCommStats {
	ODriveStreamParser::Statistics parser;
	ODriveStreamWriter::Statistics writer;
	MessageQueue::Statistics queue;
};

class ODriveCommTask : public Task {
 public:
  ODriveCommTask();
  const char* name() const override;

 private:
  void taskFunction() override;

  bool processMessage();
  void sendStatistics();

  MessageQueue m_message_queue;
  ODriveStreamParser m_stream_parser;
  ODriveStreamWriter m_stream_writer;

  uint32_t m_next_statistics_ts{0};

  static unsigned char s_message_queue_buffer[1024];
  static unsigned char s_write_buffer[512];
  static unsigned char s_parse_buffer[512];
  static unsigned char s_scratchpad[128];

  int m_no_write_cnt{0};
  int m_no_read_cnt{0};
};
}  // namespace goldobot
