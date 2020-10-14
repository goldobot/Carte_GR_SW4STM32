#pragma once
#include "goldobot/odrive/odrive_stream_parser.hpp"
#include "goldobot/odrive/odrive_stream_writer.hpp"
#include "goldobot/platform/message_queue.hpp"
#include "goldobot/platform/task.hpp"

#include <cstdint>

namespace goldobot {
class ODriveCommTask : public Task {
 public:
  ODriveCommTask();
  const char* name() const override;

 private:
  void taskFunction() override;

  void processMessage();

  ODriveStreamWriter m_stream_writer;
  ODriveStreamParser m_stream_parser;
  MessageQueue m_message_queue;

  static unsigned char s_message_queue_buffer[1024];
  static unsigned char s_write_buffer[256];
  static unsigned char s_parse_buffer[256];
  static unsigned char s_scratchpad[128];
};
}  // namespace goldobot
