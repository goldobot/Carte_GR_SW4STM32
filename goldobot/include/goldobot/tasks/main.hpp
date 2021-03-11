#pragma once
#include "goldobot/message_types.hpp"
#include "goldobot/core/message_queue.hpp"
#include "goldobot/platform/task.hpp"
#include "goldobot/propulsion/controller.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/sequence_engine.hpp"

#include <cstdint>

namespace goldobot {
class MainTask : public Task {
 public:
  MainTask();
  const char* name() const override;

  // Number of seconds before end of match
  int remainingMatchTime();
  void preMatchBegin();
  void preMatchStep();

  SequenceEngine& sequenceEngine() { return m_sequence_engine; };

 private:
  void taskFunction() override;
  void process_message();
  void process_message_config();

  bool m_match_timer_running{false};
  uint32_t m_start_of_match_time;
  uint32_t m_cnt{0};
  MessageQueue m_message_queue;
  static unsigned char s_message_queue_buffer[2048];
  unsigned char m_scratchpad[128];

  SequenceEngine m_sequence_engine;
};
}  // namespace goldobot
