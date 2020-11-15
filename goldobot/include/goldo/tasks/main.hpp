#pragma once
#include "goldo/message_types.hpp"
#include "goldo/sequence_engine.hpp"
#include "goldo/core/message_queue.hpp"
#include "goldo/tasks/task.hpp"
#include "goldo/propulsion/simple_odometry.hpp"
#include "goldo/propulsion/controller.hpp"

#include <cstdint>

#include "FreeRTOS.h"
#include "semphr.h"


namespace goldobot
{

  class MainTask : public Task
  {
  public:
    MainTask();
    const char* name() const override;

    // Number of seconds before end of match
    int remainingMatchTime();
    void preMatchBegin();
    void preMatchStep();

    SequenceEngine& sequenceEngine() { return m_sequence_engine;};
  private:

    void taskFunction() override;
    void process_message();
    void process_message_config();

    uint32_t m_start_of_match_time;
    MessageQueue m_message_queue;
    unsigned char m_message_queue_buffer[128];
    unsigned char m_scratchpad[128];

    SequenceEngine m_sequence_engine;
  };

}
