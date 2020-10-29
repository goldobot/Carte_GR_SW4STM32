#pragma once
#include "goldobot/config.hpp"
#include "goldobot/platform/message_queue.hpp"
#include "goldobot/platform/task.hpp"

#include <cstdint>

namespace goldobot {

class ServosTask : public Task {
 public:
  ServosTask();
  const char *name() const override;
  void taskFunction() override;

 private:

  void processMessage();
  void updateServo(int id, uint16_t pos, uint16_t speed);

  void publishServoState(int id, bool state);

  MessageQueue m_message_queue;
  unsigned char m_message_queue_buffer[128];

  ServosConfig* m_servos_config{nullptr};
  static constexpr int c_max_num_servos = 32;
  static constexpr int c_update_period = 20; //update period in ms
  static constexpr uint32_t c_fpga_servos_base = 0x80008404;
  float m_servos_positions[32];
  uint16_t m_servos_speeds[32];
  uint16_t m_servos_target_positions[32];
};
}  // namespace goldobot


