#pragma once
#include "goldobot/platform/task.hpp"
#include "goldobot/platform/message_queue.hpp"

#include <cstdint>

namespace goldobot {
class HeartbeatTask : public Task {
 public:
  HeartbeatTask();
  const char* name() const override;

 private:
  void taskFunction() override;

  void checkGpioState();

  uint32_t m_gpio_state{0};
  uint32_t m_fpga_gpio_state{0};
  bool m_fpga_gpio_state_changed{false};

  MessageQueue m_message_queue;
  unsigned char m_message_queue_buffer[128];
};
}  // namespace goldobot
