#pragma once
#include "goldobot/config.hpp"
#include "goldobot/core/message_queue.hpp"
#include "goldobot/platform/task.hpp"

#include <cstdint>

namespace goldobot {

class ServosTask : public Task {
 public:
  ServosTask();
  const char* name() const override;
  void taskFunction() override;

 private:
  void processMessage();
  void updateServo(int id, uint16_t pos, uint16_t speed, uint8_t torque);

  void publishServoState(int id, bool state);

  void moveMultiple(int num_servos);

  void checkSynchronization();

  void processDynamixelResponse();

  bool isEnabled(int id) const noexcept { return (m_servo_enabled & (1 << id)) != 0; };
  void setEnabled(int id, bool enabled) noexcept {
    m_servo_enabled = (m_servo_enabled & (0xffff - (1 << id))) | (enabled ? (1 << id) : 0);
  };

  MessageQueue m_message_queue;
  unsigned char m_message_queue_buffer[128];
  uint8_t m_scratchpad[128];

  ServosConfig* m_servos_config{nullptr};
  static constexpr int c_max_num_servos = 32;
  static constexpr int c_update_period = 20;  // update period in ms
  static constexpr uint32_t c_fpga_servos_base = 0x80008404;
  static const uint32_t c_lift_base[2];

  float m_servos_positions[32];
  uint16_t m_servos_speeds[32];
  uint8_t m_servos_torques[32];
  uint16_t m_servos_target_positions[32];
  uint16_t m_servos_measured_positions[32];

  uint32_t m_servo_enabled{0};
  uint32_t m_servo_moving{0};

  uint32_t m_next_statistics_ts{10};
};
}  // namespace goldobot
