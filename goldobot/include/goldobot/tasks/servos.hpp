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
  void processMessageCommand();

  void updateServo(int id, uint16_t pos, uint16_t speed, uint8_t torque);
  void updateServoDynamixelAX12(int id, bool enabled, uint16_t pos, float speed, uint8_t torque);
  void updateServoDynamixelMX28(int id, bool enabled, uint16_t pos, float speed, uint8_t torque);
  void updateServoGoldoLift(int id, bool enabled, uint16_t pos, float speed, uint8_t torque);

  void publishServoState(int id, bool state);

  void moveMultiple(int num_servos);

  void checkSynchronization();

  void processDynamixelResponse();
  void onFpgaReadRegStatus();

  bool isEnabled(int id) const noexcept { return (m_servo_enabled & (1 << id)) != 0; };
  void setEnabled(int id, bool enabled) noexcept {
    m_servo_enabled = ((m_servo_enabled & (0xffff - (1 << id)))) | (enabled ? (1 << id) : 0);
  };

  MessageQueue m_message_queue;
  MessageQueue m_message_queue_commands;
  unsigned char m_message_queue_buffer[256];
  unsigned char m_message_queue_commands_buffer[256];
  uint8_t m_scratchpad[128];

  ServosConfig* m_servos_config{nullptr};
  static constexpr int c_max_num_servos = 32;
  static constexpr int c_update_period = 30;  // update period in ms
  static constexpr uint32_t c_fpga_servos_base = 0x80008404;
  static const uint32_t c_lift_base[2];

  float m_servos_positions[32];
  uint16_t m_servos_speeds[32];
  uint8_t m_servos_torques[32];
  uint16_t m_servos_target_positions[32];
  uint16_t m_servos_measured_positions[32];

  uint32_t m_servo_enabled{0};
  uint32_t m_servo_moving{0};

  bool m_lift_initialized[2] = {false, false};
  bool m_lift_homed[2] = {false, false};
  uint8_t m_lift_servo_id[2] = {0, 0};
  uint16_t m_lift_bltrig = 80;

  uint32_t m_next_statistics_ts{10};
};
}  // namespace goldobot
