#pragma once
#include "goldobot/config.hpp"
#include "goldobot/core/message_queue.hpp"
#include "goldobot/platform/task.hpp"

#include <cstdint>

namespace goldobot {

class LiftController {
public:
	enum class State {

	};
	enum class Register {
		Cmd,
		Status,
		Position,
		Debug
	};
public:
	void init(int id, uint8_t* scratchpad);
	void setEnable(bool enable);

	void update(uint16_t pos, float speed, uint8_t torque);



	void onFpgaReadStatus(uint32_t apb_address, uint32_t value);
	void onRegRead(Register reg, uint32_t value);
	void regWrite(Register reg, uint32_t value);

	void regsRead();

	void cmdSetEnable(bool enable);
	// 28 bits fixed point, 12 bits integer part, 16 fractional
	void cmdSetKp(uint32_t kp);
	void cmdSetKi(uint32_t ki);
	void cmdSetKd(uint32_t kd);
	void cmdDoHoming();
	void cmdJumpTarget(int32_t target);
	void cmdGotoTarget(int32_t target);
	void cmdSetRangeClamp(uint16_t range, uint16_t clamp);
	void cmdSetBltrigSpeed(uint16_t bltrig, uint16_t speed);
	void cmdResetError();

	// homing done bit 0x1
	// position a max range, donc faire set range clamp avant
	// convention range: 0 en bas, course maximale haut


private:
	static const uint32_t c_lift_base[2];

	uint32_t m_apb_base;
	uint8_t* m_scratchpad;
	bool m_enable{false};
};



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

  LiftController m_lifts[2];

  uint8_t m_lift_servo_id[2] = {0, 0};

  uint32_t m_next_statistics_ts{10};
};
}  // namespace goldobot
