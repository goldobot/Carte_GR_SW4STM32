#pragma once
#include "goldobot/config.hpp"

#include <cstdint>

namespace goldobot {
class LiftController {
 public:
  enum class State { Init, InitDisabled, HomingWaitPwm, Homing, HomedWaitPosition, Homed };
  enum class Register { Cmd, Status, Position, Debug, MotorPwm };

 public:
  void init(int id, const LiftConfig& config, uint8_t* scratchpad);

  void update(bool enable, uint16_t pos, float speed, uint8_t torque);
  void doHoming();
  void setTimestamp(uint32_t ts);
  void onFpgaReadStatus(uint32_t apb_address, uint32_t value);
  void onRegRead(Register reg, uint32_t value);
  void regWrite(Register reg, uint32_t value);
  bool homingFinished();

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

  uint32_t m_position;

 private:
  static const uint32_t c_lift_base[2];
  static const uint32_t c_motor_apb[2];

  LiftConfig m_config;
  uint32_t m_apb_base;
  uint8_t* m_scratchpad;
  int m_id;
  State m_state;
  uint32_t m_status{0};
  uint32_t m_timestamp{0};
  uint32_t m_next_ts{0};
  bool m_enable{false};
  bool m_homing_finished{false};
};
}  // namespace goldobot
