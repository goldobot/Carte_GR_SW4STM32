#include "goldobot/lift_controller.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/message_types.hpp"
#include <cstring>
#include <algorithm>

using namespace goldobot;

const uint32_t LiftController::c_lift_base[2] = {0x80008500, 0x80008510};
const uint32_t LiftController::c_motor_apb[2] = {0x80008494, 0x8000849c};

void LiftController::init(int id, const LiftConfig& config, uint8_t *scratchpad) {
  m_config = config;
  m_apb_base = c_lift_base[id];
  m_scratchpad = scratchpad;
  m_id = id;
  m_state = State::Init;
}

void LiftController::setTimestamp(uint32_t ts) { m_timestamp = ts; }

void LiftController::update(bool enable, uint16_t pos, float speed, uint8_t torque) {
  // Schedule read status
  // read status and position
  uint32_t buff[1] = {m_apb_base + 4};
  Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaReadRegInternal,
                                                 (unsigned char *)buff, 4);
  buff[0] = m_apb_base + 8;
  Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaReadRegInternal,
                                                 (unsigned char *)buff, 4);
  if (m_state == State::Init) {
    cmdSetEnable(false);
  }
  if (m_state == State::Homing) {
    uint32_t homing_pwm[2] = {0x80, 0xffffff80};
    // regWrite(LiftController::Register::MotorPwm, homing_pwm[m_id]);
  }
  if (m_state == State::Homed) {
	  /*
    cmdSetEnable(enable);
    if (enable) {
      // lift_speed in ticks per 10ms
      uint16_t bltrig = 80;
      uint16_t lift_speed = static_cast<uint16_t>(speed * 1.1 / 100) + 1;
      if (lift_speed > 40) lift_speed = 40;

      cmdGotoTarget(pos);
      cmdSetBltrigSpeed(bltrig, lift_speed);
    }*/
  }
}

void LiftController::doHoming() {
  //uint32_t homing_pwm[2] = {0x80, 0xffffff80};
 // regWrite(LiftController::Register::MotorPwm, homing_pwm[m_id]);
  m_state = State::HomingWaitPwm;
  cmdDoHoming();
  m_state = State::Homing;
  m_homing_finished = false;
}

bool LiftController::homingFinished() {
  auto ret = m_homing_finished;
  m_homing_finished = false;
  return ret;
}

void LiftController::onFpgaReadStatus(uint32_t apb_address, uint32_t value) {
  if (apb_address == m_apb_base + 4) {
    // onRegRead(Register::Status, value);
    m_status = value;
    uint32_t state = value & 0xff000000;
    uint32_t flags = value & 0x000000ff;

    if (m_state == State::Init && state == 0) {
      m_state = State::InitDisabled;
    }
    if (m_state == State::Homing && (flags & 0x1)) {
      m_state = State::HomedWaitPosition;
      m_next_ts = m_timestamp + 100;
    }
  }
  if (apb_address == m_apb_base + 8) {
    if (m_state == State::HomedWaitPosition && m_timestamp >= m_next_ts) {
      m_state = State::Homed;
      m_homing_finished = true;
    }
    if (m_state == State::Homed) {
      m_position = value < 0xffff ? value : 0;
    }
  }
}

void LiftController::onRegRead(Register reg, uint32_t value) {}

void LiftController::regsRead() {
  uint32_t apb_address = m_apb_base + 4;
  Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaReadRegInternal,
                                                 (unsigned char *)&apb_address, 4);
  apb_address = m_apb_base + 8;
  Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaReadRegInternal,
                                                 (unsigned char *)&apb_address, 4);
}

void LiftController::regWrite(Register reg, uint32_t value) {
  uint32_t apb_address{m_apb_base};
  switch (reg) {
    case Register::Cmd:
      apb_address += 0;
      break;
    case Register::MotorPwm:
      apb_address = c_motor_apb[m_id];
      break;
    default:
      return;
  }
  uint32_t buff[2] = {apb_address, value};
  Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaWriteReg,
                                                 (unsigned char *)buff, 8);
}

void LiftController::cmdSetEnable(bool enable) {
  regWrite(Register::Cmd, 0x10000000 | (enable ? 1 : 0));
}
void LiftController::cmdSetKp(uint32_t kp) {
  regWrite(Register::Cmd, 0x20000000 | (kp & 0x0fffffff));
}

void LiftController::cmdSetKi(uint32_t ki) {
  regWrite(Register::Cmd, 0x30000000 | (ki & 0x0fffffff));
}

void LiftController::cmdSetKd(uint32_t kd) {
  regWrite(Register::Cmd, 0x40000000 | (kd & 0x0fffffff));
}

void LiftController::cmdDoHoming() { regWrite(Register::Cmd, 0x50000000); }

void LiftController::cmdJumpTarget(int32_t target) {
  regWrite(Register::Cmd, 0x60000000 | (target & 0x0fffffff));
}

void LiftController::cmdGotoTarget(int32_t target) {
  regWrite(Register::Cmd, 0x70000000 | (target & 0x0fffffff));
}

void LiftController::cmdSetRangeClamp(uint16_t range, uint16_t clamp) {
  uint32_t r = (uint32_t)(range & 0xfff) << 16;
  uint32_t c = (uint32_t)(clamp & 0xffff);
  regWrite(Register::Cmd, 0x80000000 | r | c);
}

void LiftController::cmdSetBltrigSpeed(uint16_t bltrig, uint16_t speed) {
  uint32_t t = (uint32_t)(bltrig & 0xfff) << 16;
  uint32_t s = (uint32_t)(speed & 0xffff);
  regWrite(Register::Cmd, 0x90000000 | t | s);
}

void LiftController::cmdResetError() {
  regWrite(Register::Cmd, 0xf0000000);
  // 0x80000000 robot reset reg
}

/*
 * Lift initial state:
 * Asserv disable
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
