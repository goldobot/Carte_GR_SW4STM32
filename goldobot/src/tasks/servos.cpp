#include "goldobot/tasks/servos.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/utils/update_timestamp.hpp"

#include <cstring>
#include <algorithm>

using namespace goldobot;

const uint32_t ServosTask::c_lift_base[2] = {0x80008500, 0x80008510};

ServosTask::ServosTask()
    : m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer)) {}

const char *ServosTask::name() const { return "servos"; }

void ServosTask::taskFunction() {
  set_priority(4);
  Robot::instance().mainExchangeIn().subscribe({40, 49, &m_message_queue});
  Robot::instance().exchangeInternal().subscribe({31, 31, &m_message_queue});

  m_servos_config = Robot::instance().servosConfig();

  for (unsigned i = 0; i < c_max_num_servos; i++) {
    m_servos_positions[i] = -1;
    m_servos_target_positions[i] = 0;
  }

  while (1) {
    while (m_message_queue.message_ready()) {
      processMessage();
    }

    // Recompute servo targets
    float delta_t = c_update_period * 1e-3;

    for (int i = 0; i < m_servos_config->num_servos; i++) {
      // skip uninitialized servos
      if (m_servos_positions[i] < 0) {
        continue;
      }

      bool was_moving = false;

      auto position = m_servos_positions[i];
      auto target_position = m_servos_target_positions[i];

      if (position > target_position) {
        m_servos_positions[i] =
            std::max<float>(target_position, position - m_servos_speeds[i] * delta_t);
        updateServo(i, static_cast<uint16_t>(m_servos_positions[i]), m_servos_speeds[i], m_servos_torques[i]);
        was_moving = true;
      }

      if (position < target_position) {
        m_servos_positions[i] =
            std::min<float>(target_position, position + m_servos_speeds[i] * delta_t);
        updateServo(i, static_cast<uint16_t>(m_servos_positions[i]), m_servos_speeds[i], m_servos_torques[i]);
        was_moving = true;
      }

      if (was_moving && m_servos_positions[i] == m_servos_target_positions[i]) {
        publishServoState(i, false);
      }
    }

    uint8_t watchdog_id = 5;
    Robot::instance().exchangeInternal().pushMessage(CommMessageType::WatchdogReset, &watchdog_id,
                                                     1);

    delay_periodic(c_update_period);
  } /* while(1) */
}

void ServosTask::updateServo(int id, uint16_t pos, uint16_t speed, uint16_t torque) {
  const auto &config = m_servos_config->servos[id];
  // fpga servo
  if (config.type == ServoType::StandardServo) {
    uint32_t buff[2] = {c_fpga_servos_base + 8 * config.id, static_cast<uint32_t>(pos) << 2};
    Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaWriteReg,
                                                   (unsigned char *)buff, 8);
  }
  if (config.type == ServoType::GoldoLift)
  {
	  uint32_t reg_val = 0x70000000 | (0x0fffffff & static_cast<uint32_t>(pos));
	  uint32_t buff[2] = {c_lift_base[config.id], static_cast<uint32_t>(reg_val) << 2};
	  Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaWriteReg,
	                                                     (unsigned char *)buff, 8);
  }
  if(config.type == ServoType::DynamixelAX12)
  {
	  // one pos unit = 0.29 deg
	  // one speed unit is about 0.111 rpm, or 0.666 dps, or 2.3 pos units per second
	  // speed = 0 correspond to max rpm in dynamixel, so add one
	  uint16_t dyna_speed = static_cast<uint16_t>(speed * 1.1f / 2.3f) + 1;
	  uint8_t buff[2];
	  *reinterpret_cast<uint16_t*>(buff + 0) = 0x8000; // msb=1 => send response to internal exchange
	  *reinterpret_cast<uint8_t*>(buff + 2) = 1;//protocol version
	  *reinterpret_cast<uint8_t*>(buff + 3) = config.id;
	  *reinterpret_cast<uint8_t*>(buff + 4) = 0x03; // write
	  *reinterpret_cast<uint16_t*>(buff + 5) = pos;
	  *reinterpret_cast<uint16_t*>(buff + 5) = dyna_speed;
	  *reinterpret_cast<uint16_t*>(buff + 5) = torque;

	  Robot::instance().mainExchangeIn().pushMessage(CommMessageType::DynamixelsRequest,
	  	                                                     (unsigned char *)buff, 8);
  }
}

void ServosTask::publishServoState(int servo_id, bool state) {
  unsigned char buff[2] = {(unsigned char)servo_id, true};
  Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ServoState,
                                                  (unsigned char *)buff, 2);
  Robot::instance().exchangeInternal().pushMessage(CommMessageType::ServoState,
                                                   (unsigned char *)buff, 2);
}

void ServosTask::processMessage() {
  auto message_type = (CommMessageType)m_message_queue.message_type();

  switch (message_type) {
    case CommMessageType::ServoMove: {
      unsigned char buff[5];
      m_message_queue.pop_message(buff, 5);
      int servo_id = buff[0];
      int pwm = *(uint16_t *)(buff + 1);
      int target_speed = *(uint16_t *)(buff + 3);
      auto& cfg = m_servos_config->servos[servo_id];
      if(pwm < cfg.cw_limit)
      {
    	  pwm = cfg.cw_limit;
      }
      if(pwm > cfg.ccw_limit)
      {
    	  pwm = cfg.cw_limit;
      }
      if (m_servos_positions[servo_id] < 0) {
        m_servos_positions[servo_id] = pwm;
        updateServo(servo_id, pwm, m_servos_config->servos[servo_id].max_speed, 0x1023);
      }
      // Send message when servo start moving
      bool is_moving = m_servos_positions[servo_id] >= 0 &&
                       m_servos_target_positions[servo_id] != m_servos_positions[servo_id];
      bool target_changed = m_servos_target_positions[servo_id] != pwm;
      if (target_changed && !is_moving) {
        publishServoState(servo_id, true);
      }
      m_servos_target_positions[servo_id] = pwm;
      m_servos_speeds[servo_id] =
          (m_servos_config->servos[servo_id].max_speed * target_speed) / 100;
    } break;
    default:
      m_message_queue.pop_message(nullptr, 0);
      break;
  };
}
