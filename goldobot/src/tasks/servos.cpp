#include "goldobot/tasks/servos.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/utils/update_timestamp.hpp"

#include <cstring>
#include <algorithm>

using namespace goldobot;

const uint32_t ServosTask::c_lift_base[2] = {0x80008500, 0x80008510};

ServosTask::ServosTask()
    : m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer)),
	  m_message_queue_commands(m_message_queue_commands_buffer, sizeof(m_message_queue_commands_buffer)){}

const char *ServosTask::name() const { return "servos"; }

void ServosTask::taskFunction() {
  set_priority(4);
  Robot::instance().mainExchangeIn().subscribe({40, 49, &m_message_queue_commands});
  Robot::instance().exchangeInternal().subscribe({31, 31, &m_message_queue});
  Robot::instance().exchangeInternal().subscribe({61, 61, &m_message_queue});

  m_servos_config = Robot::instance().servosConfig();

  for (unsigned i = 0; i < m_servos_config->num_servos; i++) {
    m_servos_positions[i] = -1;
    m_servos_target_positions[i] = 0;
    m_servos_torques[i] = m_servos_config->servos[i].max_torque;
  }

  int cnt = 0;

  while (1) {
    while (m_message_queue.message_ready()) {
      processMessage();
    }
    while (m_message_queue_commands.message_ready()) {
      processMessageCommand();
    }

    // Recompute servo targets
    float delta_t = c_update_period * 1e-3;
    auto old_moving = m_servo_moving;
    m_servo_moving = 0;

    for (int i = 0; i < m_servos_config->num_servos; i++) {
      // skip uninitialized servos
      if (m_servos_positions[i] < 0) {
        updateServo(i, static_cast<uint16_t>(m_servos_positions[i]), m_servos_speeds[i],
                    m_servos_torques[i]);
        continue;
      }

      bool was_moving = false;

      if(isEnabled(i)) {
        auto position = m_servos_positions[i];
        auto target_position = m_servos_target_positions[i];

        if (position > target_position) {
          m_servos_positions[i] =
              std::max<float>(target_position, position - m_servos_speeds[i] * delta_t);
          was_moving = true;
        }

        if (position < target_position) {
          m_servos_positions[i] =
              std::min<float>(target_position, position + m_servos_speeds[i] * delta_t);
          was_moving = true;
        }
      }
      m_servo_moving |= was_moving ? (1 << i) : 0;
      if (was_moving && m_servos_positions[i] == m_servos_target_positions[i]) {
        publishServoState(i, false);
      }
      updateServo(i, static_cast<uint16_t>(m_servos_positions[i]), m_servos_speeds[i],
                  m_servos_torques[i]);
    }

    if (old_moving != m_servo_moving || cnt == 0) {
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ServosMoving,
                                                      (unsigned char *)&m_servo_moving, 4);
    }

    cnt++;

    if (cnt == 16) {
      cnt = 0;
    }

    delay_periodic(c_update_period);
  } /* while(1) */
}

void ServosTask::updateServo(int id, uint16_t pos, uint16_t speed, uint8_t torque) {
  const auto &config = m_servos_config->servos[id];
  bool enabled = isEnabled(id);

  // fpga servo
  if (config.type == ServoType::StandardServo) {
    uint32_t servo_pwm = enabled ? static_cast<uint32_t>(pos) << 2 : 0;
    uint32_t buff[2] = {c_fpga_servos_base + 8 * config.id, servo_pwm};
    Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaWriteReg,
                                                   (unsigned char *)buff, 8);
  }

  if (config.type == ServoType::GoldoLift) {
	  m_lift_servo_id[config.id] = id;

	  if(m_lift_initialized[config.id] && m_lift_homed[config.id])
	  {
    	  // enable control
		  uint32_t reg_val_e = enabled ? 0x10000001 : 0x10000001;
		  uint32_t buff[2] = {c_lift_base[config.id], reg_val_e};
		  Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaWriteReg,
														 (unsigned char *)buff, 8);
		  if (enabled) {
		        // lift goto_target
		        uint32_t reg_val_p = 0x70000000 | (0x0fffffff & static_cast<uint32_t>(pos));
		        buff[1] = reg_val_p;
		        Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaWriteReg,
		                                                       (unsigned char *)buff, 8);
		        // set speed
		        uint16_t lift_speed = (speed / 100) + 1;
		        buff[1] = 0x90000000 | speed | ((m_lift_bltrig & 0xfff) << 16);
		      }
	  }
	  // read status and position
	  uint32_t buff[1] = {c_lift_base[config.id] + 4};
	  Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaReadRegInternal,
													 (unsigned char *)buff, 4);
	  buff[0] = c_lift_base[config.id] + 8;
	  Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaReadRegInternal,
	  													 (unsigned char *)buff, 4);
  }

  if (config.type == ServoType::DynamixelAX12) {
    // one pos unit = 0.29 deg
    // one speed unit is about 0.111 rpm, or 0.666 dps, or 2.3 pos units per second
    // speed = 0 correspond to max rpm in dynamixel, so add one
    uint16_t dyna_speed = static_cast<uint16_t>(speed * 1.1f / 2.3f) + 1;
    if (dyna_speed > 0x3ff) {
      dyna_speed = 0x3ff;
    }
    uint16_t dyna_torque = ((uint32_t)torque * 0x3ff) / 0xff;
    uint8_t buff[12];
    *reinterpret_cast<uint16_t *>(buff + 0) =
        0x8000;                                  // msb=1 => send response to internal exchange
    *reinterpret_cast<uint8_t *>(buff + 2) = 1;  // protocol version
    *reinterpret_cast<uint8_t *>(buff + 3) = config.id;
    *reinterpret_cast<uint8_t *>(buff + 4) = 0x03;  // write

    // torque enable
    *reinterpret_cast<uint8_t *>(buff + 5) = 24;  // torque enable register
    *reinterpret_cast<uint16_t *>(buff + 6) = enabled ? 1 : 0;
    Robot::instance().mainExchangeIn().pushMessage(CommMessageType::DynamixelsRequest,
                                                   (unsigned char *)buff, 7);
    if (enabled) {
      *reinterpret_cast<uint8_t *>(buff + 5) = 30;  // position, speed, torque registers
      *reinterpret_cast<uint16_t *>(buff + 6) = pos;
      *reinterpret_cast<uint16_t *>(buff + 8) = dyna_speed;
      *reinterpret_cast<uint16_t *>(buff + 10) = dyna_torque;
      Robot::instance().mainExchangeIn().pushMessage(CommMessageType::DynamixelsRequest,
                                                     (unsigned char *)buff, 12);
    }

    // request state
    *reinterpret_cast<uint16_t *>(buff + 0) =
        (36 << 8) | id | 0x8000;                    // msb=1 => send response to internal exchange
    *reinterpret_cast<uint8_t *>(buff + 4) = 0x02;  // read

    *reinterpret_cast<uint8_t *>(buff + 5) = 36;  // present position, speed, load
    *reinterpret_cast<uint8_t *>(buff + 6) = 6;
    Robot::instance().mainExchangeIn().pushMessage(CommMessageType::DynamixelsRequest,
                                                   (unsigned char *)buff, 7);
  }

  if (config.type == ServoType::DynamixelMX28) {
    // one pos unit = 0.088  deg
    // one speed unit is about 0.114 rpm, or 0.684 dps, or 7.773 pos units per second
    // speed = 0 correspond to max rpm in dynamixel, so add one
    uint16_t dyna_speed = static_cast<uint16_t>(speed * 1.1f / 7.773f) + 1;
    if (dyna_speed > 0x3ff) {
      dyna_speed = 0x3ff;
    }
    uint16_t dyna_torque = (torque * 0x3ff) / 0xff;  // dynamixel max torque = 0x3ff or 1023
    uint8_t buff[12];
    *reinterpret_cast<uint16_t *>(buff + 0) =
        0x8000;                                  // msb=1 => send response to internal exchange
    *reinterpret_cast<uint8_t *>(buff + 2) = 1;  // protocol version
    *reinterpret_cast<uint8_t *>(buff + 3) = config.id;
    *reinterpret_cast<uint8_t *>(buff + 4) = 0x03;  // write

    // torque enable
    *reinterpret_cast<uint8_t *>(buff + 5) = 24;  // torque enable register
    *reinterpret_cast<uint16_t *>(buff + 6) = enabled ? 1 : 0;
    Robot::instance().mainExchangeIn().pushMessage(CommMessageType::DynamixelsRequest,
                                                   (unsigned char *)buff, 7);
    if (enabled) {
      *reinterpret_cast<uint8_t *>(buff + 5) = 30;  // position, speed, torque registers
      *reinterpret_cast<uint16_t *>(buff + 6) = pos;
      *reinterpret_cast<uint16_t *>(buff + 8) = dyna_speed;
      *reinterpret_cast<uint16_t *>(buff + 10) = dyna_torque;
      Robot::instance().mainExchangeIn().pushMessage(CommMessageType::DynamixelsRequest,
                                                     (unsigned char *)buff, 12);
    }

    // request state
    *reinterpret_cast<uint16_t *>(buff + 0) =
        (36 << 8) | id | 0x8000;                    // msb=1 => send response to internal exchange
    *reinterpret_cast<uint8_t *>(buff + 4) = 0x02;  // read

    *reinterpret_cast<uint8_t *>(buff + 5) = 36;  // present position, speed, load
    *reinterpret_cast<uint8_t *>(buff + 6) = 6;
    Robot::instance().mainExchangeIn().pushMessage(CommMessageType::DynamixelsRequest,
                                                   (unsigned char *)buff, 7);
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
    case CommMessageType::DynamixelsResponse: {
      auto msg_size = m_message_queue.pop_message(m_scratchpad, sizeof(m_scratchpad));
      uint16_t seq = *reinterpret_cast<uint16_t *>(m_scratchpad);
      if (msg_size == 11) {
        // position, speed, load sread response
        int id = seq & 0xff;
        uint16_t position = *reinterpret_cast<uint16_t *>(m_scratchpad + 5);
        //uint16_t speed = *reinterpret_cast<uint16_t *>(m_scratchpad + 7);
        //uint16_t load = *reinterpret_cast<uint16_t *>(m_scratchpad + 9);
        //uint8_t error = m_scratchpad[4];
        m_servos_measured_positions[id] = position;
        if(m_servos_positions[id] < 0)
        {
        	m_servos_positions[id] = position;
        	m_servos_target_positions[id] = position;
        }
        if (!isEnabled(id)) {
          m_servos_positions[id] = position;
        }
      }
    } break;
    case CommMessageType::FpgaReadRegStatus:
    	m_message_queue.pop_message(m_scratchpad, 8);
    	onFpgaReadRegStatus();
    	break;
    default:
      m_message_queue.pop_message(nullptr, 0);
      break;
  };
}

void ServosTask::processMessageCommand() {
  auto message_type = (CommMessageType)m_message_queue_commands.message_type();

  switch (message_type) {
    case CommMessageType::ServoDisableAll:
      for (int i = 0; i < m_servos_config->num_servos; i++) {
        m_servo_enabled = 0;
      }
      m_message_queue_commands.pop_message(nullptr, 0);
      break;
    case CommMessageType::ServoSetEnable: {
    	m_message_queue_commands.pop_message(m_scratchpad, sizeof(m_scratchpad));
      int id = m_scratchpad[0];
      setEnabled(id, m_scratchpad[1] != 0);
    } break;

    case CommMessageType::ServoMove: {
      unsigned char buff[5];
      m_message_queue_commands.pop_message(buff, 5);
      int servo_id = buff[0];
      int pwm = *(uint16_t *)(buff + 1);
      int target_speed = *(uint16_t *)(buff + 3);
      auto &cfg = m_servos_config->servos[servo_id];
      if (pwm < cfg.cw_limit) {
        pwm = cfg.cw_limit;
      }
      if (pwm > cfg.ccw_limit) {
        pwm = cfg.ccw_limit;
      }
      if (m_servos_positions[servo_id] < 0) {
        m_servos_positions[servo_id] = pwm;
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
    case CommMessageType::ServoMoveMultiple: {
      auto msg_size = m_message_queue_commands.pop_message(m_scratchpad, sizeof(m_scratchpad));
      moveMultiple((msg_size - 4) / 3);
      // send ack with sequence number
      Robot::instance().mainExchangeOutPrio().pushMessage(CommMessageType::ServoMoveMultiple,
                                                          (unsigned char *)m_scratchpad, 2);
    } break;
    case CommMessageType::ServoSetLiftEnable:
    {
    	m_message_queue_commands.pop_message(m_scratchpad, 4);
    	uint16_t sequence_number = *(uint16_t*)m_scratchpad;
    	uint8_t id_ = m_scratchpad[2];
        m_lift_initialized[id_] = m_scratchpad[3] != 0;
    }
        break;
    case CommMessageType::ServoLiftDoHoming:
    {
    	m_message_queue_commands.pop_message(m_scratchpad, 3);
    	uint16_t sequence_number = *(uint16_t*)m_scratchpad;
    	uint8_t id_ = m_scratchpad[2];

		uint32_t buff[2] = {c_lift_base[id_], 0x50000000 };
		Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaWriteReg,
		  										 (unsigned char *)buff, 8);
    }
    break;
    case CommMessageType::ServoGetState:
	{
		m_message_queue_commands.pop_message(m_scratchpad, 3);
		uint16_t sequence_number = *(uint16_t*)m_scratchpad;
		uint8_t id_ = m_scratchpad[2];
		*(uint16_t*)(m_scratchpad + 3) = m_servos_measured_positions[id_];
		Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ServoGetState,
				  										 (unsigned char *)m_scratchpad, 5);
	}
    case CommMessageType::ServoSetMaxTorque:
	{
		/*m_message_queue_commands.pop_message(m_scratchpad, 3);
		uint16_t sequence_number = *(uint16_t*)m_scratchpad;
		uint8_t id_ = m_scratchpad[2];
		*(uint16_t*)(m_scratchpad + 3) = m_servos_measured_positions[id_];
		Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ServoGetState,
														 (unsigned char *)m_scratchpad, 5);*/
	}
	break;
    default:
      m_message_queue_commands.pop_message(nullptr, 0);
      break;
  };
}

void ServosTask::checkSynchronization() {}

void ServosTask::moveMultiple(int num_servos) {
  float speed = (*reinterpret_cast<uint16_t *>(m_scratchpad + 2) / 1023.f);
  if (speed > 1) {
    speed = 1;
  };
  float move_duration = 1e-3f;  // minimal move duration is 1ms to prevent division by zero
  // compute the duration of the move, limited by the slowest servo
  for (int i = 0; i < num_servos; i++) {
    uint8_t *ptr = &m_scratchpad[4 + 3 * i];
    uint8_t id = *reinterpret_cast<uint8_t *>(ptr);
    uint16_t target = *reinterpret_cast<uint16_t *>(ptr + 1);
    const auto &config = m_servos_config->servos[id];

    if (target < config.cw_limit) {
      target = config.cw_limit;
      *reinterpret_cast<uint16_t *>(ptr + 1) = target;
    }
    if (target > config.ccw_limit) {
      target = config.ccw_limit;
      *reinterpret_cast<uint16_t *>(ptr + 1) = target;
    }

    float diff = m_servos_positions[id] >= 0 ? target - m_servos_positions[id] : 0;
    float t = fabsf(diff / (config.max_speed * speed));
    if (t > move_duration) {
      move_duration = t;
    }
  }

  float move_duration_inv = 1.0f / move_duration;

  // update each servo
  for (int i = 0; i < num_servos; i++) {
    uint8_t *ptr = &m_scratchpad[4 + 3 * i];
    uint8_t id = *reinterpret_cast<uint8_t *>(ptr);
    uint16_t target = *reinterpret_cast<uint16_t *>(ptr + 1);
    const auto &config = m_servos_config->servos[id];

    if (m_servos_positions[id] < 0) {
      m_servos_speeds[id] = config.max_speed;
      m_servos_positions[id] = target;
    } else {
      float diff = target - m_servos_positions[id];
      int servo_speed = static_cast<int>(fabsf(diff * move_duration_inv));
      if (servo_speed > 0xffff) {
        servo_speed = 0xffff;
      }
      m_servos_speeds[id] = servo_speed;
    }
    m_servos_target_positions[id] = target;
  }
}

void ServosTask::onFpgaReadRegStatus()
{
	uint32_t apb_address = *reinterpret_cast<uint32_t*>(m_scratchpad);
	uint32_t apb_value = *reinterpret_cast<uint32_t*>(m_scratchpad + 4);
	for(int i= 0; i < 2; i++)
	{
		if(apb_address == c_lift_base[i] + 4)
		{
			m_lift_homed[i] = (apb_value & 0x1) != 0;
		}
		if(apb_address == c_lift_base[i] + 8)
		{
			auto id_ = m_lift_servo_id[i];
			m_servos_measured_positions[id_] = apb_value & 0xffff;
		}
	}
}
