#include "goldobot/tasks/servos.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/message_types.hpp"
#include "goldobot/utils/update_timestamp.hpp"

#include <cstring>
#include <algorithm>

using namespace goldobot;

const uint32_t ServosTask::c_lift_base[2] = {0x80008500, 0x80008510};

ServosTask::ServosTask()
    : m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer)),
      m_message_queue_commands(m_message_queue_commands_buffer,
                               sizeof(m_message_queue_commands_buffer)) {}

const char *ServosTask::name() const { return "servos"; }

void ServosTask::taskFunction() {
  set_priority(2);
  Robot::instance().mainExchangeIn().subscribe({40, 49, &m_message_queue_commands});
  Robot::instance().mainExchangeIn().subscribe({230, 239, &m_message_queue_commands}); /* FIXME : TODO : create a separate task for "Lifts" */
  Robot::instance().exchangeInternal().subscribe({31, 31, &m_message_queue});
  Robot::instance().exchangeInternal().subscribe({61, 61, &m_message_queue});

  m_servos_config = Robot::instance().servosConfig();
  m_lifts_config = Robot::instance().liftsConfig();

  for (unsigned i = 0; i < m_servos_config->num_servos; i++) {
    auto &config = m_servos_config->servos[i];
    m_servos_positions[i] = 0;
    m_servos_target_positions[i] = 0;
    m_servos_target_timestamps[i] = 0;
    m_servos_torques[i] = 0xff;
    if (config.type == ServoType::GoldoLift) {
      m_lift_servo_id[config.id] = i;
      setInitialized(i, true);
    }
  }

  if(m_lifts_config != nullptr) {
  if(m_lifts_config->num_lifts > 2) {
      m_lifts_config->num_lifts = 2;
    }
    for (unsigned i = 0; i < m_lifts_config->num_lifts; i++) {
      m_lifts[i].init(i, m_lifts_config->lifts[i], m_scratchpad);
    }
  }

  int cnt = 0;

  while (1) {
    m_current_timestamp = hal::get_tick_count();
    for (unsigned i = 0; i < 2; i++) {
      m_lifts[i].setTimestamp(m_current_timestamp);
    }
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
      if (!isInitialized(i) && (i + cnt) % 2) {
        updateServo(i, static_cast<uint16_t>(m_servos_positions[i]), m_servos_speeds[i],
                    m_servos_torques[i]);
        continue;
      }

      auto& config = m_servos_config->servos[i];

      bool was_moving = false;

      if (isEnabled(i)) {
        // move servo position at speed
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
      } else {
        // if servo is disabled, position and target position track measured positions

        m_servos_target_positions[i] = m_servos_measured_positions[i];
        m_servos_positions[i] = m_servos_measured_positions[i];
      }

      if(m_servos_positions[i] < config.cw_limit) {
        m_servos_positions[i] = config.cw_limit;
      }
      if(m_servos_positions[i] > config.ccw_limit) {
        m_servos_positions[i] = config.ccw_limit;
      }

      m_servo_moving |= was_moving ? (1 << i) : 0;
      if ((i + cnt) % 4 == 0)
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

    if (m_current_timestamp >= m_next_telemetry_ts) {
      publishTelemetry();
      m_next_telemetry_ts = std::max(m_next_telemetry_ts + 50, m_current_timestamp);
    }
    delay_periodic(c_update_period);
  }
}

void ServosTask::updateServoDynamixelAX12(int id, bool enabled, uint16_t pos, float speed,
                                          uint8_t torque) {
  const auto &config = m_servos_config->servos[id];
  // one pos unit = 0.29 deg
  // one speed unit is about 0.111 rpm, or 0.666 dps, or 2.3 pos units per second
  // speed = 0 correspond to max rpm in dynamixel, so add one
  uint16_t dyna_speed = static_cast<uint16_t>(speed * 1.1f / 2.3f) + 1;
  if (dyna_speed > 0x3ff) {
    dyna_speed = 0x3ff;
  }
  uint16_t dyna_torque = ((uint32_t)torque * config.max_torque) / 0xff;
  if (dyna_torque > 0x3ff) {
    dyna_torque = 0x3ff;
  }
  uint8_t buff[12];
  *reinterpret_cast<uint16_t *>(buff + 0) = 0x8000;  // msb=1 => send response to internal exchange
  *reinterpret_cast<uint8_t *>(buff + 2) = 1;        // protocol version
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

void ServosTask::updateServoDynamixelMX28(int id, bool enabled, uint16_t pos, float speed,
                                          uint8_t torque) {
  const auto &config = m_servos_config->servos[id];
  // one pos unit = 0.088  deg
  // one speed unit is about 0.114 rpm, or 0.684 dps, or 7.773 pos units per second
  // speed = 0 correspond to max rpm in dynamixel, so add one
  uint16_t dyna_speed = static_cast<uint16_t>(speed * 1.1f / 7.773f) + 1;
  if (dyna_speed > 0x3ff) {
    dyna_speed = 0x3ff;
  }
  uint16_t dyna_torque = (torque * 0x3ff) / 0xff;  // dynamixel max torque = 0x3ff or 1023
  uint8_t buff[12];
  *reinterpret_cast<uint16_t *>(buff + 0) = 0x8000;  // msb=1 => send response to internal exchange
  *reinterpret_cast<uint8_t *>(buff + 2) = 1;        // protocol version
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

void ServosTask::updateServoGoldoLift(int id, bool enabled, uint16_t pos, float speed,
                                      uint8_t torque) {
  const auto &config = m_servos_config->servos[id];
  m_lifts[config.id].update(enabled, pos, speed, torque);
}

void ServosTask::updateServo(int id, uint16_t pos, uint16_t speed, uint8_t torque) {
  const auto &config = m_servos_config->servos[id];
  bool enabled = isEnabled(id);

  if (config.cw_limit > config.ccw_limit) {
    return;
  }
  if (pos < config.cw_limit) {
    pos = config.cw_limit;
  }
  if (pos > config.ccw_limit) {
    pos = config.ccw_limit;
  }

  // fpga servo
  if (config.type == ServoType::StandardServo) {
    uint32_t servo_pwm = enabled ? static_cast<uint32_t>(pos) << 2 : 0;
    uint32_t buff[2] = {c_fpga_servos_base + 8 * config.id, servo_pwm};
    Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaWriteReg,
                                                   (unsigned char *)buff, 8);
  }

  switch (config.type) {
    case ServoType::DynamixelAX12:
      updateServoDynamixelAX12(id, enabled, pos, speed, torque);
      break;
    case ServoType::DynamixelMX28:
      updateServoDynamixelMX28(id, enabled, pos, speed, torque);
      break;
    case ServoType::GoldoLift:
      updateServoGoldoLift(id, enabled, pos, speed, torque);
      break;
    default:
      break;
  }
}

void ServosTask::publishTelemetry() {
  *reinterpret_cast<uint32_t *>(m_scratchpad) = m_current_timestamp;
  for (unsigned i = 0; i < m_servos_config->num_servos; i++) {
    uint8_t *ptr = &m_scratchpad[6 + i * 6];
    *reinterpret_cast<uint16_t *>(ptr) = (float)m_servos_positions[i];
    *reinterpret_cast<uint16_t *>(ptr + 2) = m_servos_measured_positions[i];
    *reinterpret_cast<uint16_t *>(ptr + 4) = m_servos_measured_torques[i];
  }
  Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ServoState,
                                              (unsigned char *)m_scratchpad,
                                              m_servos_config->num_servos * 6 + 6);
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
        uint16_t speed = *reinterpret_cast<uint16_t *>(m_scratchpad + 7);
        uint16_t load = *reinterpret_cast<uint16_t *>(m_scratchpad + 9);
        // uint8_t error = m_scratchpad[4];
        m_servos_measured_positions[id] = position;
        m_servos_measured_speeds[id] = speed < 1024 ? speed : -(speed - 1024);
        m_servos_measured_torques[id] = load < 1024 ? load : -(load - 1024);

        setInitialized(id, true);
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

size_t ServosTask::readCommand(size_t size) {
  auto msg_size = m_message_queue_commands.pop_message(m_scratchpad, size);
  m_sequence_number = *(uint16_t *)m_scratchpad;
  return msg_size;
}

void ServosTask::ackCommand(CommMessageType type, size_t size) {
  *(uint16_t *)m_scratchpad = m_sequence_number;
  Robot::instance().mainExchangeOutPrio().pushMessage(type, (unsigned char *)m_scratchpad, size);
}

void ServosTask::processMessageCommand() {
  auto message_type = (CommMessageType)m_message_queue_commands.message_type();

  switch (message_type) {
    case CommMessageType::ServoDisableAll:
      for (int i = 0; i < m_servos_config->num_servos; i++) {
        m_servo_enabled = 0;
      }
      readCommand(2);
      ackCommand(CommMessageType::ServoAck, 2);
      break;
    case CommMessageType::ServoSetEnable: {
      auto msg_size = readCommand(sizeof(m_scratchpad));
      for (unsigned i = 0; i < (msg_size - 2) / 2; i++) {
        int id = m_scratchpad[2 + i * 2];
        setEnabled(id, m_scratchpad[3 + i * 2] != 0);
      }
      ackCommand(CommMessageType::ServoAck, 2);
    } break;
    case CommMessageType::ServoMoveMultiple: {
      // sequence number is first 2 bytes, + padding
      auto msg_size = readCommand(sizeof(m_scratchpad));
      moveMultiple((msg_size - 4) / 3);
      // send ack with sequence number
      ackCommand(CommMessageType::ServoAck, 2);
    } break;
    case CommMessageType::LiftSetEnable:
    {
      memset(m_scratchpad,0,16);
      readCommand(4);
      uint8_t id_ = m_scratchpad[2];
      bool do_enable = (m_scratchpad[3]!=0);
      m_lifts[id_].cmdSetEnable(do_enable);
      ackCommand(CommMessageType::ServoAck, 2);
    } break;
    case CommMessageType::ServoLiftDoHomingObsolete:
    case CommMessageType::LiftDoHoming:
    {
      readCommand(3);
      uint8_t id_ = m_scratchpad[2];
      m_lifts[id_].doHoming();
      ackCommand(CommMessageType::ServoAck, 2);
    } break;
    case CommMessageType::ServoLiftsCmdRawObsolete:
    case CommMessageType::LiftsCmdRaw:
    {
      readCommand(2 + sizeof(MsgLiftsCmdRaw));
      MsgLiftsCmdRaw cmd;
      memcpy(&cmd, m_scratchpad + 2, sizeof(cmd));

      if(cmd.lift1_speed != 0) {
        m_lifts[0].cmdSetBltrigSpeed(cmd.lift1_bltrig, cmd.lift1_speed);
        m_lifts[0].cmdGotoTarget(cmd.lift1_target);
      }

      if(cmd.lift2_speed != 0) {
        m_lifts[1].cmdSetBltrigSpeed(cmd.lift2_bltrig, cmd.lift2_speed);
        m_lifts[1].cmdGotoTarget(cmd.lift2_target);
      }
      ackCommand(CommMessageType::ServoAck, 2);
    } break;
    case CommMessageType::ServoGetState: {
      m_message_queue_commands.pop_message(m_scratchpad, 3);
      uint8_t id_ = m_scratchpad[2];
      *(uint16_t *)(m_scratchpad + 3) = m_servos_measured_positions[id_];
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ServoGetState,
                                                      (unsigned char *)m_scratchpad, 5);
    }
    case CommMessageType::ServoSetMaxTorques: {
      auto msg_size = readCommand(sizeof(m_scratchpad));
      auto num_commands = (msg_size - 2) / 2;
      for (unsigned i = 0; i < num_commands; i++) {
        uint8_t id_ = m_scratchpad[2 + i * 2];
        uint8_t torque = m_scratchpad[3 + i * 2];
        m_servos_torques[id_] = torque;
      }
      ackCommand(CommMessageType::ServoAck, 2);
    } break;
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

    float diff = isInitialized(id) ? target - m_servos_positions[id] : 0;
    float t = fabsf(diff / (config.max_speed * speed));
    if (t > move_duration) {
      move_duration = t;
    }
  }

  float move_duration_inv = 1.0f / move_duration;

  uint32_t target_timestamp = m_current_timestamp + static_cast<uint32_t>(move_duration * 1000);

  // update each servo
  for (int i = 0; i < num_servos; i++) {
    uint8_t *ptr = &m_scratchpad[4 + 3 * i];
    uint8_t id = *reinterpret_cast<uint8_t *>(ptr);
    uint16_t target = *reinterpret_cast<uint16_t *>(ptr + 1);
    const auto &config = m_servos_config->servos[id];

    if (!isInitialized(id) && config.type == ServoType::StandardServo) {
      m_servos_speeds[id] = config.max_speed;
      m_servos_positions[id] = target;
      setInitialized(id, true);
    } else {
      float diff = target - m_servos_positions[id];
      int servo_speed = static_cast<int>(fabsf(diff * move_duration_inv));
      if (servo_speed > 0xffff) {
        servo_speed = 0xffff;
      }
      m_servos_speeds[id] = servo_speed;
    }
    m_servos_target_positions[id] = target;
    m_servos_target_timestamps[id] = target_timestamp;
  }
}

void ServosTask::onFpgaReadRegStatus() {
  uint32_t apb_address = *reinterpret_cast<uint32_t *>(m_scratchpad);
  uint32_t apb_value = *reinterpret_cast<uint32_t *>(m_scratchpad + 4);
  for (int i = 0; i < 2; i++) {
    m_lifts[i].onFpgaReadStatus(apb_address, apb_value);
    auto id = m_lift_servo_id[i];
    m_servos_measured_positions[id] = m_lifts[i].m_position;
    if (m_lifts[i].homingFinished()) {
      m_servos_target_positions[id] = m_servos_measured_positions[id];
      m_servos_positions[id] = m_servos_measured_positions[id];
      setInitialized(id, true);
      setEnabled(id, true);
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::LiftHomingDone, (uint8_t)i);
    }
  }
}

