#include "goldobot/sequence_engine.hpp"

#include "goldobot/message_types.hpp"
#include "goldobot/robot.hpp"

#include <cstring>

#define FLAG_Z (1 << 0)
#define FLAG_N (1 << 1)

namespace goldobot {

enum Opcode {
  Propulsion_MoveTo = 129,
};

SequenceEngine::SequenceEngine() { m_exchange_commands = &Robot::instance().mainExchangeIn(); }

void SequenceEngine::doStep() {
  switch (m_state) {
    case SequenceState::Executing:
      while (execOp(m_ops[m_pc])) {
      };
      break;
    default:
      return;
  }
}

bool SequenceEngine::execOp(const Op& op) {
  switch (op.opcode) {
    case 0:  // nop
      m_pc++;
      break;
    case 1:  // mov1
      *(int32_t*)(m_vars + 4 * op.arg1) = *(int32_t*)(m_vars + 4 * op.arg2);
      m_pc++;
      break;
    case 2:  // mov2
      *(int32_t*)(m_vars + 4 * op.arg1) = *(int32_t*)(m_vars + 4 * op.arg2);
      *(int32_t*)(m_vars + 4 * (op.arg1 + 1)) = *(int32_t*)(m_vars + 4 * (op.arg2 + 1));
      m_pc++;
      break;
    case 3:  // mov3
      *(int32_t*)(m_vars + 4 * op.arg1) = *(int32_t*)(m_vars + 4 * op.arg2);
      *(int32_t*)(m_vars + 4 * (op.arg1 + 1)) = *(int32_t*)(m_vars + 4 * (op.arg2 + 1));
      *(int32_t*)(m_vars + 4 * (op.arg1 + 2)) = *(int32_t*)(m_vars + 4 * (op.arg2 + 2));
      m_pc++;
      break;
    case 4:  // movi
      *(int32_t*)(m_vars + 4 * op.arg1) = op.arg2;
      m_pc++;
      break;
    case 8:  // addi
      *(int32_t*)(m_vars + 4 * op.arg1) += op.arg2;
      m_pc++;
      break;
    case 9:  // subi
      *(int32_t*)(m_vars + 4 * op.arg1) -= op.arg2;
      m_pc++;
      break;
    case 10:  // add
    {
      auto a = *(int32_t*)(m_vars + 4 * op.arg2);
      auto b = *(int32_t*)(m_vars + 4 * op.arg3);
      *(int32_t*)(m_vars + 4 * op.arg1) = a + b;
    }
      m_pc++;
      break;
    case 11:  // sub
    {
      auto a = *(int32_t*)(m_vars + 4 * op.arg2);
      auto b = *(int32_t*)(m_vars + 4 * op.arg3);
      *(int32_t*)(m_vars + 4 * op.arg1) = a - b;
    }
      m_pc++;
      break;
    case 32:  // delay
      if (m_end_delay == 0) {
        m_end_delay = hal::get_tick_count() + *(int*)(m_vars + 4 * op.arg1);
        return false;
      }
      if (hal::get_tick_count() >= m_end_delay) {
        m_end_delay = 0;
        m_pc++;
        return true;
      }
      return false;
    case 64:  // propulsion.motors_enable
      // Hal::motors_set_enable(true);
      m_pc++;
      return true;
    case 65:  // propulsion.enable
    {
      uint8_t b = true;
      m_exchange_commands->pushMessage(CommMessageType::DbgSetPropulsionEnable, (unsigned char*)&b,
                                       1);
      m_propulsion_state_dirty = true;
      m_pc++;
      return true;
    }
    case 66:  // propulsion.motors_disable
      // Hal::motors_set_enable(false);
      m_pc++;
      return true;
    case 67:  // propulsion.disable
    {
      uint8_t b = false;
      Robot::instance().mainExchangeIn().pushMessage(CommMessageType::DbgSetPropulsionEnable,
                                                     (unsigned char*)&b, 1);
      m_propulsion_state_dirty = true;
      m_pc++;
      return true;
    }

    case 125:  // wait_arm_finished
      if (m_arm_state == ArmState::Idle && !m_arm_state_dirty) {
        m_pc++;
        return true;
      }
      return false;
    case 126:  // wait_movement_finished
      if (!m_propulsion_state_dirty && m_propulsion_state == PropulsionState::Stopped) {
        m_pc++;
        return true;
      }
      return false;
    case 146:  // wait_servo_finished
      if (!m_servo_moving[op.arg1]) {
        m_pc++;
        return true;
      }
      return false;
    case 127:  // propulsion.set_pose
    {
      m_exchange_commands->pushMessage(CommMessageType::DbgPropulsionSetPose, m_vars + 4 * op.arg1,
                                       12);
    }
      m_pc++;
      return true;
    case 128:  // propulsion.point_to
    {
      float params[5];
      params[0] = *(float*)(m_vars + 4 * op.arg1);
      params[1] = *(float*)(m_vars + 4 * (op.arg1 + 1));
      params[2] = *(float*)(m_vars + 4 * (op.arg2));
      params[3] = *(float*)(m_vars + 4 * (op.arg2 + 1));
      params[4] = *(float*)(m_vars + 4 * (op.arg2 + 2));
      m_exchange_commands->pushMessage(CommMessageType::DbgPropulsionExecutePointTo,
                                       (unsigned char*)params, sizeof(params));
    }
      m_propulsion_state_dirty = true;
      m_pc++;
      return false;
    case 129:  // propulsion.move_to
    {
      float params[5];
      params[0] = *(float*)(m_vars + 4 * op.arg1);
      params[1] = *(float*)(m_vars + 4 * (op.arg1 + 1));
      params[2] = *(float*)(m_vars + 4 * (op.arg2));
      params[3] = *(float*)(m_vars + 4 * (op.arg2 + 1));
      params[4] = *(float*)(m_vars + 4 * (op.arg2 + 2));

      m_exchange_commands->pushMessage(CommMessageType::DbgPropulsionExecuteMoveTo,
                                       (unsigned char*)params, sizeof(params));
      m_propulsion_state_dirty = true;
      m_pc++;
      return false;
    }
    case 120:  // propulsion.trajectory
    {
      unsigned char buff[76];  // 12 for traj params and 8*8 for points
      *(float*)(buff) = get_var_float(op.arg3);
      *(float*)(buff + 4) = get_var_float(op.arg3 + 1);
      *(float*)(buff + 8) = get_var_float(op.arg3 + 2);

      memcpy(buff + 12, m_vars + 4 * op.arg1, op.arg2 * 8);

      m_exchange_commands->pushMessage(CommMessageType::DbgPropulsionExecuteTrajectory, buff,
                                       12 + op.arg2 * 8);

      m_propulsion_state_dirty = true;
      m_pc++;
      return false;
    }
    case 130:  // propulsion.rotate
    {
      float params[4];
      params[0] = *(float*)(m_vars + 4 * op.arg1);
      params[1] = *(float*)(m_vars + 4 * (op.arg2));
      params[2] = *(float*)(m_vars + 4 * (op.arg2 + 1));
      params[3] = *(float*)(m_vars + 4 * (op.arg2 + 2));

      m_exchange_commands->pushMessage(CommMessageType::DbgPropulsionExecuteRotation,
                                       (unsigned char*)params, sizeof(params));
      m_propulsion_state_dirty = true;
      m_pc++;
      return false;
    }
    case 131:  // propulsion.translate
    {
      float params[4];
      params[0] = *(float*)(m_vars + 4 * op.arg1);
      params[1] = *(float*)(m_vars + 4 * (op.arg2));
      params[2] = *(float*)(m_vars + 4 * (op.arg2 + 1));
      params[3] = *(float*)(m_vars + 4 * (op.arg2 + 2));
      m_exchange_commands->pushMessage(CommMessageType::PropulsionExecuteTranslation,
                                       (unsigned char*)params, sizeof(params));
      m_propulsion_state_dirty = true;
      m_pc++;
      return false;
    }
    case 132:  // propulsion.reposition
    {
      float params[2];
      params[0] = *(float*)(m_vars + 4 * op.arg1);
      params[1] = *(float*)(m_vars + 4 * (op.arg2));
      m_exchange_commands->pushMessage(CommMessageType::DbgPropulsionExecuteReposition,
                                       (unsigned char*)params, sizeof(params));
      m_propulsion_state_dirty = true;
      m_pc++;
      return false;
    }
    case 133:  // propulsion.enter_manual
    {
      m_exchange_commands->pushMessage(CommMessageType::PropulsionEnterManualControl, nullptr, 0);
      m_pc++;
      m_propulsion_state_dirty = true;
      return false;
    }
    case 134:  // propulsion.exit_manual
    {
      m_exchange_commands->pushMessage(CommMessageType::PropulsionExitManualControl, nullptr, 0);
      m_pc++;
      m_propulsion_state_dirty = true;
      return false;
    }
    case 135:  // propulsion.set_control_levels
    {
      m_exchange_commands->pushMessage(CommMessageType::PropulsionSetControlLevels, &(op.arg1), 2);
      m_pc++;
      return false;
    }
    case 136:  // propulsion.set_target_pose
    {
      RobotPose pose;
      pose.position.x = *(float*)(m_vars + 4 * op.arg1);
      pose.position.y = *(float*)(m_vars + 4 * (op.arg1 + 1));
      pose.yaw = *(float*)(m_vars + 4 * (op.arg1 + 2));
      pose.speed = *(float*)(m_vars + 4 * (op.arg2 + 0));
      pose.yaw_rate = *(float*)(m_vars + 4 * (op.arg2 + 1));
      pose.acceleration = 0;
      pose.angular_acceleration = 0;
      m_exchange_commands->pushMessage(CommMessageType::PropulsionSetTargetPose,
                                       (unsigned char*)&pose, sizeof(pose));
      m_pc++;
      return false;
    }
    case 137:  // propulsion.face_direction
    {
      float params[4];
      params[0] = *(float*)(m_vars + 4 * op.arg1);
      params[1] = *(float*)(m_vars + 4 * (op.arg2));
      params[2] = *(float*)(m_vars + 4 * (op.arg2 + 1));
      params[3] = *(float*)(m_vars + 4 * (op.arg2 + 2));

      m_exchange_commands->pushMessage(CommMessageType::PropulsionExecuteFaceDirection,
                                       (unsigned char*)params, sizeof(params));
      m_propulsion_state_dirty = true;
      m_pc++;
      return false;
    }
    case 138:  // propulsion.set_adversary_detection_enable
    {
      unsigned char buff;
      buff = op.arg1;

      m_exchange_commands->pushMessage(CommMessageType::PropulsionSetAdversaryDetectionEnable,
                                       &buff, 1);
    }
      m_pc++;
      return true;
    case 139:  // propulsion.measure_normal
    {
      // arg: vector(border normal angle, projection of border point on normal
      float buff[2] = {get_var_float(op.arg1), get_var_float(op.arg1 + 1)};
      m_exchange_commands->pushMessage(CommMessageType::PropulsionMeasureNormal,
                                       (unsigned char*)&buff, sizeof(buff));
      m_pc++;
      return false;
    }
    case 147:  // propulsion.measure_point
    {
      // arg: vector(border normal angle, projection of border point on normal
      float buff[4] = {get_var_float(op.arg1), get_var_float(op.arg1 + 1), get_var_float(op.arg2),
                       get_var_float(op.arg2 + 1)};
      m_exchange_commands->pushMessage(CommMessageType::PropulsionMeasurePoint,
                                       (unsigned char*)&buff, sizeof(buff));
      m_pc++;
      return false;
    }
    case 30:  // ret
      if (m_stack_level == 0) {
        m_state = SequenceState::Idle;
        return false;
      } else {
        m_stack_level--;
        m_pc = m_call_stack[m_stack_level] + 1;
      }
      return true;
    case 31:  // call
      m_call_stack[m_stack_level] = m_pc;
      m_stack_level++;
      m_pc = m_sequence_offsets[op.arg1];
      return true;
    case 33:  // yield
      /* Do nothing and wait for next task tick. use for polling loop */
      m_pc++;
      return false;
    case 34:  // send_event
    {
      unsigned char buff[41];
      buff[0] = op.arg1;
      memcpy(buff + 1, m_vars + 4 * op.arg2, 4 * op.arg3);

      //	Robot::instance().mainExchangeOut().pushMessage(
      //		CommMessageType::SequenceEvent,
      //		buff,
      //		1 + 4 * op.arg3);
    }
      m_pc++;
      return true;

    case 140:  // pump.set_pwm
    {
      unsigned char buff[3];
      buff[0] = 0;
      *(int16_t*)(buff + 1) = *(int*)(m_vars + 4 * op.arg1);
      m_exchange_commands->pushMessage(CommMessageType::FpgaCmdDCMotor, buff, 3);
    }
      m_pc++;
      return true;
    case 144:  // dc_motor.set_pwm
    {
      unsigned char buff[3];
      buff[0] = op.arg1;
      *(int16_t*)(buff + 1) = *(int*)(m_vars + 4 * op.arg2);
      m_exchange_commands->pushMessage(CommMessageType::FpgaCmdDCMotor, buff, 3);
    }
      m_pc++;
      return true;
    case 145:  // gpio.set
      hal::gpio_set(op.arg1, op.arg2);
      m_pc++;
      return true;
    case 141:  // arm.go_to_position
    {
      unsigned char buff[4];
      buff[0] = op.arg1;
      buff[1] = op.arg2;
      *(uint16_t*)(buff + 2) = op.arg3;  // speed in % of max speed
      m_exchange_commands->pushMessage(CommMessageType::DbgArmsGoToPosition, buff, 4);
      m_arm_state_dirty = true;
    }
      m_pc++;
      return true;
    case 142:  // set_servo
    {
      m_servo_state_dirty[op.arg1] = true;
      unsigned char buff[4];
      buff[0] = op.arg1;
      *(uint16_t*)(buff + 1) = *(int*)(m_vars + 4 * op.arg2);
      buff[3] = op.arg3;
      m_exchange_commands->pushMessage(CommMessageType::FpgaCmdServo, buff, 4);
    }
      m_pc++;
      return true;
    case 143:  // arms.shutdown
    {
      m_exchange_commands->pushMessage(CommMessageType::ArmsShutdown, nullptr, 0);
    }
      m_pc++;
      return true;
    case 150:  // check_sensor
      if (Robot::instance().sensorsState() & (1 << op.arg1)) {
        m_status_register |= 1;
      } else {
        m_status_register &= (0xffff - 1);
      }
      m_pc++;
      return true;
    case 151:  // check_propulsion_state
      // Set zero flag if equality
      if (!m_propulsion_state_dirty && (uint8_t)m_propulsion_state == op.arg1) {
        m_status_register &= (0xffff - FLAG_Z);
      } else {
        m_status_register |= FLAG_Z;
      }
      m_pc++;
      return true;
    case 152:  // propulsion.get_pose
    {
      auto pose = Robot::instance().odometry().pose();
      *(float*)(m_vars + 4 * op.arg1) = pose.position.x;
      *(float*)(m_vars + 4 * (op.arg1 + 1)) = pose.position.y;
      *(float*)(m_vars + 4 * (op.arg1 + 2)) = pose.yaw;
    }
      m_pc++;
      return true;
    case 160:  // cmp
    {
      // Set zero flag if equality
      int v1 = *(int*)(m_vars + 4 * op.arg1);
      int v2 = *(int*)(m_vars + 4 * op.arg2);
      if (v1 == v2) {
        m_status_register |= FLAG_Z;
      } else {
        m_status_register &= (0xffff - FLAG_Z);
      }
      if (v1 >= v2) {
        m_status_register &= (0xffff - FLAG_N);
      } else {
        m_status_register |= FLAG_N;
      }
    }

      m_pc++;
      return true;
    case 200:  // jmp
      m_pc = (uint16_t)op.arg1 | ((uint16_t)op.arg2 << 8);
      return true;
    case 201:  // jz,je
      if (m_status_register & FLAG_Z) {
        m_pc++;
      } else {
        m_pc = (uint16_t)op.arg1 | ((uint16_t)op.arg2 << 8);
      }
      return true;
    case 202:  // jnz,jne
      if (m_status_register & FLAG_Z) {
        m_pc = (uint16_t)op.arg1 | ((uint16_t)op.arg2 << 8);
      } else {
        m_pc++;
      }
      return true;
    case 203:  // jge
      // jump if last comparison was posive (has negative flag to 0
      if (m_status_register & FLAG_N) {
        m_pc++;
      } else {
        m_pc = (uint16_t)op.arg1 | ((uint16_t)op.arg2 << 8);
      }
      return true;
    default:
      m_pc++;
      return false;
  }
  return true;
}

void SequenceEngine::beginLoad() { m_load_offset = 0; }

void SequenceEngine::endLoad() {
  m_state = SequenceState::Idle;
  // decode header
  SequenceHeader* header = (SequenceHeader*)(m_buffer);

  // check crc
  // uint16_t crc = update_crc16(m_buffer + 4, header->size, 0);
  uint16_t crc = 0;
  if (crc == header->crc16) {
    unsigned char buff[1];
    buff[0] = 1;
    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::MainSequenceLoadStatus, buff,
                                                    1);
  } else {
    unsigned char buff[1];
    buff[0] = 0;
    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::MainSequenceLoadStatus, buff,
                                                    1);
  }
  m_num_vars = header->num_vars;
  m_num_seqs = header->num_seqs;
  m_vars = m_buffer + sizeof(SequenceHeader);
  m_sequence_offsets = (uint16_t*)(m_vars + 4 * header->num_vars);
  m_ops = (Op*)(m_vars + 4 * header->num_vars + 2 * header->num_seqs);
}

void SequenceEngine::loadData(unsigned char* data, uint16_t size) {
  std::memcpy(m_buffer + m_load_offset, data, size);
  m_load_offset += size;
}

void SequenceEngine::startSequence(int id) {
  m_state = SequenceState::Executing;
  m_pc = m_sequence_offsets[id];
}

void SequenceEngine::abortSequence() { m_state = SequenceState::Idle; }

void SequenceEngine::updateArmState(ArmState sta) {
  m_arm_state = sta;
  m_arm_state_dirty = false;
}

void SequenceEngine::updatePropulsionState(PropulsionState state) {
  m_propulsion_state = state;
  m_propulsion_state_dirty = false;
}
void SequenceEngine::updateServoState(int id, bool moving) {
  m_servo_moving[id] = moving;
  m_servo_state_dirty[id] = false;
}

}  // namespace goldobot
