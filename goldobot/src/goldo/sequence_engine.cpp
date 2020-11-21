#include "goldo/sequence_engine.hpp"
#include "goldo/message_types.hpp"
#include "goldo/robot.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <cstring>
uint16_t update_crc16(const unsigned char* data_p, size_t length, uint16_t crc = 0xFFFF);

#define FLAG_Z (1 << 0)
#define FLAG_N (1 << 1)

namespace goldobot
{

  enum Opcode
  {
    Propulsion_MoveTo = 129,
  };


  SequenceEngine::SequenceEngine()
  {
  }


  void SequenceEngine::doStep()
  {
#if 0 /* FIXME : TODO : obstacle avoid sequence */
    bool act_obstacle = (Hal::get_gpio(2)!=0);
#else
    bool act_obstacle = false;
#endif

    if (m_adversary_detection_enabled && (m_obstacle_count==0))
    {
      switch(m_irq_pc)
      {
      case 0x0000: // calculate escape_point & point to (anti_)escape_point
        break;
      case 0x0001: // wait point_to finished
        break;
      case 0x0002: // move to escape_point
        break;
      case 0x0003: // wait move_to finished
        if ((act_obstacle))
        {
          Robot::instance().mainExchangeIn().pushMessage(CommMessageType::CmdEmergencyStop, nullptr, 0);
          m_obstacle_count = 1000;
        }
        break;
      case 0x0004: // point to final_escape_point
        break;
      case 0x0005: // wait point_to finished
        break;
      case 0x0006: // move to final_escape_point
        break;
      case 0x0007: // wait move_to finished
        if ((act_obstacle))
        {
          Robot::instance().mainExchangeIn().pushMessage(CommMessageType::CmdEmergencyStop, nullptr, 0);
          m_obstacle_count = 1000;
        }
        break;
      case 0x0008: // point to target_point
        break;
      case 0x0009: // wait point_to finished
        break;
      case 0x000a: // move to target_point
        break;
      case 0x000b: // wait move_to finished
        if ((act_obstacle))
        {
          Robot::instance().mainExchangeIn().pushMessage(CommMessageType::CmdEmergencyStop, nullptr, 0);
          m_obstacle_count = 1000;
        }
        break;
      case 0x000c: // end irq
        break;

      case 0xffff: // not executing irq sequence
        if ((act_obstacle))
        {
          m_obstacle_count = 3000;
          IRQ (1);
        }
        break;
      }
    }

    if ((m_obstacle_count==1))
    {
      switch(m_irq_pc)
      {
      case 0x0000: // calculate escape_point & point to (anti_)escape_point
        break;
      case 0x0001: // wait point_to finished
        break;
      case 0x0002: // move to escape_point
        break;
      case 0x0003: // wait move_to finished
        if ((act_obstacle))
        {
          m_obstacle_count = 200;
        }
        else
        {
          Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionClearError, nullptr, 0);
          m_propulsion_state_dirty = true;
          Hal::set_motors_enable(true);
          m_irq_pc = 0x0002;
        }
        break;
      case 0x0004: // point to final_escape_point
        break;
      case 0x0005: // wait point_to finished
        break;
      case 0x0006: // move to final_escape_point
        break;
      case 0x0007: // wait move_to finished
        if ((act_obstacle))
        {
          m_obstacle_count = 200;
        }
        else
        {
          Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionClearError, nullptr, 0);
          m_propulsion_state_dirty = true;
          Hal::set_motors_enable(true);
          m_irq_pc = 0x0006;
        }
        break;
      case 0x0008: // point to target_point
        break;
      case 0x0009: // wait point_to finished
        break;
      case 0x000a: // move to target_point
        break;
      case 0x000b: // wait move_to finished
        if ((act_obstacle))
        {
          m_obstacle_count = 200;
        }
        else
        {
          Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionClearError, nullptr, 0);
          m_propulsion_state_dirty = true;
          Hal::set_motors_enable(true);
          m_irq_pc = 0x000a;
        }
        break;
      case 0x000c: // end irq
        break;

      case 0xffff: // not executing irq sequence
        if ((act_obstacle))
        {
          if (m_escape_impossible) // on peut rien faire, on bouge pas et.. on attend..
          {
            m_obstacle_count = 200;
          }
          else
          {
            Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionClearError, nullptr, 0);
            m_propulsion_state_dirty = true; /* FIXME : TODO : OK? */
            Hal::set_motors_enable(true);
            m_irq_pc = 0x0000;
          }
        }
        else
        {
          Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionClearError, nullptr, 0);
          m_propulsion_state_dirty = true; /* FIXME : TODO : OK? */
          Hal::set_motors_enable(true);
          m_irq_pc = 0x000a;
        }
        break;
      }
    }

    m_prev_obstacle = act_obstacle;

    if (m_obstacle_count>0)
    {
      m_obstacle_count--;
      return;
    }

    if ((m_irq_pc != 0xffff))
    {
      doStepIrqSeq();
    }

    switch(m_state)
    {
    case SequenceState::Executing:
      while(execOp(m_ops[m_pc])){};
      break;
    default:
      return;
    }
  }


  bool SequenceEngine::execOp(const Op& op)
  {
    switch(op.opcode)
    {
    case 0: // nop
      m_pc++;
      break;
    case 1: // mov1
      *(int32_t*)(m_vars + 4 * op.arg1) = *(int32_t*)(m_vars + 4 * op.arg2);
      m_pc++;
      break;
    case 2: // mov2
      *(int32_t*)(m_vars + 4 * op.arg1) = *(int32_t*)(m_vars + 4 * op.arg2);
      *(int32_t*)(m_vars + 4 * (op.arg1 + 1)) = *(int32_t*)(m_vars + 4 * (op.arg2 + 1));
      m_pc++;
      break;
    case 3: // mov3
      *(int32_t*)(m_vars + 4 * op.arg1) = *(int32_t*)(m_vars + 4 * op.arg2);
      *(int32_t*)(m_vars + 4 * (op.arg1 + 1)) = *(int32_t*)(m_vars + 4 * (op.arg2 + 1));
      *(int32_t*)(m_vars + 4 * (op.arg1 + 2)) = *(int32_t*)(m_vars + 4 * (op.arg2 + 2));
      m_pc++;
      break;
    case 4: // movi
      *(int32_t*)(m_vars + 4 * op.arg1) = op.arg2;
      m_pc++;
      break;
    case 8: // addi
      *(int32_t*)(m_vars + 4 * op.arg1) += op.arg2;
      m_pc++;
      break;
    case 9: // subi
      *(int32_t*)(m_vars + 4 * op.arg1) -= op.arg2;
      m_pc++;
      break;
    case 10: // add
      {
        auto a = *(int32_t*)(m_vars + 4 * op.arg2);
        auto b = *(int32_t*)(m_vars + 4 * op.arg3);
        *(int32_t*)(m_vars + 4 * op.arg1) = a + b;
      }
      m_pc++;
      break;
    case 11: // sub
      {
        auto a = *(int32_t*)(m_vars + 4 * op.arg2);
        auto b = *(int32_t*)(m_vars + 4 * op.arg3);
        *(int32_t*)(m_vars + 4 * op.arg1) = a - b;
      }
      m_pc++;
      break;
    case 32: // delay
      if(m_end_delay == 0)
      {
        m_end_delay = xTaskGetTickCount() + *(int*)(m_vars + 4 * op.arg1);
        return false;
      }
      if(xTaskGetTickCount() >= m_end_delay)
      {
        m_end_delay = 0;
        m_pc++;
        return true;
      }
      return false;
    case 64: // propulsion.motors_enable
      Hal::set_motors_enable(true);
      m_pc++;
      return true;
    case 65: // propulsion.enable
      {
        uint8_t b = true;
        Robot::instance().mainExchangeIn().pushMessage(CommMessageType::DbgSetPropulsionEnable, (unsigned char*)&b, 1);
        m_propulsion_state_dirty = true;
        m_pc++;
        return true;
      }
    case 66: // propulsion.motors_disable
      Hal::set_motors_enable(false);
      m_pc++;
      return true;
    case 67: // propulsion.disable
      {
        uint8_t b = false;
        Robot::instance().mainExchangeIn().pushMessage(CommMessageType::DbgSetPropulsionEnable, (unsigned char*)&b, 1);
        m_propulsion_state_dirty = true;
        m_pc++;
        return true;
      }
    case 125: // wait_arm_finished
      if(m_arm_state == ArmState::Idle && !m_arm_state_dirty)
      {
        m_pc++;
        return true;
      }
      return false;
    case 126: // wait_movement_finished
      if (m_irq_pc != 0xffff)
      {
        return false;
      }
      if(!m_propulsion_state_dirty && m_propulsion_state == PropulsionState::Stopped)
      {
        m_pc++;
        return true;
      }
      return false;
    case 146: // wait_servo_finished
      if(!m_servo_moving[op.arg1])
      {
        m_pc++;
        return true;
      }
      return false;
    case 127: // propulsion.set_pose
      if (m_irq_pc != 0xffff)
      {
        return false;
      }

      {
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::DbgPropulsionSetPose,
          m_vars + 4 * op.arg1, 12);
      }
      m_pc++;
      return true;
    case 128: // propulsion.point_to
      if (m_irq_pc != 0xffff)
      {
        return false;
      }

      {
        float params[5];
        params[0] = *(float*)(m_vars + 4 * op.arg1);
        params[1] = *(float*)(m_vars + 4 * (op.arg1+1));
        params[2] = *(float*)(m_vars + 4 * (op.arg2));
        params[3] = *(float*)(m_vars + 4 * (op.arg2+1));
        params[4] = *(float*)(m_vars + 4 * (op.arg2+2));
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::DbgPropulsionExecutePointTo,
          (unsigned char*) params, sizeof(params));

        m_propulsion_state_dirty = true;
        m_pc++;
        return false;
      }
      break;
    case 129: // propulsion.move_to
      if (m_irq_pc != 0xffff)
      {
        return false;
      }

      {
        float params[5];
        params[0] = *(float*)(m_vars + 4 * op.arg1);
        params[1] = *(float*)(m_vars + 4 * (op.arg1+1));
        params[2] = *(float*)(m_vars + 4 * (op.arg2));
        params[3] = *(float*)(m_vars + 4 * (op.arg2+1));
        params[4] = *(float*)(m_vars + 4 * (op.arg2+2));

        m_saved_target_x = params[0];
        m_saved_target_y = params[1];

        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::DbgPropulsionExecuteMoveTo,
          (unsigned char*) params, sizeof(params));
        m_propulsion_state_dirty = true;
        m_pc++;
        return false;
      }
      break;
    case 120: // propulsion.trajectory
      if (m_irq_pc != 0xffff)
      {
        return false;
      }

      {
        unsigned char buff[76];//12 for traj params and 8*8 for points
        *(float*)(buff) = get_var_float(op.arg3);
        *(float*)(buff+4) = get_var_float(op.arg3+1);
        *(float*)(buff+8) = get_var_float(op.arg3 + 2);

        memcpy(buff+12, m_vars + 4 * op.arg1, op.arg2 * 8);

        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::DbgPropulsionExecuteTrajectory,
          buff, 12 + op.arg2 * 8);

        m_propulsion_state_dirty = true;
        m_pc++;
        return false;
      }
      break;
    case 130: // propulsion.rotate
      if (m_irq_pc != 0xffff)
      {
        return false;
      }

      {
        float params[4];
        params[0] = *(float*)(m_vars + 4 * op.arg1);
        params[1] = *(float*)(m_vars + 4 * (op.arg2));
        params[2] = *(float*)(m_vars + 4 * (op.arg2+1));
        params[3] = *(float*)(m_vars + 4 * (op.arg2+2));

        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::DbgPropulsionExecuteRotation,
          (unsigned char*) params, sizeof(params));
        m_propulsion_state_dirty = true;
        m_pc++;
        return false;
      }
      break;
    case 131: // propulsion.translate
      if (m_irq_pc != 0xffff)
      {
        return false;
      }

      {
        float params[4];
        params[0] = *(float*)(m_vars + 4 * op.arg1);
        params[1] = *(float*)(m_vars + 4 * (op.arg2));
        params[2] = *(float*)(m_vars + 4 * (op.arg2+1));
        params[3] = *(float*)(m_vars + 4 * (op.arg2+2));
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::PropulsionExecuteTranslation,
          (unsigned char*) params, sizeof(params));
        m_propulsion_state_dirty = true;
        m_pc++;
        return false;
      }
      break;
    case 132: // propulsion.reposition
      if (m_irq_pc != 0xffff)
      {
        return false;
      }

      {
        float params[2];
        params[0] = *(float*)(m_vars + 4 * op.arg1);
        params[1] = *(float*)(m_vars + 4 * (op.arg2));
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::DbgPropulsionExecuteReposition,
          (unsigned char*) params, sizeof(params));
        m_propulsion_state_dirty = true;
        m_pc++;
        return false;
      }
      break;
    case 133: // propulsion.enter_manual
      {
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::PropulsionEnterManualControl,nullptr, 0);
        m_pc++;
        m_propulsion_state_dirty = true;
        return false;
      }
    case 134: // propulsion.exit_manual
      {
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::PropulsionExitManualControl,nullptr, 0);
        m_pc++;
        m_propulsion_state_dirty = true;
        return false;
      }
    case 135: // propulsion.set_control_levels
      {
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::PropulsionSetControlLevels,&(op.arg1), 2);
        m_pc++;
        return false;
      }
    case 136: // propulsion.set_target_pose
      if (m_irq_pc != 0xffff)
      {
        return false;
      }

      {
        RobotPose pose;
        pose.position.x = *(float*)(m_vars + 4 * op.arg1);
        pose.position.y = *(float*)(m_vars + 4 * (op.arg1 + 1));
        pose.yaw = *(float*)(m_vars + 4 * (op.arg1 + 2));
        pose.speed = *(float*)(m_vars + 4 * (op.arg2 + 0));
        pose.yaw_rate = *(float*)(m_vars + 4 * (op.arg2 + 1));
        pose.acceleration = 0;
        pose.angular_acceleration = 0;
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::PropulsionSetTargetPose,(unsigned char*)&pose, sizeof(pose));
        m_pc++;
        return false;
      }
    case 137: // propulsion.face_direction
      if (m_irq_pc != 0xffff)
      {
        return false;
      }

      {
        float params[4];
        params[0] = *(float*)(m_vars + 4 * op.arg1);
        params[1] = *(float*)(m_vars + 4 * (op.arg2));
        params[2] = *(float*)(m_vars + 4 * (op.arg2+1));
        params[3] = *(float*)(m_vars + 4 * (op.arg2+2));

        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::PropulsionExecuteFaceDirection,
          (unsigned char*) params, sizeof(params));
        m_propulsion_state_dirty = true;
        m_pc++;
        return false;
      }
    case 138: // propulsion.set_adversary_detection_enable
      {
        unsigned char buff;
        buff = op.arg1;

        m_adversary_detection_enabled = (bool)buff;

        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::PropulsionSetAdversaryDetectionEnable,
          &buff,
          1);
      }
      m_pc++;
      return true;
    case 139: // propulsion.measure_normal
      {
        //arg: vector(border normal angle, projection of border point on normal
        float buff[2] = {get_var_float(op.arg1), get_var_float(op.arg1+1)};
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::PropulsionMeasureNormal,(unsigned char*)&buff, sizeof(buff));
        m_pc++;
        return false;
      }
    case 147: // propulsion.measure_point
      {
        //arg: vector(border normal angle, projection of border point on normal
        float buff[4] = {get_var_float(op.arg1),
                         get_var_float(op.arg1+1),
                         get_var_float(op.arg2),
                         get_var_float(op.arg2+1)
        };
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::PropulsionMeasurePoint,(unsigned char*)&buff, sizeof(buff));
        m_pc++;
        return false;
      }
    case 30: // ret
      if(m_stack_level == 0)
      {
        m_state = SequenceState::Idle;
        return false;
      } else
      {
        m_stack_level--;
        m_pc = m_call_stack[m_stack_level]+1;
      }
      return true;
    case 31: // call
      m_call_stack[m_stack_level] = m_pc;
      m_stack_level++;
      m_pc = m_sequence_offsets[op.arg1];
      return true;
    case 33: // yield
      /* Do nothing and wait for next task tick. use for polling loop */
      m_pc++;
      return false;
    case 34: // send_event
      {
        unsigned char buff[41];
        buff[0] = op.arg1;
        memcpy(buff+1, m_vars + 4 * op.arg2, 4 * op.arg3);

        Robot::instance().mainExchangeOut().pushMessage(
          CommMessageType::SequenceEvent,
          buff,
          1 + 4 * op.arg3);
      }
      m_pc++;
      return true;
    case 140: // pump.set_pwm
      {
        unsigned char buff[3];
        buff[0] = 0;
        *(int16_t*)(buff+1) = *(int*)(m_vars + 4 * op.arg1);
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::FpgaCmdDCMotor,
          buff,
          3);
      }
      m_pc++;
      return true;
    case 144: // dc_motor.set_pwm
      {
        unsigned char buff[3];
        buff[0] = op.arg1;
        *(int16_t*)(buff+1) = *(int*)(m_vars + 4 * op.arg2);
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::FpgaCmdDCMotor,
          buff,
          3);
      }
      m_pc++;
      return true;
    case 145: // gpio.set
      Hal::set_gpio(op.arg1,op.arg2);
      m_pc++;
      return true;
    case 141: // arm.go_to_position
      {
        unsigned char buff[4];
        buff[0] = op.arg1;
        buff[1] = op.arg2;
        *(uint16_t*)(buff+2) = op.arg3;// speed in % of max speed
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::DbgArmsGoToPosition,
          buff,
          4);
        m_arm_state_dirty = true;
      }
      m_pc++;
      return true;
    case 142: // set_servo
      {
        m_servo_state_dirty[op.arg1] = true;
        unsigned char buff[4];
        buff[0] = op.arg1;
        *(uint16_t*)(buff+1) = *(int*)(m_vars + 4 * op.arg2);
        buff[3] = op.arg3;
        Robot::instance().mainExchangeIn().pushMessage(
          CommMessageType::FpgaCmdServo,
          buff,
          4);
      }
      m_pc++;
      return true;
    case 143: // arms.shutdown
      Robot::instance().mainExchangeIn().pushMessage(
        CommMessageType::ArmsShutdown,
        nullptr,
        0);
      m_pc++;
      return true;
    case 150: // check_sensor
      if(Robot::instance().sensorsState() & (1 << op.arg1))
      {
        m_status_register |= 1;
      }
      else
      {
        m_status_register &= (0xffff-1);
      }
      m_pc++;
      return true;
    case 151: // check_propulsion_state
      // Set zero flag if equality
      if(!m_propulsion_state_dirty && (uint8_t)m_propulsion_state ==  op.arg1)
      {
        m_status_register &= (0xffff-FLAG_Z);
      }
      else
      {
        m_status_register |= FLAG_Z;
      }
      m_pc++;
      return true;
    case 152: // propulsion.get_pose
      {
        auto pose = Robot::instance().odometry().pose();
        *(float*)(m_vars + 4 * op.arg1) = pose.position.x;
        *(float*)(m_vars + 4 * (op.arg1 + 1)) = pose.position.y;
        *(float*)(m_vars + 4 * (op.arg1 + 2)) = pose.yaw;
      }
      m_pc++;
      return true;
    case 160: // cmp
      {
        // Set zero flag if equality
        int v1 = *(int*)(m_vars + 4 * op.arg1);
        int v2 = *(int*)(m_vars + 4 * op.arg2);
        if(v1 == v2)
        {
          m_status_register |= FLAG_Z;
        } 
        else
        {
          m_status_register &= (0xffff-FLAG_Z);
        }
        if(v1 >= v2)
        {
          m_status_register &= (0xffff-FLAG_N);
        } 
        else
        {
          m_status_register |= FLAG_N;
        }
      }

      m_pc++;
      return true;
    case 200: // jmp
      m_pc = (uint16_t)op.arg1 | ((uint16_t)op.arg2 << 8);
      return true;
    case 201: // jz,je
      if(m_status_register & FLAG_Z)
      {
        m_pc++;
      } 
      else
      {
        m_pc = (uint16_t)op.arg1 | ((uint16_t)op.arg2 << 8);
      }
      return true;
    case 202: // jnz,jne
      if(m_status_register & FLAG_Z)
      {
        m_pc = (uint16_t)op.arg1 | ((uint16_t)op.arg2 << 8);
      } 
      else
      {
        m_pc++;
      }
      return true;
    case 203: // jge
      // jump if last comparison was posive (has negative flag to 0
      if(m_status_register & FLAG_N)
      {
        m_pc++;
      } 
      else
      {
        m_pc = (uint16_t)op.arg1 | ((uint16_t)op.arg2 << 8);
      }
      return true;
    default: // nop (?)
      m_pc++;
      return false;
    }
    return true;
  }

  void SequenceEngine::beginLoad()
  {
    m_load_offset = 0;
  }

  void SequenceEngine::endLoad()
  {
    m_state = SequenceState::Idle;
    //decode header
    SequenceHeader* header = (SequenceHeader*)(m_buffer);

    //check crc
    uint16_t crc = update_crc16(m_buffer + 4, header->size, 0);

    if(crc == header->crc16)
    {
      unsigned char buff[1];
      buff[0] = 1;
      Robot::instance().mainExchangeOut().pushMessage(
        CommMessageType::MainSequenceLoadStatus,
        buff,
        1);
    } 
    else
    {
      unsigned char buff[1];
      buff[0] = 0;
      Robot::instance().mainExchangeOut().pushMessage(
        CommMessageType::MainSequenceLoadStatus,
        buff,
        1);
    }
    m_num_vars = header->num_vars;
    m_num_seqs = header->num_seqs;
    m_vars = m_buffer + sizeof(SequenceHeader);
    m_sequence_offsets = (uint16_t*)(m_vars + 4*header->num_vars);
    m_ops = (Op*)(m_vars + 4*header->num_vars + 2 * header->num_seqs);
  }


  void SequenceEngine::loadData(unsigned char* data, uint16_t size)
  {
    std::memcpy(m_buffer+m_load_offset, data, size);
    m_load_offset += size;
  }


  void SequenceEngine::startSequence(int id)
  {
    m_state = SequenceState::Executing;
    m_pc = m_sequence_offsets[id];
  }


  void SequenceEngine::abortSequence()
  {
    m_state = SequenceState::Idle;
  }


  void SequenceEngine::updateArmState(ArmState sta)
  {
    m_arm_state = sta;
    m_arm_state_dirty = false;
  }


  void SequenceEngine::updatePropulsionState(PropulsionState state)
  {
    m_propulsion_state = state;
    m_propulsion_state_dirty = false;
  }


  void SequenceEngine::updateServoState(int id, bool moving)
  {
    m_servo_moving[id] = moving;
    m_servo_state_dirty[id] = false;
  }


  void SequenceEngine::IRQ(int /*irq_id*/)
  {
    if((m_propulsion_state == PropulsionState::FollowTrajectory))
    {
      //m_state = SequenceState::Interruption;
      Robot::instance().mainExchangeIn().pushMessage(CommMessageType::CmdEmergencyStop, nullptr, 0);
    }
  }


  void SequenceEngine::beginIrqSeq(int obstacle_state)
  {
    if ((obstacle_state==1)) /* obstacle encore present */
    {
      Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionClearError, nullptr, 0);

      m_propulsion_state_dirty = true; /* FIXME : TODO : OK? */
      //m_state = SequenceState::Executing;

      m_irq_pc = 0x0000;

      Hal::set_motors_enable(true);
    }
    else /* l'adversaire s'est barre */
    {
      Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionClearError, nullptr, 0);

      m_propulsion_state_dirty = true; /* FIXME : TODO : OK? */
      //m_state = SequenceState::Executing;

      m_irq_pc = 0x000a;

      Hal::set_motors_enable(true);
    }
  }


  void SequenceEngine::doStepIrqSeq()
  {
    float params[5];

    switch(m_irq_pc)
    {
    case 0x0000: // calculate escape_point & point to (anti_)escape_point
      m_adversary_detection_enabled = false;
      calculateEscapePoint();
      //params[0] = m_anti_escape_x_m;
      //params[1] = m_anti_escape_y_m;
      params[0] = m_escape_x_m;
      params[1] = m_escape_y_m;
      params[2] = 3.1415;
      params[3] = 3.1415;
      params[4] = 3.1415;
      Robot::instance().mainExchangeIn().pushMessage(
        CommMessageType::DbgPropulsionExecutePointTo,
        (unsigned char*) params, sizeof(params));
      m_propulsion_state_dirty = true;
      m_irq_pc = 0x0001;
      break;
    case 0x0001: // wait point_to finished
      if(!m_propulsion_state_dirty && m_propulsion_state == PropulsionState::Stopped)
      {
        m_irq_pc = 0x0002;
      }
      break;
    case 0x0002: // move to escape_point
      m_adversary_detection_enabled = true;
      params[0] = m_escape_x_m;
      params[1] = m_escape_y_m;
      params[2] = 0.3;
      params[3] = 0.3;
      params[4] = 0.3;
      Robot::instance().mainExchangeIn().pushMessage(
        CommMessageType::DbgPropulsionExecuteMoveTo,
        (unsigned char*) params, sizeof(params));
      m_propulsion_state_dirty = true;
      m_irq_pc = 0x0003;
      break;
    case 0x0003: // wait move_to finished
      if(!m_propulsion_state_dirty && m_propulsion_state == PropulsionState::Stopped)
      {
        m_irq_pc = 0x0004;
      }
      break;
    case 0x0004: // point to final_escape_point
      params[0] = m_final_escape_x_m;
      params[1] = m_final_escape_y_m;
      params[2] = 3.1415;
      params[3] = 3.1415;
      params[4] = 3.1415;
      Robot::instance().mainExchangeIn().pushMessage(
        CommMessageType::DbgPropulsionExecutePointTo,
        (unsigned char*) params, sizeof(params));
      m_propulsion_state_dirty = true;
      m_irq_pc = 0x0005;
      break;
    case 0x0005: // wait point_to finished
      if(!m_propulsion_state_dirty && m_propulsion_state == PropulsionState::Stopped)
      {
        m_irq_pc = 0x0006;
      }
      break;
    case 0x0006: // move to final_escape_point
      params[0] = m_final_escape_x_m;
      params[1] = m_final_escape_y_m;
      params[2] = 0.3;
      params[3] = 0.3;
      params[4] = 0.3;
      Robot::instance().mainExchangeIn().pushMessage(
        CommMessageType::DbgPropulsionExecuteMoveTo,
        (unsigned char*) params, sizeof(params));
      m_propulsion_state_dirty = true;
      m_irq_pc = 0x0007;
      break;
    case 0x0007: // wait move_to finished
      if(!m_propulsion_state_dirty && m_propulsion_state == PropulsionState::Stopped)
      {
        m_irq_pc = 0x0008;
      }
      break;
    case 0x0008: // point to target_point
      params[0] = m_saved_target_x;
      params[1] = m_saved_target_y;
      params[2] = 3.1415;
      params[3] = 3.1415;
      params[4] = 3.1415;
      Robot::instance().mainExchangeIn().pushMessage(
        CommMessageType::DbgPropulsionExecutePointTo,
        (unsigned char*) params, sizeof(params));
      m_propulsion_state_dirty = true;
      m_irq_pc = 0x0009;
      break;
    case 0x0009: // wait point_to finished
      if(!m_propulsion_state_dirty && m_propulsion_state == PropulsionState::Stopped)
      {
        m_irq_pc = 0x000a;
      }
      break;
    case 0x000a: // move to target_point
      params[0] = m_saved_target_x;
      params[1] = m_saved_target_y;
      params[2] = 0.3;
      params[3] = 0.3;
      params[4] = 0.3;
      Robot::instance().mainExchangeIn().pushMessage(
        CommMessageType::DbgPropulsionExecuteMoveTo,
        (unsigned char*) params, sizeof(params));
      m_propulsion_state_dirty = true;
      m_irq_pc = 0x000b;
      break;
    case 0x000b: // wait move_to finished
      if(!m_propulsion_state_dirty && m_propulsion_state == PropulsionState::Stopped)
      {
        m_irq_pc = 0x000c;
      }
      break;
    case 0x000c: // end irq
      m_escape_impossible = false;
      m_adversary_detection_enabled = true;
      m_irq_pc = 0xffff;
      break;
    default:
      m_irq_pc = 0xffff;
    }
  }


  void SequenceEngine::calculateEscapePoint()
  {
    RobotPose my_pose = Robot::instance().odometry().pose();
    double my_x_m = my_pose.position.x;
    double my_y_m = my_pose.position.y;
    double my_theta_rad = my_pose.yaw;
    double my_cos_theta = cos(my_theta_rad);
    double my_sin_theta = sin(my_theta_rad);

    double new_theta_right_rad =  my_theta_rad - (M_PI/2.0);
    double new_theta_left_rad  =  my_theta_rad + (M_PI/2.0);

    double cos_new_theta_right =  my_sin_theta;
    double sin_new_theta_right = -my_cos_theta;

    double cos_new_theta_left  = -my_sin_theta;
    double sin_new_theta_left  =  my_cos_theta;

    double remaining_dx = m_saved_target_x - my_x_m;
    double remaining_dy = m_saved_target_y - my_y_m;
    double remaining_dist = sqrt(remaining_dx*remaining_dx + remaining_dy*remaining_dy);
    remaining_dist = remaining_dist<0.4?remaining_dist:0.4;

    double x_right_m = my_x_m + 0.4*cos_new_theta_right;
    double y_right_m = my_y_m + 0.4*sin_new_theta_right;

    double final_x_right_m = x_right_m + remaining_dist*my_cos_theta;
    double final_y_right_m = y_right_m + remaining_dist*my_sin_theta;

    double x_left_m  = my_x_m + 0.4*cos_new_theta_left;
    double y_left_m  = my_y_m + 0.4*sin_new_theta_left;

    double final_x_left_m = x_left_m + remaining_dist*my_cos_theta;
    double final_y_left_m = y_left_m + remaining_dist*my_sin_theta;

    double x_center_m  = 0.8;
    double y_center_m  = 0.0;

    bool right_invalid = (!insideMovingZone(x_right_m, y_right_m)) || (!insideMovingZone(final_x_right_m,final_y_right_m));
    bool left_invalid = (!insideMovingZone(x_left_m, y_left_m)) || (!insideMovingZone(final_x_left_m,final_y_left_m));

#if 0
    double D2_right  = (x_right_m-x_center_m)*(x_right_m-x_center_m) + 
                       (y_right_m-y_center_m)*(y_right_m-y_center_m);
    double D2_left   = (x_left_m-x_center_m)*(x_left_m-x_center_m) + 
                       (y_left_m-y_center_m)*(y_left_m-y_center_m);
#else
    double D2_right  = distToBorders(x_right_m, y_right_m);
    double D2_left   = distToBorders(x_left_m, y_left_m);
#endif

    if ((D2_left<D2_right) && (!right_invalid))
    { /* le cote droit est + eloigne de la bordure donc moins dangereux */
      m_escape_impossible = false;
      m_escape_x_m = x_right_m;
      m_escape_y_m = y_right_m;
      m_anti_escape_x_m = x_left_m;
      m_anti_escape_y_m = y_left_m;
      m_final_escape_x_m = final_x_right_m;
      m_final_escape_y_m = final_y_right_m;
    }
    else if (!left_invalid)
    { /* le cote gauche est + eloigne de la bordure donc moins dangereux */
      m_escape_impossible = false;
      m_escape_x_m = x_left_m;
      m_escape_y_m = y_left_m;
      m_anti_escape_x_m = x_right_m;
      m_anti_escape_y_m = y_right_m;
      m_final_escape_x_m = final_x_left_m;
      m_final_escape_y_m = final_y_left_m;
    }
    else // (right_invalid && left_invalid)
    { /* les deux cotes sont bloques, on peut rien faire.. */
      m_escape_impossible = true;
      m_escape_x_m = my_x_m;
      m_escape_y_m = my_y_m;
      m_anti_escape_x_m = my_x_m;
      m_anti_escape_y_m = my_y_m;
      m_final_escape_x_m = my_x_m;
      m_final_escape_y_m = my_y_m;
    }
  }


  bool SequenceEngine::insideMovingZone(double x_m, double y_m)
  {
    if ((x_m<0.12) || (x_m>1.88))
    {
      return false;
    }

    if ((y_m<-1.38) || (y_m>1.38))
    {
      return false;
    }

    if ((x_m>1.423) && (y_m>-0.33) && (y_m<0.33))
    {
      return false;
    }

    if ((x_m>1.223) && (y_m>-0.14) && (y_m<0.14))
    {
      return false;
    }

    return true;
  }


  double SequenceEngine::distToBorders(double x_m, double y_m)
  {
    double d1_x_m, d2_x_m, d1_y_m, d2_y_m;
    double d_x_m, d_y_m;

    d1_x_m = x_m - 0.0;
    d2_x_m = 2.0 - x_m;
    d1_y_m = y_m - (-1.5);
    d2_y_m = 1.5 - y_m;

    if ((d1_x_m<d2_x_m) && (d1_x_m>0.0))
    {
      d_x_m = d1_x_m;
    }
    else if ((d2_x_m>0.0))
    {
      d_x_m = d2_x_m;
    }
    else // should never happen..
    {
      d_x_m = 10.0;
    }

    if ((d1_y_m<d2_y_m) && (d1_y_m>0.0))
    {
      d_y_m = d1_y_m;
    }
    else if ((d2_y_m>0.0))
    {
      d_y_m = d2_y_m;
    }
    else // should never happen..
    {
      d_y_m = 10.0;
    }

    return ((d_x_m<d_y_m)?d_x_m:d_y_m);
  }

} /* namespace goldobot */
