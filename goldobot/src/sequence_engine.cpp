#include "goldobot/sequence_engine.hpp"
#include "goldobot/message_types.hpp"
#include "goldobot/robot.hpp"

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
	bool act_obstacle = (Hal::get_gpio(2)!=0);

	if ((act_obstacle) && (!m_prev_obstacle) && (m_obstacle_count==0))
	{
		m_obstacle_count = 4000;
		IRQ (1);
	}

	if ((m_obstacle_count==2000))
	{
		if ((act_obstacle))
		{
			IRQ_END (1);
		}
		else
		{
			IRQ_END (0);
		}

	}

	if (m_obstacle_count>0)
	{
		m_obstacle_count--;
	}

	m_prev_obstacle = act_obstacle;

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
	case 0: // NOP
		m_pc++;
		break;
	case 1: // MOV1
		*(int32_t*)(m_vars + 4 * op.arg1) = *(int32_t*)(m_vars + 4 * op.arg2);
		m_pc++;
		break;
	case 2: // MOV2
		*(int32_t*)(m_vars + 4 * op.arg1) = *(int32_t*)(m_vars + 4 * op.arg2);
		*(int32_t*)(m_vars + 4 * (op.arg1 + 1)) = *(int32_t*)(m_vars + 4 * (op.arg2 + 1));
		m_pc++;
		break;
	case 3: // MOV3
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
	case 32: // DELAY
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
	case 64: // PROPULSION.MOTORS_ENABLE
		Hal::set_motors_enable(true);
		m_pc++;
		return true;
	case 65: // PROPULSION.ENABLE
	{
		uint8_t b = true;
		Robot::instance().mainExchangeIn().pushMessage(CommMessageType::DbgSetPropulsionEnable, (unsigned char*)&b, 1);
		m_propulsion_state_dirty = true;
		m_pc++;
		return true;
	}
	case 66: //PROPULSION.MOTORS_DISABLE
		Hal::set_motors_enable(false);
		m_pc++;
		return true;
	case 67: // PROPULSION.DISABLE
		{
			uint8_t b = false;
			Robot::instance().mainExchangeIn().pushMessage(CommMessageType::DbgSetPropulsionEnable, (unsigned char*)&b, 1);
			m_propulsion_state_dirty = true;
			m_pc++;
			return true;
		}

	case 125: // WAIT_ARM_FINISHED
		if(m_arm_state == ArmState::Idle && !m_arm_state_dirty)
			{
				m_pc++;
				return true;
			}
		return false;
	case 126: // WAIT_MOVEMENT_FINISHED
		if(!m_propulsion_state_dirty && m_propulsion_state == PropulsionState::Stopped)
			{
				m_pc++;
				return true;
			}
		return false;
	case 146: // wait_for_servo_finished
		if(!m_servo_moving[op.arg1])
			{
				m_pc++;
				return true;
			}
		return false;
	case 127: // PROPULSION.SETPOSE
		{
			Robot::instance().mainExchangeIn().pushMessage(
					CommMessageType::DbgPropulsionSetPose,
					m_vars + 4 * op.arg1, 12);
		}
			m_pc++;
			return true;
	case 128: // PROPULSION.POINTTO
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
	}
		m_propulsion_state_dirty = true;
		m_pc++;
		return false;
	case 129: // PROPULSION.MOVE_TO
	{
		float params[5];
		params[0] = *(float*)(m_vars + 4 * op.arg1);
		params[1] = *(float*)(m_vars + 4 * (op.arg1+1));
		params[2] = *(float*)(m_vars + 4 * (op.arg2));
		params[3] = *(float*)(m_vars + 4 * (op.arg2+1));
		params[4] = *(float*)(m_vars + 4 * (op.arg2+2));

		Robot::instance().mainExchangeIn().pushMessage(
				CommMessageType::DbgPropulsionExecuteMoveTo,
				(unsigned char*) params, sizeof(params));
		m_propulsion_state_dirty = true;
		m_pc++;
		return false;
	}
	case 120://propulsion.trajectory
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
	case 130: // PROPULSION.ROTATE
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
	case 131: // PROPULSION.TRANSLATE
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
	case 132: // PROPULSION.REPOSITION
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
	case 133: // PROPULSION.ENTER_MANUAL
			{
				Robot::instance().mainExchangeIn().pushMessage(
						CommMessageType::PropulsionEnterManualControl,nullptr, 0);
				m_pc++;
				m_propulsion_state_dirty = true;
				return false;
			}
	case 134: // PROPULSION.EXIT_MANUAL
			{
				Robot::instance().mainExchangeIn().pushMessage(
						CommMessageType::PropulsionExitManualControl,nullptr, 0);
				m_pc++;
				m_propulsion_state_dirty = true;
				return false;
			}
	case 135: // PROPULSION.SET_CONTROL_LEVELS
			{
				Robot::instance().mainExchangeIn().pushMessage(
						CommMessageType::PropulsionSetControlLevels,&(op.arg1), 2);
				m_pc++;
				return false;
			}
	case 136: // PROPULSION.SET_TARGET_POSE
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
	case 137://PROPULSION.FACE_DIRECTION
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
	case 138: // set adversary detection enable
		{
			unsigned char buff;
			buff = op.arg1;

			Robot::instance().mainExchangeIn().pushMessage(
					CommMessageType::PropulsionSetAdversaryDetectionEnable,
					&buff,
					1);
		}
		m_pc++;
		return true;
	case 139://propulsion.measure_normal
		{
			//arg: vector(border normal angle, projection of border point on normal
			float buff[2] = {get_var_float(op.arg1), get_var_float(op.arg1+1)};
			Robot::instance().mainExchangeIn().pushMessage(
					CommMessageType::PropulsionMeasureNormal,(unsigned char*)&buff, sizeof(buff));
			m_pc++;
			return false;
		}
	case 147://propulsion.measure_normal
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
	case 30: // RET
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
	case 31: // CALL
		m_call_stack[m_stack_level] = m_pc;
		m_stack_level++;
		m_pc = m_sequence_offsets[op.arg1];
		return true;
	case 33: // YIELD. Do nothing and wait for next task tick. use for polling loop
		m_pc++;
		return false;
	case 34: // SEQUENCE EVENT
		{
			unsigned char buff[2];
			buff[0] = op.arg1;
			buff[1] = op.arg2;
			Robot::instance().mainExchangeOut().pushMessage(
					CommMessageType::SequenceEvent,
					buff,
					2);
		}
		m_pc++;
		return true;

	case 140: // PUMP.SET_PWM
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
	case 144://dc motor
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
	case 145://gpio out
		Hal::set_gpio(op.arg1,op.arg2);
		m_pc++;
		return true;
	case 141: // ARM.GO_TO_POSITION
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
	case 142: // SET_SERVO
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
	case 143: // ARMS.SHUTDOWN
		{
			Robot::instance().mainExchangeIn().pushMessage(
					CommMessageType::ArmsShutdown,
					nullptr,
					0);

		}
		m_pc++;
		return true;
	case 150: // CHECK_SENSOR
		if(Robot::instance().sensorsState() & (1 << op.arg1))
		{
			m_status_register |= 1;
		} else
		{
			m_status_register &= (0xffff-1);
		}
		m_pc++;
		return true;
	case 151: // check propulsion state
		// Set zero flag if equality
		if(!m_propulsion_state_dirty && (uint8_t)m_propulsion_state ==  op.arg1)
		{
			m_status_register &= (0xffff-FLAG_Z);
		} else
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
		} else
		{
			m_status_register &= (0xffff-FLAG_Z);
		}
		if(v1 >= v2)
		{
			m_status_register &= (0xffff-FLAG_N);
		} else
		{
			m_status_register |= FLAG_N;
		}
	}

		m_pc++;
		return true;
	case 200://JMP
		m_pc = (uint16_t)op.arg1 | ((uint16_t)op.arg2 << 8);
		return true;
	case 201://JZ
		if(m_status_register & FLAG_Z)
		{
			m_pc++;
		} else
		{
			m_pc = (uint16_t)op.arg1 | ((uint16_t)op.arg2 << 8);
		}
		return true;
	case 202://JNZ
		if(m_status_register & FLAG_Z)
		{
			m_pc = (uint16_t)op.arg1 | ((uint16_t)op.arg2 << 8);
		} else
		{
			m_pc++;
		}
		return true;
	case 203://JGE
		// jump if last comparison was posive (has negative flag to 0
		if(m_status_register & FLAG_N)
		{
			m_pc++;
		} else
		{
			m_pc = (uint16_t)op.arg1 | ((uint16_t)op.arg2 << 8);

		}
		return true;

	default: //NOP (?)
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
	} else
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

/* FIXME : TODO : traiter d'autres causes d'interruption (pour l'instant on a : irq_id=1:="obstacle") */
void SequenceEngine::IRQ(int /*irq_id*/)
{
	if((m_propulsion_state == PropulsionState::FollowTrajectory) && (m_ops[m_pc].opcode==126) && ((m_ops[m_pc-1].opcode==129)) ) // WAIT_MOVEMENT_FINISHED && PROPULSION.MOVE_TO
	{
		m_state = SequenceState::Interruption;
		Robot::instance().mainExchangeIn().pushMessage(CommMessageType::CmdEmergencyStop, nullptr, 0);
	}
}

void SequenceEngine::IRQ_END(int irq_id)
{
	if ((irq_id==1)) /* obstacle encore present */
	{
		uint16_t irq_seq_pc = m_sequence_offsets[5];

		if (((irq_seq_pc+4)!=(m_pc)) || (m_saved_irq_pc==0xffff)) m_saved_irq_pc = m_pc-1;

		Op& old_op = m_ops[m_saved_irq_pc];

		//Op& op0 = m_ops[irq_seq_pc]; // DELAY : raf


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

		double x_right_m = my_x_m + 0.3*cos_new_theta_right;
		double y_right_m = my_y_m + 0.3*sin_new_theta_right;

		double x_left_m  = my_x_m + 0.3*cos_new_theta_left;
		double y_left_m  = my_y_m + 0.3*sin_new_theta_left;

		double x_center_m  = 0.8;
		double y_center_m  = 0.0;

		double new_x_m = x_center_m;
		double new_y_m = y_center_m;

		double anti_new_x_m = x_center_m;
		double anti_new_y_m = y_center_m;

		double D2_right  = (x_right_m-x_center_m)*(x_right_m-x_center_m) + 
                           (y_right_m-y_center_m)*(y_right_m-y_center_m);

		double D2_left   = (x_left_m-x_center_m)*(x_left_m-x_center_m) + 
                           (y_left_m-y_center_m)*(y_left_m-y_center_m);

        if (D2_right<D2_left)
        {
            new_x_m = x_right_m;
            new_y_m = y_right_m;
            anti_new_x_m = x_left_m;
            anti_new_y_m = y_left_m;
        }
        else
        {
            new_x_m = x_left_m;
            new_y_m = y_left_m;
            anti_new_x_m = x_right_m;
            anti_new_y_m = y_right_m;
        }

		Op& op1 = m_ops[irq_seq_pc+1]; // PROPULSION.POINT_TO parking
		*(float*)(m_vars + 4 * op1.arg1)     = anti_new_x_m;
		*(float*)(m_vars + 4 * (op1.arg1+1)) = anti_new_y_m;

		//Op& op2 = m_ops[irq_seq_pc+2]; // WAIT_MOVEMENT_FINISHED point_to parking : raf

		Op& op3 = m_ops[irq_seq_pc+3]; // PROPULSION.MOVE_TO parking : raf
		*(float*)(m_vars + 4 * op3.arg1)     = new_x_m;
		*(float*)(m_vars + 4 * (op3.arg1+1)) = new_y_m;

		//Op& op4 = m_ops[irq_seq_pc+4]; // WAIT_MOVEMENT_FINISHED move_to parking : raf

		Op& op5 = m_ops[irq_seq_pc+5]; // PROPULSION.POINT_TO cible
		op5.arg1 = old_op.arg1;
		op5.arg2 = old_op.arg2;

		//Op& op6 = m_ops[irq_seq_pc+6]; // WAIT_MOVEMENT_FINISHED point_to cible : raf

		Op& op7 = m_ops[irq_seq_pc+7]; // JMP
		uint16_t dummy = m_saved_irq_pc & 0x00ff;
		op7.arg1 = (uint8_t) dummy;
		dummy = (m_saved_irq_pc >> 8) & 0x00ff;
		op7.arg2 = (uint8_t) dummy;

		m_pc = irq_seq_pc;

		Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionClearError, nullptr, 0);

		m_propulsion_state_dirty = true; /* FIXME : TODO : OK? */
		m_state = SequenceState::Executing;

		Hal::set_motors_enable(true);
	}
	else /* l'adversaire s'est barre */
	{
		uint16_t irq_seq_pc = m_sequence_offsets[5];

		m_pc = m_pc-1;

		Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionClearError, nullptr, 0);

		m_propulsion_state_dirty = true; /* FIXME : TODO : OK? */
		m_state = SequenceState::Executing;

		Hal::set_motors_enable(true);
	}
}

}
