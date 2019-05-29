#include "goldobot/sequence_engine.hpp"
#include "goldobot/message_types.hpp"
#include "goldobot/robot.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <cstring>
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
		m_pc++;
		return true;
	}

	case 125: // WAIT_ARM_FINISHED
		if(m_arm_moving == false)
			{
				m_pc++;
				return true;
			}
		return false;
	case 126: // WAIT_MOVEMENT_FINISHED
		if(m_moving == false)
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
		m_moving = true;
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
		m_moving = true;
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
		m_moving = true;
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
			m_moving = true;
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
			m_moving = true;
			m_pc++;
			return false;
		}
	case 133: // PROPULSION.ENTER_MANUAL
			{
				Robot::instance().mainExchangeIn().pushMessage(
						CommMessageType::PropulsionEnterManualControl,nullptr, 0);
				m_pc++;
				return false;
			}
	case 134: // PROPULSION.EXIT_MANUAL
			{
				Robot::instance().mainExchangeIn().pushMessage(
						CommMessageType::PropulsionExitManualControl,nullptr, 0);
				m_pc++;
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
	case 141: // ARM.GO_TO_POSITION
		{
			unsigned char buff[4];
			buff[0] = op.arg1;
			buff[1] = op.arg2;
			*(uint16_t*)(buff+2) = 1000;
			Robot::instance().mainExchangeIn().pushMessage(
					CommMessageType::DbgArmsGoToPosition,
					buff,
					4);
			m_arm_moving = true;
		}
		m_pc++;
		return true;
	case 142: // SET_SERVO
		{
			unsigned char buff[3];
			buff[0] = op.arg1;
			*(uint16_t*)(buff+1) = *(int*)(m_vars + 4 * op.arg2);
			Robot::instance().mainExchangeIn().pushMessage(
					CommMessageType::FpgaCmdServo,
					buff,
					4);
		}
		m_pc++;
		return true;
	default: // NOP (?)
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

/* FIXME : TODO : traiter d'autres causes d'interruption (pour l'instant on a : irq_id=1:="obstacle") */
void SequenceEngine::IRQ(int /*irq_id*/)
{
	if(m_moving && (m_ops[m_pc].opcode==126) && ((m_ops[m_pc-1].opcode==129)) ) // WAIT_MOVEMENT_FINISHED && PROPULSION.MOVE_TO
	{
		m_state = SequenceState::Interruption;
		Robot::instance().mainExchangeIn().pushMessage(CommMessageType::CmdEmergencyStop, nullptr, 0);
		m_moving = false;
	}
}

void SequenceEngine::IRQ_END(int /*irq_id*/)
{
	if(m_state == SequenceState::Interruption)
	{
		Hal::set_motors_enable(true);

		Op& op = m_ops[m_pc-1];

		float params[5];
		params[0] = *(float*)(m_vars + 4 * op.arg1);
		params[1] = *(float*)(m_vars + 4 * (op.arg1+1));
		params[2] = *(float*)(m_vars + 4 * (op.arg2));
		params[3] = *(float*)(m_vars + 4 * (op.arg2+1));
		params[4] = *(float*)(m_vars + 4 * (op.arg2+2));

		Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionClearError, nullptr, 0);
		Robot::instance().mainExchangeIn().pushMessage(
				CommMessageType::DbgPropulsionExecuteMoveTo,
				(unsigned char*) params, sizeof(params));
		m_moving = true;
		m_state = SequenceState::Executing;
	}
}


}
