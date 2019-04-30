#include "goldobot/sequence_engine.hpp"
#include "goldobot/message_types.hpp"
#include "goldobot/robot.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <cstring>
namespace goldobot
{

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
	case 32:
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
	case 126:
		if(m_moving == false)
			{
				m_pc++;
				return true;
			}
		return false;
	case 127://propulsion.setPose
		{
			Robot::instance().mainExchangeIn().pushMessage(
					CommMessageType::DbgPropulsionSetPose,
					m_vars + 4 * op.arg1, 12);
		}
			m_pc++;
			return true;
	case 128://propulsion.pointTo
	{
		float params[5];
		params[0] = *(float*)(m_vars + 4 * op.arg1);
		params[1] = *(float*)(m_vars + 4 * (op.arg1+1));
		params[2] = 0.5;
		params[3] = 0.5;
		params[4] = 0.5;
		Robot::instance().mainExchangeIn().pushMessage(
				CommMessageType::DbgPropulsionExecutePointTo,
				(unsigned char*) params, sizeof(params));
	}
		m_moving = true;
		m_pc++;
		return false;
	case 129://propulsion.move_to
	{
		float params[5];
		params[0] = *(float*)(m_vars + 4 * op.arg1);
		params[1] = *(float*)(m_vars + 4 * (op.arg1+1));
		params[2] = 0.5;
		params[3] = 0.5;
		params[4] = 0.5;
		Robot::instance().mainExchangeIn().pushMessage(
				CommMessageType::DbgPropulsionExecuteMoveTo,
				(unsigned char*) params, sizeof(params));
		}
		m_moving = true;
		m_pc++;
		return false;
		break;
	case 30:
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
	case 31:
		m_call_stack[m_stack_level] = m_pc;
		m_stack_level++;
		m_pc = m_sequence_offsets[op.arg1];
		return true;

	case 140:
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
	case 141:
		{
			unsigned char buff[4];
			buff[0] = op.arg1;
			buff[1] = op.arg2;
			*(uint16_t*)(buff+2) = 1000;
			Robot::instance().mainExchangeIn().pushMessage(
					CommMessageType::DbgArmsGoToPosition,
					buff,
					4);
		}
		m_pc++;
		return true;
	case 142:
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
	default:
		m_pc++;
		return false;

	}
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

}
