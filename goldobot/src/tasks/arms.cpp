#include "goldobot/tasks/arms.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#include <string.h>
#include <algorithm>
#include "FreeRTOS.h"
#include "task.h"

using namespace goldobot;

/** Defines         **/
/** Instruction Set **/
#define AX_PING                     1
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3
#define AX_REG_WRITE                4
#define AX_ACTION                   5
#define AX_RESET                    6
#define AX_SYNC_WRITE               131


struct DynamixelPacketHeader
{
    uint16_t magic;//0xFFFF
    uint8_t id;
    uint8_t length;
    uint8_t command;
};

ArmsTask::ArmsTask():
	m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer))
{
}


const char* ArmsTask::name() const
{
	return "arms";
}

void ArmsTask::taskFunction()
{
	Robot::instance().mainExchangeIn().subscribe({72,78,&m_message_queue});
	Robot::instance().mainExchangeIn().subscribe({160,165,&m_message_queue});

	m_arm_state = ArmState::Inactive;
	unsigned char buff[2];
	buff[0] = 0;
	buff[1] = (unsigned char)m_arm_state;
	Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ArmsStateChange, buff, 2);

	while(1)
	{
		uint32_t clock = xTaskGetTickCount();
		if(m_arm_state == ArmState::Moving && clock >= m_end_move_timestamp )
		{
			m_arm_state = ArmState::Idle;
			unsigned char buff[2];
			buff[0] = 0;
			buff[1] = (unsigned char)m_arm_state;
			Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ArmsStateChange, buff, 2);
		}
		while(m_message_queue.message_ready() && m_arm_state != ArmState::Moving)
		{
			process_message();

			// Periodically check servo positions and torques
		}
		delay_periodic(1);
	}
}

bool ArmsTask::dynamixels_read_data(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size)
{
	unsigned char tmp_buff[2];
	tmp_buff[0] = address;
	tmp_buff[1] = size;
	dynamixels_transmit_packet(id, AX_READ_DATA, tmp_buff, 2);
	bool received = dynamixels_receive_packet();
	if(received)
	{
		memcpy(buffer, m_dynamixels_buffer + 5, size);
	}
	return received;
}

bool ArmsTask::dynamixels_write_data(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size)
{
	unsigned char tmp_buff[16];
	tmp_buff[0] = address;
	memcpy(tmp_buff+1, buffer, size);
	dynamixels_transmit_packet(id, AX_WRITE_DATA, tmp_buff, size + 1);
	bool received = dynamixels_receive_packet();
	return received;
}

bool ArmsTask::dynamixels_reg_write(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size)
{
	unsigned char tmp_buff[32];
	tmp_buff[0] = address;
	memcpy(tmp_buff+1, buffer, size);
	dynamixels_transmit_packet(id, AX_REG_WRITE, tmp_buff, size + 1);
	bool received = dynamixels_receive_packet();
	return received;
}

void ArmsTask::dynamixels_action()
{
	dynamixels_transmit_packet(0xFE, AX_ACTION, nullptr, 0);
}

void ArmsTask::go_to_position(uint8_t pos_id, uint16_t time_ms, int torque_settings)
{
	time_ms = 1000;
	int pos_idx = pos_id * 3;

	// Launch dynamixels
	uint8_t servo_ids[] = {81,82,1};
	uint8_t servo_types[3] = {1,1,0};

	//get previous positions and compute move timing based on limiting speed
	uint16_t prev_posa[3];

	uint16_t tim = 1;

	for(int i=0; i< 3;i++)
	{
		uint16_t prev_pos = m_current_position[i];
		uint16_t tar_pos = m_config.m_positions[pos_idx+i];
		dynamixels_read_data(servo_ids[i],0x24,(unsigned char*)&prev_pos, 2);
		prev_posa[i] = prev_pos;
		int diff_angle = abs(tar_pos - prev_pos);
		switch(servo_types[i])
		{
		case 0:
			//ax 12
			tim = std::max<uint16_t>((diff_angle*25000)/(57 * 0x3ff), tim);
			break;
		case 1:
			//mx28
			tim = std::max<uint16_t>((diff_angle*128)/0x3ff, tim);
			break;
		default:
			break;
		}
	}

	time_ms = tim*2;
	for(int i=0; i< 3;i++)
	{
		uint16_t buff[3];
		uint16_t prev_pos = m_current_position[i];
		buff[0] = m_config.m_positions[pos_idx+i]; // position setpoint
		buff[2] = 1023;//m_config.m_torque_settings[3*torque_settings+i]; // torque limit

		prev_pos = prev_posa[i];

		int diff_angle = abs(buff[0] - prev_pos);

		switch(servo_types[i])
		{
		case 0:
			//ax 12
			buff[1] = std::min<uint16_t>((diff_angle*25000)/(time_ms*57), 0x3ff);
			break;
		case 1:
			//mx28
			buff[1] = std::min<uint16_t>((diff_angle*128)/(time_ms), 0x3ff);
			break;
		default:
			break;
		}
		// write new register values
		dynamixels_reg_write(servo_ids[i],0x1E,(unsigned char*)buff, 6);
	}
	dynamixels_action();

	// Set time of end


	m_arm_state = ArmState::Moving;
	unsigned char buff[2];
	buff[0] = 0;
	buff[1] = (unsigned char)m_arm_state;
	Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ArmsStateChange, buff, 2);

	m_end_move_timestamp = xTaskGetTickCount() + (uint32_t)time_ms;

}

void ArmsTask::process_message()
{
	switch(m_message_queue.message_type())
	{

	case CommMessageType::DbgDynamixelsList:
		{
			m_message_queue.pop_message(nullptr, 0);
			uint8_t buff[4] = {25,1};
			for(unsigned id = 0; id < 0xFE; id++)
			{
				if(dynamixels_read_data(id,0 , buff, 4))
				{
					Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DbgDynamixelDescr, (unsigned char*)buff, 4);
				}
			}
		}
		break;
	case CommMessageType::DbgDynamixelSetTorqueEnable:
		{
			unsigned char buff[2];
			m_message_queue.pop_message(buff, 2);
			// Torque enable
			dynamixels_write_data(buff[0], 0x18, buff+1, 1);
		}
		break;
	case CommMessageType::DbgDynamixelSetGoalPosition:
		{
			unsigned char buff[3];
			m_message_queue.pop_message(buff, 3);
			// Goal position
			dynamixels_write_data(buff[0], 0x1E, buff+1, 2);
		}
		break;
	case CommMessageType::DbgDynamixelSetTorqueLimit:
		{
			unsigned char buff[3];
			m_message_queue.pop_message(buff, 3);
			// Goal position
			dynamixels_write_data(buff[0], 0x22, buff+1, 2);
		}
		break;
	case CommMessageType::DbgDynamixelGetRegisters:
		{
			unsigned char buff[3];
			unsigned char data_read[64];

			m_message_queue.pop_message(buff, 3);
			memcpy(data_read, buff, 2);
			if(dynamixels_read_data(buff[0], buff[1], data_read+2, buff[2]))
			{
				Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DbgDynamixelGetRegisters, (unsigned char*)data_read, buff[2]+2);
			}
		}
		break;
	case CommMessageType::DbgDynamixelSetRegisters:
		{
			unsigned char buff[128];
			uint16_t size = m_message_queue.message_size();
			m_message_queue.pop_message(buff, 128);
			//id, addr, data
			if(dynamixels_write_data(buff[0], buff[1], buff+2, size-2));
		}
		break;
	case CommMessageType::DbgArmsSetPose:
		{
			unsigned char buff[8];
			m_message_queue.pop_message((unsigned char*)buff,8);
			uint16_t* ptr = m_config.m_positions + 3*buff[1];
			memcpy(ptr, (unsigned char*)(buff+2), 6);
		}
		break;
	case CommMessageType::DbgArmsSetTorques:
		{
			uint16_t buff[4];
			m_message_queue.pop_message((unsigned char*)buff,8);
			uint16_t* ptr = m_config.m_torque_settings + 3*buff[0];
			memcpy(ptr, (unsigned char*)(buff+1), 6);
		}
		break;
	case CommMessageType::DbgArmsGoToPosition:
		{
			unsigned char buff[4];
			m_message_queue.pop_message((unsigned char*)buff,4);
			go_to_position(buff[0], *(uint16_t*)(buff+2), buff[1]);
		}
		break;

	default:
		m_message_queue.pop_message(nullptr, 0);
		break;
	}
}

