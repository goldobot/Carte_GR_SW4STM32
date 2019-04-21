#include "goldobot/tasks/arms.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#include <string.h>
#include <algorithm>
#include "FreeRTOS.h"
#include "task.h"

using namespace goldobot;

const uint16_t c_left_arm_default_positions[5]={
	780,
	2100,
	1320,
	760,
	355
};

const uint16_t c_right_arm_default_positions[5]={
	244,
	1996,
	2776,
	264,
	669
};

const uint16_t c_grabber_default_positions[2]={
		95,
		34500
};

const uint16_t c_bascule_default_positions[2]={
		767,
		295
};

const uint16_t c_columns_default_positions[4]={
	21000,
	21000,
	46500,
	49500
};

const uint16_t c_cubibox_default_positions[2]={
		21000,
		21000
};

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
	while(1)
	{
		while(m_message_queue.message_ready())
		{
			process_message();
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
	default:
		m_message_queue.pop_message(nullptr, 0);
		break;
	}
}

