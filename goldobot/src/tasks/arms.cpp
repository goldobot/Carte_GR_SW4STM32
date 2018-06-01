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
		m_arms_moving{false, false},
		m_arms_current_position{0,0}
{
	m_arms_servo_ids[0] = 4; //left slider
	m_arms_servo_ids[1] = 83; // left  rotation
	m_arms_servo_ids[2] = 84; //left shoulder
	m_arms_servo_ids[3] = 5; // left elbow
	m_arms_servo_ids[4] = 6; //left head

	m_arms_servo_ids[5] = 1; //left slider
	m_arms_servo_ids[6] = 81; // left  rotation
	m_arms_servo_ids[7] = 82; //left shoulder
	m_arms_servo_ids[8] = 2; // left elbow
	m_arms_servo_ids[9] = 3; //left head
}

const char* ArmsTask::name() const
{
	return "arms";
}

void ArmsTask::execute_sequence(uint8_t arm_id, uint8_t sequence_id)
{
	m_arms_next_command_ts[arm_id] = xTaskGetTickCount();
	m_arms_current_sequence[arm_id] = sequence_id;
	m_arms_current_idx[arm_id] = m_arms_sequences[m_arms_current_sequence[arm_id]].begin_command_idx;
	m_arms_moving[arm_id] = true;
}

void ArmsTask::taskFunction()
{
	while(1)
	{
		delay_periodic(1);
		uint32_t clock = xTaskGetTickCount();
		for(int arm_id = 0; arm_id<2; arm_id++)
		{
			if(m_arms_moving[arm_id])
			{
				if(clock >= m_arms_next_command_ts[arm_id])
				{
					const auto& seq = m_arms_sequences[m_arms_current_sequence[arm_id]];
					if(m_arms_current_idx[arm_id] < seq.end_command_idx)
					{
						auto& cmd = m_arms_commands[m_arms_current_idx[arm_id]];
						_execute_command(arm_id, cmd);
						m_arms_next_command_ts[arm_id] = clock + cmd.delay_ms;
						m_arms_current_idx[arm_id]++;

					} else
					{
						m_arms_moving[arm_id] = false;
					}
				}
			}
		}
	}
}

void ArmsTask::_execute_command(int arm_id, const ArmCommand& command)
{
	switch(command.type)
	{
	case 0:
		break;
	case 1:
		go_to_position(arm_id, command.pos_id, command.delay_ms, command.torque_setting);
		break;
	case 2:
		Robot::instance().fpgaTask().goldo_fpga_cmd_motor(arm_id, command.pump_pwm);
		break;
	case 3:
		Robot::instance().fpgaTask().goldo_fpga_cmd_motor(2, command.pump_pwm);
		break;
	default:
		break;

	}
}

void ArmsTask::go_to_position(uint8_t arm_id, uint8_t pos_id, uint16_t time_ms, int torque_settings)
{
	int pos_idx = pos_id + 32*arm_id;
	int prev_pos_idx =  m_arms_current_position[arm_id] + 32*arm_id;
	m_arms_current_position[arm_id] = pos_id;
	for(int i=0; i< 5;i++)
	{
		uint16_t buff[3];
		int pos_idx = pos_id + 32*arm_id;
		buff[0] = m_arms_positions[pos_idx*5+i];

		int diff_angle = abs(m_arms_positions[pos_idx*5+i] - m_arms_positions[prev_pos_idx*5+i]);


		if(i == 1 || i ==2)
		{
			// mx28
			buff[1] = std::min<uint16_t>((diff_angle*128)/(time_ms), 0x3ff);

		} else
		{
			// ax12
			buff[1] = std::min<uint16_t>((diff_angle*25000)/(time_ms*57), 0x3ff);
		}
		dynamixels_reg_write(m_arms_servo_ids[i + arm_id * 5],0x1E,(unsigned char*)buff, 4);
	}
	dynamixels_action();
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

void ArmsTask::dynamixels_transmit_packet(uint8_t id,  uint8_t command,  unsigned char* parameters, uint8_t num_parameters)
{
	DynamixelPacketHeader& header = *reinterpret_cast<DynamixelPacketHeader*>(m_dynamixels_buffer);
	header.magic = 0xFFFF;
	header.id = id;
	header.length = num_parameters + 2;
	header.command = command;
	memcpy(m_dynamixels_buffer+5, parameters, num_parameters);

	uint8_t checksum = 0;
	for(unsigned i = 2; i < 5 + num_parameters;i++)
	{
		checksum += m_dynamixels_buffer[i];
	}
	checksum = ~checksum;
	m_dynamixels_buffer[5 + num_parameters] = checksum;
	Hal::set_gpio(3, 1);
	Hal::uart_transmit(1,(char*)m_dynamixels_buffer,6 + num_parameters, true);
	Hal::set_gpio(3, 0);
}

bool ArmsTask::dynamixels_receive_packet()
{
	Hal::uart_receive(1,(char*)m_dynamixels_buffer,256, false);
	for(unsigned i=0;i<20;i++)
	{
		uint16_t bytes_received = Hal::uart_bytes_received(1);
		if(bytes_received >= 4 && bytes_received >= m_dynamixels_buffer[3] + 4)
		{
			// Check checksum
			m_dynamixels_receive_num_parameters = m_dynamixels_buffer[3] - 1;
			Hal::uart_receive_abort(1);
			return true;
		}
		delay(1);
	}
	Hal::uart_receive_abort(1);
	return false;
}
