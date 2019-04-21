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
	m_num_arms(0)
{
	for(unsigned i=0;i < c_num_arms; i++)
	{
		m_arms_moving[i] = false;
		m_arms_current_position[i] = 0;
	}
	// left arm
	m_servo_descrs[0] = {4,0,0,1023}; //left slider
	m_servo_descrs[1] = {83,1,0,4095}; // left  rotation
	m_servo_descrs[2] = {84,1,0,4095}; //left shoulder
	m_servo_descrs[3] = {5,0,0,1023}; // left elbow
	m_servo_descrs[4] = {6,0,0,1023}; //left head

	//right arm
	m_servo_descrs[5] = {1,0,0,1023}; //left slider
	m_servo_descrs[6] = {81,1,0,4095}; // left  rotation
	m_servo_descrs[7] = {82,1,0,4095}; //left shoulder
	m_servo_descrs[8] = {2,0,0,1023}; // left elbow
	m_servo_descrs[9] = {3,0,0,1023}; //left head

	//pince
	m_servo_descrs[10] = {101,0,0,1023}; // grabber servo
	m_servo_descrs[11] = {7,2,0,65535}; //grabber pincer

	//bascule
	m_servo_descrs[12] = {7,0,0,1023}; // bascule
	m_servo_descrs[13] = {8,0,0,1023}; // gache

	//colonnes
	m_servo_descrs[14] = {1,2,0,65535}; // gache gauche
	m_servo_descrs[15] = {0,2,0,65535}; // gache droite
	m_servo_descrs[16] = {3,2,0,65535}; // porte gauche
	m_servo_descrs[17] = {2,2,0,65535}; // porte droite

	m_servo_descrs[18] = {4,2,0,65535}; // cubibox gauche
	m_servo_descrs[19] = {5,2,0,65535}; // cubibox droite

	register_arm(5,64,0,c_left_arm_default_positions);
	register_arm(5,64,1,c_right_arm_default_positions);
	register_arm(2,16,0,c_grabber_default_positions);
	register_arm(2,8,0,c_bascule_default_positions);
	register_arm(4,8,0,c_columns_default_positions);
	register_arm(2,8,0,c_cubibox_default_positions);
}

void ArmsTask::register_arm(unsigned num_servos, unsigned num_positions, unsigned pump_id, const uint16_t* default_positions)
{
	uint16_t position_idx_begin=0;
	uint8_t servo_idx_begin = 0;
	if(m_num_arms > 0)
	{
		auto& last_descr = m_arms_descrs[m_num_arms-1];
		position_idx_begin = last_descr.positions_idx_begin + last_descr.num_positions * last_descr.num_servos;
		servo_idx_begin = last_descr.servo_idx_begin + last_descr.num_servos;
	}
	m_arms_descrs[m_num_arms] = {servo_idx_begin, num_servos, position_idx_begin, num_positions, pump_id};
	auto& descr = m_arms_descrs[m_num_arms];
	m_num_arms++;
	for(unsigned i=0; i < descr.num_positions; i++)
	{
		for(unsigned j=0; j < descr.num_servos; j++)
		{
			m_arms_positions[descr.positions_idx_begin + i * descr.num_servos + j] = default_positions[j];
		}
	}
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
		for(int arm_id = 0; arm_id<c_num_arms; arm_id++)
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
	auto& arm_descr = m_arms_descrs[arm_id];

	switch(command.type)
	{
	case 0:
		break;
	case 1:
		go_to_position(arm_id, command.pos_id, command.delay_ms, command.torque_setting);
		break;
	case 2:
		Robot::instance().fpgaTask().goldo_fpga_cmd_motor(arm_descr.pump_idx, command.pump_pwm);
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
	if(arm_id == 6) /* GOLDO : support colonnes : cas special */
	{
		if(pos_id == 0)
		{
			Robot::instance().fpgaTask().goldo_fpga_columns_calib();
		}
		else
		{
			Robot::instance().fpgaTask().goldo_fpga_columns_move(pos_id);
		}
		return;
	}

	auto& arm_descr = m_arms_descrs[arm_id];

	if(pos_id >= arm_descr.num_positions)
	{
		return;
	}

	int pos_idx = pos_id * arm_descr.num_servos + arm_descr.positions_idx_begin;
	int prev_pos_idx =  m_arms_current_position[arm_id] * arm_descr.num_servos + arm_descr.positions_idx_begin;
	m_arms_current_position[arm_id] = pos_id;

	// Launch dynamixels
	for(int i=0; i< arm_descr.num_servos;i++)
	{
		auto& servo_descr = m_servo_descrs[i+arm_descr.servo_idx_begin];
		uint16_t buff[3];
		uint16_t prev_pos = m_arms_positions[prev_pos_idx+i];
		buff[0] = m_arms_positions[pos_idx+i];
		buff[2] = 500; //torque limit, \todo adjust in sequence, and add debug mode limit

		if(servo_descr.servo_type < 2)
		{
			dynamixels_read_data(servo_descr.servo_id,0x24,(unsigned char*)&prev_pos, 2);
		}
		int diff_angle = abs(m_arms_positions[pos_idx+i] - prev_pos);

		switch(servo_descr.servo_type)
		{
		case 0:
			//ax 12
			buff[1] = std::min<uint16_t>((diff_angle*25000)/(time_ms*57), 0x3ff);
			dynamixels_reg_write(servo_descr.servo_id,0x1E,(unsigned char*)buff, 6);
			break;
		case 1:
			//mx28
			buff[1] = std::min<uint16_t>((diff_angle*128)/(time_ms), 0x3ff);
			dynamixels_reg_write(servo_descr.servo_id,0x1E,(unsigned char*)buff, 6);
			break;
		case 2:
			//goldo servo
			Robot::instance().fpgaTask().goldo_fpga_cmd_servo(servo_descr.servo_id, buff[0]);
			break;
		default:
			break;
		}
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


