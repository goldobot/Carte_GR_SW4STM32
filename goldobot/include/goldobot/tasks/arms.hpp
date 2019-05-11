#pragma once
#include "goldobot/tasks/task.hpp"
#include "goldobot/core/message_queue.hpp"
#include "goldobot/config.hpp"

#include <cstdint>

namespace goldobot
{
struct DynamixelsConfig
{
	uint16_t m_positions[3*32];
	uint16_t m_torque_settings[3*8];
};



class ArmsTask : public Task
{
public:
	ArmsTask();
	const char* name() const override;

	// Dirty provisional api

	void go_to_position(uint8_t pos_id, uint16_t time_ms, int torque_setting=0);

	//! \brief Read data from dynamixel registers
	bool dynamixels_read_data(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size);
	bool dynamixels_write_data(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size);
	// \brief Write data to dynamixel buffer. data is transferred to control registers on ACTION packet
	bool dynamixels_reg_write(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size);
	void dynamixels_action();

	//void _execute_command(int arm_id, const ArmCommand& command);

//private:
	void dynamixels_transmit_packet(uint8_t id,  uint8_t command,  unsigned char* parameters, uint8_t num_parameters);
	bool dynamixels_receive_packet();
	void taskFunction() override;

	DynamixelsConfig m_config;
	uint16_t m_current_position[3];
	uint16_t m_current_load[3];


	unsigned char m_dynamixels_buffer[256];
	bool m_dynamixels_receive_ok;
	uint8_t m_dynamixels_receive_id;
	uint8_t m_dynamixels_receive_num_parameters;
	uint8_t m_dynamixels_receive_error;
	uint8_t m_dynamixels_receive_offset;

	MessageQueue m_message_queue;
	unsigned char m_message_queue_buffer[256];

	uint16_t m_dynamixels_positions[3];
	uint16_t m_dynamixels_loads[3];

	uint32_t m_end_move_timestamp;
	ArmState m_arm_state{ArmState::Unconfigured};

	void process_message();
};
}
