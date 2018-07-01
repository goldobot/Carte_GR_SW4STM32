#pragma once
#include "goldobot/tasks/task.hpp"

#include <cstdint>

namespace goldobot
{
	struct ArmCommand
	{
		uint8_t type;//0, delay, 1, move arm, 2, set pump
		uint8_t reserved;
		union
		{
			struct {
				uint8_t pos_id;
				uint8_t torque_setting;
			};
			int16_t pump_pwm;
		};
		uint16_t delay_ms;
	};

	struct ArmSequence
	{
		uint8_t begin_pos;
		uint8_t end_pos;
		uint8_t begin_command_idx;
		uint8_t end_command_idx;
	};

	struct ArmDescr
	{
		uint8_t servo_idx_begin;
		uint8_t num_servos;
		uint16_t positions_idx_begin;
		uint8_t num_positions;
		uint8_t pump_idx;
	};

	struct ServoDescr
	{
		uint8_t servo_id;
		uint8_t servo_type;
		uint16_t limit_low;
		uint16_t limit_high;
	};

	class ArmsTask : public Task
	{
	public:
		ArmsTask();
		const char* name() const override;

		// Dirty provisional api

		void go_to_position(uint8_t arm_id, uint8_t pos_id, uint16_t time_ms, int torque_setting=0);
		void execute_sequence(uint8_t arm_id, uint8_t sequence_id);

		//! \brief Read data from dynamixel registers
		bool dynamixels_read_data(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size);
		bool dynamixels_write_data(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size);
		// \brief Write data to dynamixel buffer. data is transferred to control registers on ACTION packet
		bool dynamixels_reg_write(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size);
		void dynamixels_action();

		void _execute_command(int arm_id, const ArmCommand& command);

	//private:
		void dynamixels_transmit_packet(uint8_t id,  uint8_t command,  unsigned char* parameters, uint8_t num_parameters);
		bool dynamixels_receive_packet();
		void register_arm(unsigned num_servos, unsigned num_positions, unsigned pump_id, const uint16_t* default_positions);
		void taskFunction() override;

		static constexpr int c_num_arms = 7;
		static constexpr int c_num_servos = 20;
		static constexpr int c_num_positions = 256;

		unsigned char m_dynamixels_buffer[256];
		bool m_dynamixels_receive_ok;
		uint8_t m_dynamixels_receive_id;
		uint8_t m_dynamixels_receive_num_parameters;
		uint8_t m_dynamixels_receive_error;
		uint8_t m_dynamixels_receive_offset;

		ArmDescr m_arms_descrs[c_num_arms];
		ServoDescr m_servo_descrs[c_num_servos];
		uint16_t m_arms_positions[1024];
		uint16_t m_arms_torque_settings[8*5];

		ArmCommand m_arms_commands[128];
		ArmSequence m_arms_sequences[32];

		bool m_arms_moving[c_num_arms];
		uint8_t m_arms_current_position[c_num_arms];
		uint8_t m_arms_current_sequence[c_num_arms];
		uint8_t m_arms_current_idx[c_num_arms];
		uint32_t m_arms_next_command_ts[c_num_arms];

		uint16_t m_num_arms;
	};
}
