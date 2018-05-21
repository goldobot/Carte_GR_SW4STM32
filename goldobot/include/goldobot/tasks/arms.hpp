#pragma once
#include "goldobot/tasks/task.hpp"

#include <cstdint>

namespace goldobot
{
	class ArmsTask : public Task
	{
	public:
		ArmsTask();
		const char* name() const override;

		//! \brief Read data from dynamixel registers
		bool dynamixels_read_data(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size);
		bool dynamixels_write_data(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size);
		// \brief Write data to dynamixel buffer. data is transferred to control registers on ACTION packet
		bool dynamixels_reg_write(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size);
		void dynamixels_action();

	//private:
		void dynamixels_transmit_packet(uint8_t id,  uint8_t command,  unsigned char* parameters, uint8_t num_parameters);
		bool dynamixels_receive_packet();
		void taskFunction() override;

		unsigned char m_dynamixels_buffer[256];
		bool m_dynamixels_receive_ok;
		uint8_t m_dynamixels_receive_id;
		uint8_t m_dynamixels_receive_num_parameters;
		uint8_t m_dynamixels_receive_error;
	};
}
