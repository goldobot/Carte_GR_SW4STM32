#pragma once
#include "goldobot/tasks/task.hpp"
#include "goldobot/core/message_queue.hpp"
#include <cstdint>

namespace goldobot
{
	class FpgaTask : public Task
	{
	public:
		FpgaTask();
		const char* name() const override;
		void taskFunction() override;

		int goldo_fpga_master_spi_read_word (unsigned int apb_addr, unsigned int *pdata);
		int goldo_fpga_master_spi_write_word (unsigned int apb_addr, unsigned int data);
		unsigned int goldo_fpga_get_version (void);
		int goldo_fpga_cmd_servo (int servo_id, unsigned int new_pos);// range 0 0x40000
		int goldo_fpga_cmd_motor (int motor_id, int new_val);//val -511,511 0 pompe droite 1 pompe gauche 2 tapis
		int goldo_fpga_cmd_stepper (int stp_id, unsigned int new_pos);
		int goldo_fpga_get_stepper_pos (int stp_id, unsigned int *new_pos);
		int goldo_fpga_columns_calib (void);
		int goldo_fpga_columns_move (int col_id);
		int goldo_fpga_set_columns_offset (int col_id, int col_offset);

	private:
		unsigned char spi_buf_out[64];
		unsigned char spi_buf_in[64];
		int goldo_fpga_send_spi_frame(void);
		void process_message();

		MessageQueue m_message_queue;
		unsigned char m_message_queue_buffer[128];

		uint32_t m_sensors_state{0};
	};
}
