#pragma once

#include "goldobot/tasks/task.hpp"
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

	private:
		unsigned char spi_buf_out[256];
		unsigned char spi_buf_in[256];
		int goldo_fpga_send_spi_frame(void);
	};
}