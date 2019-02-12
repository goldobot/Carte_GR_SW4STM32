#pragma once

#include "goldobot/tasks/task.hpp"
#include <cstdint>

/******************************************************************************/
/************************** ADXRS453 Definitions ******************************/
/******************************************************************************/

#define ADXRS453_SLAVE_ID       1

#define DATA_SIZE               4   //4 bytes = 32 bits
#define PARITY_BIT              0x01 //parity check on first bit
#define STATUS_MASK             0x0C
#define FIRST_BYTE_DATA_MASK    0x03 //mask to find sensor data bits on first byte
#define THIRD_BYTE_DATA_MASK    0xFC //mask to find sensor data bits on third byte
#define WRITE_COMMAND           0x20 //0010 0000

#define ADXRS453_READ           (1 << 7)
#define ADXRS453_WRITE          (1 << 6)
#define ADXRS453_SENSOR_DATA    (1 << 5)

#define ADXRS453_REG_RATE       0x00
#define ADXRS453_REG_TEM        0x02
#define ADXRS453_REG_LOCST      0x04
#define ADXRS453_REG_HICST      0x06
#define ADXRS453_REG_QUAD       0x08
#define ADXRS453_REG_FAULT      0x0A
#define ADXRS453_REG_PID        0x0C
#define ADXRS453_REG_SN_HIGH    0x0E
#define ADXRS453_REG_SN_LOW     0x10

namespace goldobot
{
	class GyroTask : public Task
	{
	public:
		GyroTask();
		const char* name() const override;
		void taskFunction() override;

		int goldo_gyro_master_spi_read_word (unsigned int apb_addr, unsigned int *pdata);
		int goldo_gyro_master_spi_write_word (unsigned int apb_addr, unsigned int data);
		unsigned int goldo_gyro_get_id();
		float goldo_gyro_get_temperature();
		float goldo_gyro_get_rate();
		float goldo_gyro_get_angle();
		float goldo_gyro_get_bias();
		unsigned short goldo_gyro_get_part_id();

	private:
		int init_status = 0;
		unsigned short part_id = 0.0;
		float temperature = 0.0;
		float rate = 0.0;
		float bias = 0.0;
		float angle = 0.0;
		unsigned char spi_buf_out[256];
		unsigned char spi_buf_in[256];
		int goldo_gyro_send_spi_frame(void);
	};
}
