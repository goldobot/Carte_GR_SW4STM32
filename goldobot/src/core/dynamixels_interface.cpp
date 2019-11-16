
#include "goldobot/hal.hpp"
#include "goldobot/tasks/arms.hpp"
#include <string.h>


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


namespace goldobot {



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

DynamixelStatusError ArmsTask::dynamixels_receive_packet()
{
	memset(m_dynamixels_buffer, 0, 255);
	Hal::uart_receive(1,(char*)m_dynamixels_buffer,256, false);
	uint16_t bytes_received = 0;
	for(unsigned i=0;i<10;i++)
	{
		bytes_received = Hal::uart_bytes_received(1);

		// search for magic 0xFF
		if(bytes_received >= 4 && bytes_received >= m_dynamixels_buffer[3] + 4)
		{
			Hal::uart_receive_abort(1);

			// Check checksum
			m_dynamixels_receive_id = m_dynamixels_buffer[2];
			m_dynamixels_receive_num_parameters = m_dynamixels_buffer[3] - 2;
			m_dynamixels_receive_error = m_dynamixels_buffer[4];
			m_dynamixels_receive_params = m_dynamixels_buffer + 5;

			uint8_t checksum = 0;
			for(unsigned i = 2; i < 5 + m_dynamixels_receive_num_parameters;i++)
			{
				checksum += m_dynamixels_buffer[i];
			}
			checksum = ~checksum;

			if(checksum == m_dynamixels_buffer[5 + m_dynamixels_receive_num_parameters])
			{
				return DynamixelStatusError::Ok;
			} else
			{
				return DynamixelStatusError::ChecksumError;
			}
		}
		delay(1);
	}
	Hal::uart_receive_abort(1);
	return DynamixelStatusError::TimeoutError;
}


}
