#pragma once
#include <cstdint>

namespace goldobot
{
	// Forward declarations
	typedef uint32_t TickType_t;


	class Hal
	{
	public:
		enum class Status
		{
			Error = 0,
			Ok = 1
		};

	public:
		//! \brief Setup peripherals.
		static
		void  init();

		//! \brief Configure peripherals
		static
		void configure(void* config);

		static TickType_t get_tick_count();

		static
		bool gpio_get(int gpio_id);

		static
		void gpio_set(int gpio_id, bool value);

		static
		void pwm_set(int pwm_id, float value);

		static
		uint16_t encoder_get(int encoder_id);

		static
		uint16_t encoder_set(int encoder_id, uint16_t value);


		//! \brief set motors enable
		static
		void motors_set_enable(bool enabled);

		//! \brief Set motors pwm, range is [-1,+1]
		static
		void motors_set_pwm(float left, float right);

		static
		uint16_t uart_read(int fd, uint8_t* buffer, uint16_t buffer_size);

		static
		uint16_t uart_write(int fd, const uint8_t* buffer, uint16_t buffer_size);

		static
		uint16_t uart_write_space_available(int fd);



		static
		void send_spi_frame(unsigned char* buff_out, unsigned char* buff_in);

		static Status spi_message(int fd, uint8_t* rx_buff, uint16_t rx_size, const uint8_t* tx_buff, uint16_t tx_size);

		static Status i2c_memory_read(int fd, uint16_t dev_address, uint16_t mem_address, uint16_t mem_address_size, uint8_t* buffer, uint16_t size);
		static Status i2c_memory_write(int fd, uint16_t dev_address, uint16_t mem_address, uint16_t mem_address_size, const uint8_t* buffer, uint16_t size);
	};
}
