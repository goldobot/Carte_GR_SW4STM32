#pragma once
#include <cstdint>

namespace goldobot
{
	// Forward declarations
	class SimpleOdometry;
	struct OdometryConfig;

	class Hal
	{
	public:
		//! \brief Setup peripherals.
		static
		void  init();

		static
		void set_servo_pwm(uint16_t pwm);

		//! \brief read wheel encoders value
		static
		void read_encoders(uint16_t& left, uint16_t& right);

		//! \brief set motors enable
		static
		void set_motors_enable(bool enabled);

		//! \brief Set motors pwm, range is [-1,+1]
		static
		void set_motors_pwm(float left, float right);

#if 1 /* FIXME : DEBUG */
		static
		void disable_motors_pwm();
#endif
		static
		bool uart_read_char(int uart_index, char* c, bool blocking);

		static
		bool uart_transmit(int uart_index, const char* buffer, uint16_t size, bool blocking = true);

		static
		bool uart_transmit_dma(int uart_index, const char* buffer, uint16_t size);

		static
		bool uart_transmit_finished(int uart_index);

		static
		void uart_wait_for_transmit(int uart_index);

		static
		bool uart_receive(int uart_index, const char* buffer, uint16_t size, bool blocking = true);

		static
		bool uart_receive_finished(int uart_index);

		static
		void uart_wait_for_receive(int uart_index);

		static
		uint16_t uart_bytes_received(int uart_index);

		static
		uint16_t uart_receive_abort(int uart_index);

		static
		void set_gpio(int gpio_index, bool value);

		static
		bool get_gpio(int gpio_index);

		static
		bool user_flash_erase(int start_page, int num_pages);

		static
		bool user_flash_read(uint16_t start_offset, char* buffer, uint16_t size);

		static
		void simulation_step();
	};
}
