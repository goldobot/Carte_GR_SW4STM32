#pragma once
#include <cstdint>
#include <cstddef>

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

		static
		uint8_t* uart_lock_read(size_t& buffer_size);

		static
	    void uart_unlock_read(size_t read_size);

		static
		uint8_t* uart_lock_write(size_t& buffer_size);

		static
		void uart_unlock_write(size_t written_size);

		static
		bool uart_transmit(int uart_index, const char* buffer, uint16_t size, bool blocking = true);

		static
		bool uart_transmit_dma(int uart_index, const char* buffer, uint16_t size);

		static
		void uart_wait_for_transmit(int uart_index);

		static
		bool uart_receive(int uart_index, const char* buffer, uint16_t size, bool blocking = true);

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
	};
}
