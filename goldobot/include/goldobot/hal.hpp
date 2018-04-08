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

		//! \brief read wheel encoders value
		static
		void read_encoders(uint16_t& left, uint16_t& right);

		//! \brief set motors enable
		static
		void set_motors_enable(bool enabled);

		//! \brief Set motors pwm, range is [-1,+1]
		static
		void set_motors_pwm(float left, float right);

		static void uart_transmit();
		static void uart_receive();
	};
}
