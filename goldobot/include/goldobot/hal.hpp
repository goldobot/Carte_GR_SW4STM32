#pragma once
#include <cstdint>

namespace goldobot
{
	// Forward declarations
	typedef uint32_t TickType_t;

	enum class IODeviceType : uint8_t
	{
		None=0,
		Uart=1,
		I2C=2,
		Spi=3,
		Encoder,
		Pwm
	};


	struct IODeviceConfig
	{
		IODeviceType device_type;
		uint8_t fd;
		uint16_t rx_buffer_size;
		uint16_t tx_buffer_size;
		uint16_t device_flags;
	};

	struct IODeviceConfigUART : IODeviceConfig
	{
		uint8_t uart_index;
		uint8_t reserved;
		uint8_t rx_port;
		uint8_t rx_pin;
		uint8_t tx_port;
	    uint8_t tx_pin;
	    uint16_t flags;
		uint32_t baudrate;
	};


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

		static void register_uart(IODeviceConfigUART* config);

		static TickType_t get_tick_count();

		//! \brief read wheel encoders value
		static
		void read_encoders(uint16_t& left, uint16_t& right);

		static
		void encoders_write(uint16_t left, uint16_t right);

		//! \brief set motors enable
		static
		void set_motors_enable(bool enabled);

		//! \brief Set motors pwm, range is [-1,+1]
		static
		void set_motors_pwm(float left, float right);

		static
		uint16_t uart_read(int fd, uint8_t* buffer, uint16_t buffer_size);

		static
		uint16_t uart_write(int fd, const uint8_t* buffer, uint16_t buffer_size);

		static
		uint16_t uart_write_space_available(int fd);

		static
		Status uart_clear_rx_fifo(int fd);

		static
		void set_gpio(int gpio_index, bool value);

		static
		bool get_gpio(int gpio_index);

		static
		void send_spi_frame(unsigned char* buff_out, unsigned char* buff_in);

		static Status spi_message(int fd, uint8_t* rx_buff, uint16_t rx_size, const uint8_t* tx_buff, uint16_t tx_size);

		static Status i2c_memory_read(int fd, uint16_t dev_address, uint16_t mem_address, uint16_t mem_address_size, uint8_t* buffer, uint16_t size);
		static Status i2c_memory_write(int fd, uint16_t dev_address, uint16_t mem_address, uint16_t mem_address_size, const uint8_t* buffer, uint16_t size);
	};
}
