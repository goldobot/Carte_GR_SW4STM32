#include "goldobot/hal.hpp"
#include "goldobot/robot_simulator.hpp"

#include "robot_emulator.hpp"

#include <errno.h>
#include <math.h>

#include <iostream>
#include <iomanip>

using namespace goldobot;



void Hal::init()
{

#ifdef SIMULATE_ROBOT
	// Init simulator
	RobotSimulatorConfig simulator_config;
	simulator_config.speed_coeff = 1.7f; // Measured on big robot
	simulator_config.wheels_spacing = 0.2f;
	simulator_config.encoders_spacing = 0.3f;
	simulator_config.encoders_counts_per_m = 1 / 1.5e-05f;
	s_robot_simulator.m_config = simulator_config;
#endif
}

TickType_t Hal::get_tick_count()
{
	return RobotEmulator::instance().get_tick_count();
}


void Hal::read_encoders(uint16_t& left, uint16_t& right)
{
	auto& emu = RobotEmulator::instance();
	std::lock_guard<std::mutex> lck(emu.m_mutex);
	emu.read_encoders(left, right);
}

void Hal::set_motors_enable(bool enabled)
{
	auto& emu = RobotEmulator::instance();
	std::lock_guard<std::mutex> lck(emu.m_mutex);
	emu.m_robot_simulator.m_motors_enabled = enabled;
}

void Hal::set_servo_pwm(uint16_t pwm)
{
	//__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pwm);
}

#if 0 /* FIXME : DEBUG */
extern bool g_goldo_megakill_switch;
void Hal::disable_motors_pwm()
{
	HAL_GPIO_WritePin(MAXON1_DIR_GPIO_Port, MAXON1_DIR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MAXON2_DIR_GPIO_Port, MAXON2_DIR_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}
#endif

void Hal::set_motors_pwm(float left, float right)
{
	auto& emu = RobotEmulator::instance();
	std::lock_guard<std::mutex> lck(emu.m_mutex);
	emu.m_robot_simulator.m_left_pwm = left;
	emu.m_robot_simulator.m_left_pwm = right;
}

bool Hal::uart_transmit(int uart_index, const char* buffer, uint16_t size, bool blocking)
{	
	//std::cout << "transmit_uart " << uart_index << std::endl;


	if (uart_index == 0)
	{		
		auto& emu = RobotEmulator::instance();
		std::lock_guard<std::mutex> lck(emu.m_mutex);
		for (size_t i = 0; i < size; i++)
		{
			emu.m_uart0_tx_buffer.push_back(buffer[i]);
		}		
	}
	return true;
}

bool Hal::uart_transmit_dma(int uart_index, const char* buffer, uint16_t size)
{	
	//std::cout << "transmit_uart_dma " << uart_index << std::endl;
	return true;
}

bool Hal::uart_transmit_finished(int uart_index)
{
	return true;
}

void Hal::uart_wait_for_transmit(int uart_index)
{
	return;	
}

bool Hal::uart_receive(int uart_index, char* buffer, uint16_t size, bool blocking)
{
	if (uart_index == 0)
	{
		auto& emu = RobotEmulator::instance();
		std::lock_guard<std::mutex> lck(emu.m_mutex);
		emu.m_uart0_rx_recv_buffer = buffer;
		emu.m_uart0_rx_recv_size = size;
		emu.m_uart0_rx_recv_start_timestamp = std::chrono::steady_clock::now();
	}
	return false;
}

bool Hal::uart_receive_dma(int uart_index, const char* buffer, uint16_t size)
{	
	return true;
}


bool Hal::uart_receive_finished(int uart_index)
{
	auto& emu = RobotEmulator::instance();

	if (uart_index == 0 && emu.m_uart0_rx_recv_buffer != nullptr)
	{		
		std::lock_guard<std::mutex> lck(emu.m_mutex);

		// bytes received at baudrate since start of receive
		auto nanoseconds_elapsed =(double)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - emu.m_uart0_rx_recv_start_timestamp).count();
		size_t bytes_received_ts = (size_t)(nanoseconds_elapsed * 23040 / 1e9);

		if (emu.m_uart0_rx_buffer.size() >= emu.m_uart0_rx_recv_size && bytes_received_ts >= emu.m_uart0_rx_recv_size)
		{
			std::vector<uint8_t> vec;
			vec.resize(emu.m_uart0_rx_buffer.size() - emu.m_uart0_rx_recv_size);

			memcpy(emu.m_uart0_rx_recv_buffer, emu.m_uart0_rx_buffer.data(), emu.m_uart0_rx_recv_size);
			memcpy(vec.data(), emu.m_uart0_rx_buffer.data() + emu.m_uart0_rx_recv_size, vec.size());

			emu.m_uart0_rx_buffer = std::move(vec);
			emu.m_uart0_rx_recv_size = 0;
			emu.m_uart0_rx_recv_bytes_received= 0;
			emu.m_uart0_rx_recv_buffer = nullptr;
			return true;
		}
	}
	return false;
}

void Hal::uart_wait_for_receive(int uart_index)
{
	return;
}

uint16_t Hal::uart_bytes_received(int uart_index)
{
	return 0;
}
uint16_t Hal::uart_receive_abort(int uart_index)
{
	auto& emu = RobotEmulator::instance();

	if (uart_index == 0 && emu.m_uart0_rx_recv_buffer != nullptr)
	{
		std::lock_guard<std::mutex> lck(emu.m_mutex);

		// bytes received at baudrate since start of receive
		auto nanoseconds_elapsed = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - emu.m_uart0_rx_recv_start_timestamp).count();
		size_t bytes_received_ts = (size_t)(nanoseconds_elapsed * 23040 / 1e9);

		int foo = nanoseconds_elapsed / 1000000;

		size_t bytes_received = min(bytes_received_ts, min(emu.m_uart0_rx_recv_size, emu.m_uart0_rx_buffer.size()));

		std::vector<uint8_t> vec;
		vec.resize(emu.m_uart0_rx_buffer.size() - bytes_received);

		memcpy(emu.m_uart0_rx_recv_buffer, emu.m_uart0_rx_buffer.data(), bytes_received);
		memcpy(vec.data(), emu.m_uart0_rx_buffer.data() + bytes_received, vec.size());

		emu.m_uart0_rx_buffer = std::move(vec);
		emu.m_uart0_rx_recv_size = 0;
		emu.m_uart0_rx_recv_bytes_received = 0;
		emu.m_uart0_rx_recv_buffer = nullptr;
		return bytes_received;
	}

	return 0;
}

void Hal::set_gpio(int gpio_index, bool value)
{	
}

bool Hal::get_gpio(int gpio_index)
{
	return 0;
}

void Hal::send_spi_frame(unsigned char* buff_out, unsigned char* buff_in)
{
}

bool Hal::user_flash_erase(int start_page, int num_pages)
{
	return true;
}



