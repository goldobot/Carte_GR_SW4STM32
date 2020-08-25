#include "goldobot/robot_simulator.hpp"
#include "goldobot/comm_deserializer.hpp"
#include "goldobot/comm_serializer.hpp"

#define ZMQ_STATIC
#include "zmq.h"

#include <vector>
#include <mutex>
#include <cstdint>
#include <chrono>

class UARTEmulator
{

};


class RobotEmulator
{
public:
	RobotEmulator();
	void init();

	static RobotEmulator& instance();

	void read_encoders(uint16_t& left, uint16_t& right)  const noexcept;


	uint32_t get_tick_count() const noexcept;

	void main_loop();


	std::mutex m_mutex;

	goldobot::RobotSimulator m_robot_simulator;
	std::chrono::time_point<std::chrono::steady_clock> m_init_timestamp;


	unsigned char m_uart0_deserializer_buffer[4096];
	unsigned char m_uart0_serializer_buffer[4096];

	goldobot::CommDeserializer m_uart0_deserializer;
	goldobot::CommSerializer m_uart0_serializer;

	std::vector<uint8_t> m_uart0_tx_buffer;

	std::vector<uint8_t> m_uart0_rx_buffer;
	size_t m_uart0_rx_recv_size{0};
	size_t m_uart0_rx_recv_bytes_received{ 0 };
	char* m_uart0_rx_recv_buffer{ nullptr };

	std::chrono::time_point<std::chrono::steady_clock> m_uart0_rx_recv_start_timestamp;


	// zmq communication
	void* m_zmq_context{ nullptr };
	void* m_pub_socket{ nullptr };
	void* m_sub_socket{ nullptr };


	static RobotEmulator s_instance;

};