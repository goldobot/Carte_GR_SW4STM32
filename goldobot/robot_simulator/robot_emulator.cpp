#include "robot_emulator.hpp"

#include "yaml-cpp/yaml.h"

#include <thread>


#define M_PI 3.14159265358979323846264338327950288


RobotEmulator RobotEmulator::s_instance;

RobotEmulator::RobotEmulator() :
	m_uart0_deserializer(m_uart0_deserializer_buffer, sizeof(m_uart0_deserializer_buffer)),
	m_uart0_serializer(m_uart0_serializer_buffer, sizeof(m_uart0_serializer_buffer))
{
	m_uart0_serializer.t_debug = true;
}

RobotEmulator& RobotEmulator::instance()
{
	return RobotEmulator::s_instance;
}

void RobotEmulator::init()
{
	// Parse yaml config
	YAML::Node yaml_config = YAML::LoadFile("D:/cdr2020/Carte_GR_SW4STM32/goldobot/robot_simulator/robot_simulator.yaml");

	goldobot::RobotSimulatorConfig robot_simulator_config;
	robot_simulator_config.speed_coeff = yaml_config["robot_config"]["speed_coeff"].as<float>();
	robot_simulator_config.wheels_spacing = yaml_config["robot_config"]["wheels_spacing"].as<float>();
	robot_simulator_config.encoders_spacing = yaml_config["robot_config"]["encoders_spacing"].as<float>();
	robot_simulator_config.encoders_counts_per_m = yaml_config["robot_config"]["encoders_counts_per_m"].as<float>();

	m_robot_simulator.m_config = robot_simulator_config;
	m_robot_simulator.m_x = yaml_config["initial_pose"]["x"].as<float>();
	m_robot_simulator.m_y = yaml_config["initial_pose"]["y"].as<float>();
	m_robot_simulator.m_yaw = yaml_config["initial_pose"]["yaw"].as<float>() * M_PI/180.0f;

	m_init_timestamp = std::chrono::steady_clock::now();

	m_zmq_context = zmq_ctx_new();
	m_pub_socket = zmq_socket(m_zmq_context, ZMQ_PUB);
	m_sub_socket = zmq_socket(m_zmq_context, ZMQ_SUB);

	zmq_bind(m_sub_socket, "tcp://*:3002");
	zmq_setsockopt(m_sub_socket, ZMQ_SUBSCRIBE, "", 0);


	zmq_bind(m_pub_socket, "tcp://*:3001");

	m_robot_simulator.init(std::chrono::milliseconds(0));

}


void  RobotEmulator::read_encoders(uint16_t& left, uint16_t& right) const noexcept
{
	left = m_robot_simulator.m_left_encoder;
	right = m_robot_simulator.m_right_encoder;
}


uint32_t  RobotEmulator::get_tick_count() const noexcept
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_init_timestamp).count();
}

void RobotEmulator::main_loop()
{
	while (1)
	{
		std::lock_guard<std::mutex> lck(m_mutex);
		if (!m_uart0_tx_buffer.empty())
		{
			m_uart0_deserializer.push_data(m_uart0_tx_buffer.data(), m_uart0_tx_buffer.size());
			m_uart0_tx_buffer.clear();
		}

		while(m_uart0_deserializer.message_ready())
		{
			unsigned char tmp_buffer[1024];
			uint16_t recv_message_type = m_uart0_deserializer.message_type();
			size_t recv_message_size = m_uart0_deserializer.message_size();
			m_uart0_deserializer.pop_message(tmp_buffer, sizeof(tmp_buffer));

			zmq_send(m_pub_socket, (const char*)(&recv_message_type), 2, ZMQ_SNDMORE);
			zmq_send(m_pub_socket, (const char*)(tmp_buffer), recv_message_size, 0);

			if (recv_message_type == 0)
			{
				// Echo synchronization messages
				//serializer.push_message(0, (unsigned char*)"goldobot", 8);
			}
		}

		int64_t pollin{ 0 };
		size_t pollin_size = sizeof(pollin);
		while (zmq_getsockopt(m_sub_socket, ZMQ_EVENTS, &pollin, &pollin_size), pollin & ZMQ_POLLIN)
		{
			unsigned char buff[1024];
			size_t bytes_read = 0;
			int64_t more = 1;
			size_t more_size = sizeof(more);
			while (more)
			{
				bytes_read += zmq_recv(m_sub_socket, buff + bytes_read, sizeof(buff) - bytes_read, 0);
				zmq_getsockopt(m_sub_socket, ZMQ_RCVMORE, &more, &more_size);
			}
			buff[bytes_read] = 0;
			uint16_t message_type = *(uint16_t*)(buff);
			m_uart0_serializer.push_message(message_type, buff + 2, bytes_read - 2);
		}
		unsigned char send_buffer[128];
		size_t dtlen = m_uart0_serializer.pop_data(send_buffer, sizeof(send_buffer));
		for (size_t i = 0; i < dtlen; i++)
		{
			m_uart0_rx_buffer.push_back(send_buffer[i]);
		}

		m_robot_simulator.do_step(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_init_timestamp));
		std::this_thread::yield();
	}
}