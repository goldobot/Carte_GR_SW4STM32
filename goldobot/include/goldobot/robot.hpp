#pragma once
#include "goldobot/core/message_exchange.hpp"
#include "goldobot/tasks/propulsion.hpp"
#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/tasks/heartbeat.hpp"
#include "goldobot/tasks/arms.hpp"
#include "goldobot/tasks/main.hpp"
#include "goldobot/tasks/fpga.hpp"
#include "goldobot/hal.hpp"

namespace goldobot
{
	struct RobotConfig
	{
		//! \brief distance from wheels axis to front of the robot
		float front_length;
		//! \brief distance from wheels axis to back of the robot
		float back_length;
	};

	class Robot
	{
	public:
		static Robot& instance();
		void init();
		void start();
		SimpleOdometry& odometry();
		PropulsionController& propulsion();
		UARTCommTask& comm();
		ArmsTask& arms();
		MainTask& mainTask();
		FpgaTask& fpgaTask();

		MessageExchange& mainExchange() { return m_main_exchange; };

		const RobotConfig& robotConfig() const;
		OdometryConfig odometryConfig();
		OdometryConfig defaultOdometryConfig();
		PropulsionControllerConfig defaultPropulsionControllerConfig();
		void setOdometryConfig(const OdometryConfig& config);
	private:
		PropulsionTask m_propulsion_task;
		UARTCommTask m_comm_task;
		HeartbeatTask m_heartbeat_task;
		OdometryConfig m_odometry_config;
		RobotConfig m_robot_config;
		MainTask m_main_task;
		ArmsTask m_arms_task;
		FpgaTask m_fpga_task;

		MessageExchange m_main_exchange;

		static Robot s_instance;
	};
}
