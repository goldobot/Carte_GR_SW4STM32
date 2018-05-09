#pragma once
#include "goldobot/tasks/propulsion.hpp"
#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/tasks/heartbeat.hpp"
#include "goldobot/hal.hpp"

namespace goldobot
{
	class Robot
	{
	public:
		static Robot& instance();
		void init();
		void start();
		SimpleOdometry& odometry();
		PropulsionController& propulsion();
		UARTCommTask& comm();


		OdometryConfig odometryConfig();
		OdometryConfig defaultOdometryConfig();
		void setOdometryConfig(const OdometryConfig& config);
	private:
		PropulsionTask m_propulsion_task;
		UARTCommTask m_comm_task;
		HeartbeatTask m_heartbeat_task;
		OdometryConfig m_odometry_config;
		static Robot s_instance;
	};
}
