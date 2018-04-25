#pragma once
#include "goldobot/tasks/propulsion.hpp"
#include "goldobot/tasks/uart_comm.hpp"
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

		OdometryConfig odometryConfig();
		OdometryConfig defaultOdometryConfig();
		void setOdometryConfig(const OdometryConfig& config);
	private:
		PropulsionTask m_propulsion_task;
		OdometryConfig m_odometry_config;
		static Robot s_instance;
	};
}
