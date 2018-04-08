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
		const SimpleOdometry& odometry() const;

		OdometryConfig odometryConfig();
		OdometryConfig defaultOdometryConfig();
		void setOdometryConfig(OdometryConfig& config);
	private:
		PropulsionTask m_propulsion_task;
		static Robot s_instance;
	};
}
