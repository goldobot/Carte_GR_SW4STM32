#pragma once
#include "goldobot/core/message_exchange.hpp"
#include "goldobot/enums.hpp"
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

		Side side() const noexcept { return m_side;};
		void setSide(Side side) noexcept {m_side = side;};

		MatchState matchState() const noexcept { return m_match_state;};
		void setMatchState(MatchState state) noexcept {m_match_state = state;};

		void setStartMatchTime(uint32_t time) {m_start_match_time = time;};

		SimpleOdometry& odometry();

		MessageExchange& mainExchangeIn() { return m_main_exchange_in; };
		MessageExchange& mainExchangeOut() { return m_main_exchange_out; };

		const RobotConfig& robotConfig() const;
		OdometryConfig odometryConfig();
		OdometryConfig defaultOdometryConfig();
		PropulsionControllerConfig defaultPropulsionControllerConfig();
		void setOdometryConfig(const OdometryConfig& config);

	private:
		Side m_side{Side::Unknown};
		MatchState m_match_state{MatchState::Idle};

		std::atomic<int> m_start_match_time{0};

		PropulsionTask m_propulsion_task;
		UARTCommTask m_comm_task;
		HeartbeatTask m_heartbeat_task;
		OdometryConfig m_odometry_config;
		RobotConfig m_robot_config;
		MainTask m_main_task;
		ArmsTask m_arms_task;
		FpgaTask m_fpga_task;

		MessageExchange m_main_exchange_in;
		MessageExchange m_main_exchange_out;

		static Robot s_instance;
	};
}
