#pragma once
#include "goldobot/core/message_exchange.hpp"
#include "goldobot/enums.hpp"
#include "goldobot/config.hpp"
#include "goldobot/tasks/propulsion.hpp"
#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/tasks/uart2_comm.hpp"
#include "goldobot/tasks/heartbeat.hpp"
#include "goldobot/tasks/rttelemetry.hpp"
#include "goldobot/tasks/arms.hpp"
#include "goldobot/tasks/main.hpp"
#include "goldobot/tasks/fpga.hpp"
#include "goldobot/hal.hpp"

#include <cstdint>

namespace goldobot
{




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

		uint32_t sensorsState() const { return m_sensors_state;};
		void setSensorsState(uint32_t state) { m_sensors_state = state;};

		void setStartMatchTime(uint32_t time) {m_start_match_time = time;};

		int remainingMatchTime() const { return m_remaining_match_time;};
		void setRemainingMatchTime(int t) { m_remaining_match_time = t;};

		SimpleOdometry& odometry();

		MessageExchange& mainExchangeIn() { return m_main_exchange_in; };
		MessageExchange& mainExchangeOut() { return m_main_exchange_out; };


		const RobotConfig& robotConfig() const;

		ArmConfig* armConfig(int arm_id);
		ServosConfig* servosConfig();

		OdometryConfig odometryConfig();

		PropulsionControllerConfig defaultPropulsionControllerConfig();
		void setOdometryConfig(const OdometryConfig& config);

		PropulsionController::State propulsionState();

		// Synopsis: beginLoad, multiple calls to loadConfig to write config buffer, endLoadConfig to check
		void beginLoadConfig();
		void loadConfig(char* buffer, size_t size);
		bool endLoadConfig(uint16_t crc);



	private:
		Side m_side{Side::Unknown};
		MatchState m_match_state{MatchState::Unconfigured};

		std::atomic<int> m_start_match_time{0};
		std::atomic<int> m_remaining_match_time{0};
		uint32_t m_sensors_state{0};

		PropulsionTask m_propulsion_task;
		UARTCommTask m_comm_task;
		//UART2CommTask m_comm2_task;
		HeartbeatTask m_heartbeat_task;
		RtTelemetryTask m_rt_telemetry_task;


		OdometryConfig* m_odometry_config;
		RobotConfig* m_robot_config;
		PropulsionControllerConfig* m_propulsion_controller_config;
		ServosConfig* m_servos_config;

		unsigned char* m_load_config_ptr{0};
		uint16_t m_load_config_crc;


		MainTask m_main_task;
		ArmsTask m_arms_task;
		FpgaTask m_fpga_task;

		MessageExchange m_main_exchange_in;
		MessageExchange m_main_exchange_out;

		static unsigned char s_config_area[8192];
		static Robot s_instance;
	};
}
