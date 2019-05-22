#include "goldobot/robot.hpp"

#include <cstring>

using namespace goldobot;

Robot Robot::s_instance;
unsigned char Robot::s_config_area[4096];

Robot& Robot::instance()
{
	return s_instance;
}

void Robot::init()
{
	m_comm_task.init();
	//First send some sync messages to ensure the reset is well received
	m_main_exchange_out.pushMessage(CommMessageType::Sync,(unsigned char*)"goldobot",8);
	m_main_exchange_out.pushMessage(CommMessageType::Reset,nullptr, 0);
	m_main_task.init();
	m_heartbeat_task.init();
	m_rt_telemetry_task.init();
}

void Robot::start()
{
	m_propulsion_task.start();
}

SimpleOdometry& Robot::odometry()
{
	return m_propulsion_task.odometry();
}

PropulsionControllerConfig Robot::defaultPropulsionControllerConfig()
{
	return *m_propulsion_controller_config;
}
OdometryConfig Robot::odometryConfig()
{
	return *m_odometry_config;
}

const RobotConfig& Robot::robotConfig() const
{
	return *m_robot_config;
}

void Robot::setOdometryConfig(const OdometryConfig& config)
{
	*m_odometry_config = config;
	odometry().setConfig(config);
}

PropulsionController::State Robot::propulsionState()
{
	return m_propulsion_task.controller().state();
}

void Robot::beginLoadConfig()
{
	m_load_config_ptr = s_config_area;
	m_load_config_crc = 0;
}

// Todo proper utils library and declaration
uint16_t update_crc16(const unsigned char* data_p, size_t length, uint16_t crc = 0xFFFF);

void Robot::loadConfig(char* buffer, size_t size)
{
	std::memcpy(m_load_config_ptr, buffer, size);
	m_load_config_ptr+=size;
	m_load_config_crc = update_crc16((unsigned char*)buffer, size, m_load_config_crc);
}

bool Robot::endLoadConfig(uint16_t crc)
{
	// Config binary starts with array of 16 bits offsets to:
	// RobotConfig
	// OdometryConfig
	// PropulsionControllerConfig
	// ArmConfig
	// Arm positions
	// ServosConfig

	uint16_t* offsets = (uint16_t*)s_config_area;

	m_robot_config = (RobotConfig*)(s_config_area + offsets[0]);
	m_odometry_config = (OdometryConfig*)(s_config_area + offsets[1]);
	m_propulsion_controller_config = (PropulsionControllerConfig*)(s_config_area + offsets[2]);

	odometry().setConfig(*m_odometry_config);
	m_propulsion_task.controller().setConfig(defaultPropulsionControllerConfig());
	m_arms_task.m_config = *(ArmConfig*)(s_config_area + offsets[3]);
	m_arms_task.m_config.positions_ptr = (uint16_t*)(s_config_area + offsets[4]);
	m_propulsion_task.init();
	m_arms_task.init();
	m_fpga_task.init();

	start();
	m_match_state = MatchState::Idle;
}

