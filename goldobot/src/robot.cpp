#include "goldobot/robot.hpp"

using namespace goldobot;

Robot Robot::s_instance;
//unsigned char Robot::s_config_area[4096];

Robot& Robot::instance()
{
	return s_instance;
}

void Robot::init()
{
	m_robot_config.front_length = 0.170;
	m_robot_config.back_length = 0.135;

	m_comm_task.init();
	//First send some sync messages to ensure the reset is well received
	m_main_exchange_out.pushMessage(CommMessageType::Sync,(unsigned char*)"goldobot",8);
	m_main_exchange_out.pushMessage(CommMessageType::Reset,nullptr, 0);
	m_heartbeat_task.init();

	setOdometryConfig(defaultOdometryConfig());
	m_propulsion_task.controller().setConfig(defaultPropulsionControllerConfig());
	m_propulsion_task.init();
	m_main_task.init();
	m_arms_task.init();
	m_fpga_task.init();

}

void Robot::start()
{
	m_propulsion_task.start();
}

SimpleOdometry& Robot::odometry()
{
	return m_propulsion_task.odometry();
}

OdometryConfig Robot::defaultOdometryConfig()
{
	OdometryConfig config;
	config.dist_per_count_left = -1.966033264580682e-05;
	config.dist_per_count_right = -1.9621583742116328e-05;
	config.wheel_spacing = 187.8e-3;
	config.encoder_period = 8192;
	config.update_period = 1e-3;
	config.speed_filter_period = 0.005f;
	return config;
}

PropulsionControllerConfig Robot::defaultPropulsionControllerConfig()
{
	PropulsionControllerConfig config;

	// Longi
	config.low_level_config_static.longi_pid_config = {
		1e-3f,  //period
		5,      //kp
		10,		//ki
		0,		//kd
		0,      //feedforward
		0.2,    //lim_iterm
		0,      //lim_dterm
		-1,     //min output
		1	    //max_output
	};
	config.low_level_config_cruise.longi_pid_config = {
		1e-3f,  //period
		5,      //kp
		15,		//ki
		0,		//kd
		0,      //feedforward
		0.2,    //lim_iterm
		0,      //lim_dterm
		-1,     //min output
		1	    //max_output
	};

	config.low_level_config_rotate.longi_pid_config = {
		1e-3f,  //period
		5,      //kp
		10,		//ki
		0,		//kd
		0,      //feedforward
		0.2,    //lim_iterm
		0,      //lim_dterm
		-1,     //min output
		1	    //max_output
	};
	// Speed
	config.low_level_config_static.speed_pid_config = {
		1e-3f,  //period
		1,      //kp
		0,		//ki
		0,		//kd
		0.64f,      //feedforward
		0.2,    //lim_iterm
		0,      //lim_dterm
		-1,     //min output
		1	    //max_output
	};
	config.low_level_config_cruise.speed_pid_config = {
		1e-3f,  //period
		1,      //kp
		0,		//ki
		0,		//kd
		0.64f,      //feedforward
		0.2,    //lim_iterm
		0,      //lim_dterm
		-1,     //min output
		1	    //max_output
	};
config.low_level_config_rotate.speed_pid_config = {
		1e-3f,  //period
		1,      //kp
		0,		//ki
		0,		//kd
		0.64f,      //feedforward
		0.2,    //lim_iterm
		0,      //lim_dterm
		-1,     //min output
		1	    //max_output
	};
	// Yaw
	config.low_level_config_static.yaw_pid_config = {
			1e-3f,  //period
			5,      //kp
			20,		//ki
			0,		//kd
			0,      //feedforward
			0.2,    //lim_iterm
			0,      //lim_dterm
			-1,     //min output
			1	    //max_output
		};
	config.low_level_config_static.yaw_pid_config = {
			1e-3f,  //period
			5,      //kp
			20,		//ki
			0,		//kd
			0,      //feedforward
			0.2,    //lim_iterm
			0,      //lim_dterm
			-1,     //min output
			1	    //max_output
		};
	config.low_level_config_rotate.yaw_pid_config = {
			1e-3f,  //period
			5,      //kp
			20,		//ki
			0,		//kd
			0,      //feedforward
			0.2,    //lim_iterm
			0,      //lim_dterm
			-1,     //min output
			1	    //max_output
		};
	// Yaw rate
	config.low_level_config_static.yaw_rate_pid_config = {
			1e-3f,  //period
			0.2,      //kp
			0,		//ki
			0,		//kd
			0.12f,      //feedforward
			0.2,    //lim_iterm
			0,      //lim_dterm
			-1,     //min output
			1	    //max_output
		};
	config.low_level_config_cruise.yaw_rate_pid_config = {
			1e-3f,  //period
			0.2,      //kp
			0,		//ki
			0,		//kd
			0.12f,      //feedforward
			0.2,    //lim_iterm
			0,      //lim_dterm
			-1,     //min output
			1	    //max_output
		};
	config.low_level_config_rotate.yaw_rate_pid_config = {
			1e-3f,  //period
			0.2,      //kp
			0,		//ki
			0,		//kd
			0.12f,      //feedforward
			0.2,    //lim_iterm
			0,      //lim_dterm
			-1,     //min output
			1	    //max_output
		};

	config.lookahead_distance = 0.15f;
	config.lookahead_time = 0;
	config.static_pwm_limit = 0.3;
	config.cruise_pwm_limit = 1;
	config.reposition_pwm_limit = 0.25;

	return config;
}
OdometryConfig Robot::odometryConfig()
{
	return m_odometry_config;
}

const RobotConfig& Robot::robotConfig() const
{
	return m_robot_config;
}

void Robot::setOdometryConfig(const OdometryConfig& config)
{
	m_odometry_config = config;
	odometry().setConfig(config);
}

PropulsionController::State Robot::propulsionState()
{
	return m_propulsion_task.controller().state();
}
