#include "goldobot/robot.hpp"

#include "goldobot/utils/crc.hpp"

#include <cassert>
#include <cstring>

using namespace goldobot;

Robot Robot::s_instance;
unsigned char Robot::s_config_area[16384];

Robot& Robot::instance() { return s_instance; }

void Robot::init() {
  m_comm_task.init();
  m_main_task.init();
  m_heartbeat_task.init();
}

void Robot::start() { m_propulsion_task.start(); }

SimpleOdometry& Robot::odometry() { return m_propulsion_task.odometry(); }

ServosConfig* Robot::servosConfig() { return m_servos_config; }

const RobotGeometryConfig& Robot::robotGeometry() const
{
	return *m_robot_geometry_config;
}

PropulsionController::State Robot::propulsionState() {
  return m_propulsion_task.controller().state();
}

void Robot::beginLoadConfig() {
  m_load_config_ptr = s_config_area;
  m_load_config_crc = 0;
}

void Robot::loadConfig(char* buffer, size_t size) {
  if (m_load_config_ptr == nullptr ||
      m_load_config_ptr + size >= s_config_area + sizeof(s_config_area)) {
    return;
  }
  std::memcpy(m_load_config_ptr, buffer, size);
  m_load_config_ptr += size;
  m_load_config_crc = update_crc16((unsigned char*)buffer, size, m_load_config_crc);
}

bool Robot::endLoadConfig(uint16_t crc) {
  // Config binary starts with array of 16 bits offsets to:
  // HalConfig
  // RobotConfig
  // RobotSimulatorConfig
  // OdometryConfig
  // PropulsionControllerConfig
  // ArmConfig
  // Arm positions
  // ServosConfig

  // change to config format:

  if (crc != m_load_config_crc) {
    return false;
  }

  // Bifield representing the tasks to start
  uint32_t tasks_enable{0};

  uint16_t num_sections = *(uint16_t*)s_config_area;
  uint16_t config_size = static_cast<uint16_t>(m_load_config_ptr - s_config_area);

  for (int section_index = 0; section_index < num_sections; section_index++) {
    ConfigSection section_type = *(ConfigSection*)(s_config_area + 2 + section_index * 4);
    uint16_t section_offset = *(uint16_t*)(s_config_area + 4 + section_index * 4);
    uint16_t section_end = (section_index + 1 == num_sections)
                               ? config_size
                               : *(uint16_t*)(s_config_area + 4 + (section_index + 1) * 4);

    assert(section_end >= section_offset);
    uint16_t section_size = section_end - section_offset;

    switch (section_type) {
      case ConfigSection::Hal:
        hal::configure(s_config_area + section_offset);
        break;
      case ConfigSection::PropulsionTask:
		m_propulsion_task.setTaskConfig(*reinterpret_cast<PropulsionTask::Config*>(s_config_area + section_offset));
		break;
      case ConfigSection::Odometry:
        odometry().setConfig(*reinterpret_cast<OdometryConfig*>(s_config_area + section_offset));
        break;
      case ConfigSection::PropulsionController:
    	m_propulsion_task.setControllerConfig(*reinterpret_cast<PropulsionControllerConfig*>(s_config_area + section_offset));
        break;
      case ConfigSection::RobotSimulator:
      	m_propulsion_task.setRobotSimulatorConfig(*reinterpret_cast<RobotSimulatorConfig*>(s_config_area + section_offset));
      	break;
      case ConfigSection::Servos:
    	  m_servos_config = reinterpret_cast<ServosConfig*>(s_config_area + section_offset);
      	break;
      case ConfigSection::TasksEnable:
         tasks_enable = *reinterpret_cast<uint32_t*>(s_config_area + section_offset);
         break;
      default:
        break;
    }
  }

  // m_robot_config = (RobotConfig*)(s_config_area + offsets[i++]);
  // m_servos_config = (ServosConfig*)(s_config_area + offsets[i++]);
  // m_main_task.sequenceEngine().endLoad();

  // if (m_robot_config->use_odrive_uart) {
  //    m_odrive_comm_task.init();
  //  }

  // m_propulsion_task.init();
  // m_fpga_task.init(256);

  // todo: cleanup
  // task ids:
  // propulsion: 0
  // odrive_comm: 1
  // servos: 2
  // dynamixels_comm: 3
  // fpga: 4

  if((tasks_enable & (1 << 0)) != 0)
  {
	  m_propulsion_task.init();
  }

  if((tasks_enable & (1 << 1)) != 0)
    {
	  m_odrive_comm_task.init();
    }

  if((tasks_enable & (1 << 2)) != 0)
    {
	  m_servos_task.init(256);
    }

  if((tasks_enable & (1 << 3)) != 0)
    {
	  m_dynamixels_comm_task.init(256);
    }
  if((tasks_enable & (1 << 4)) != 0)
     {
 	  m_fpga_task.init(256);
     }

  start();
  m_match_state = MatchState::Idle;
  return true;
}
