#include "goldobot/propulsion/controller.hpp"
#include "goldobot/propulsion/robot_simulator.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"

#include <chrono>
#include <fstream>

#include "yaml-cpp/yaml.h"

goldobot::PIDConfig load_pid_config(const YAML::Node& cfg) {
  goldobot::PIDConfig pid_config;
  pid_config.kp = cfg["kp"].as<float>();
  pid_config.ki = cfg["ki"].as<float>();
  pid_config.kd = cfg["kd"].as<float>();
  pid_config.lim_iterm = cfg["lim_i"].as<float>();
  pid_config.lim_dterm = cfg["lim_d"].as<float>();
  pid_config.min_output = cfg["out_min"].as<float>();
  pid_config.max_output = cfg["out_max"].as<float>();
  pid_config.d_filter_frequency = 100;
  return pid_config;
}

int main(int argc, char* argv[]) {
  YAML::Node config = YAML::LoadFile("D:/cdr2020/goldobot_ihm/config/goldorak_2020/nucleo.yaml");

  goldobot::SimpleOdometry odometry;
  goldobot::RobotSimulator robot_simulator;
  goldobot::PropulsionController propulsion_controller(&odometry);

  goldobot::OdometryConfig odometry_config;
  odometry_config.speed_filter_frequency = 100;
  odometry_config.accel_filter_frequency = 50;
  odometry_config.dist_per_count_left = 1.5e-5;
  odometry_config.dist_per_count_right = 1.5e-5;
  odometry_config.wheel_distance_left = 240.0e-3;
  odometry_config.wheel_distance_right = 0;

  goldobot::PropulsionControllerConfig controller_config;
  {
    auto cfg = config["propulsion"];
    controller_config.lookahead_time = cfg["lookahead_time"].as<float>();
    controller_config.lookahead_distance = cfg["lookahead_distance"].as<float>();
    controller_config.static_pwm_limit = cfg["static_pwm_limit"].as<float>();
    controller_config.cruise_pwm_limit = cfg["cruise_pwm_limit"].as<float>();
    controller_config.reposition_pwm_limit = cfg["reposition_pwm_limit"].as<float>();

    auto ll_cfg = cfg["low_level_config"];
    controller_config.low_level_config.wheels_distance = ll_cfg["wheels_distance"].as<float>();
    controller_config.low_level_config.motors_speed_factor =
        ll_cfg["motors_speed_factor"].as<float>();

    auto pid_cfg = cfg["pid_configs"][0];

    controller_config.pid_configs[0].longi_pid_config = load_pid_config(pid_cfg["longi"]);
    controller_config.pid_configs[0].speed_pid_config = load_pid_config(pid_cfg["speed"]);
    controller_config.pid_configs[0].yaw_pid_config = load_pid_config(pid_cfg["yaw"]);
    controller_config.pid_configs[0].yaw_rate_pid_config = load_pid_config(pid_cfg["yaw_rate"]);
  }

  propulsion_controller.setConfig(controller_config);
  controller_config.lookahead_distance = 0.1;

  goldobot::RobotSimulatorConfig robot_simulator_config;
  robot_simulator_config.speed_coeff = 2.0e-2;
  robot_simulator_config.wheels_spacing = 200.0e-3;
  robot_simulator_config.encoders_spacing = 240.0e-3;
  robot_simulator_config.encoders_counts_per_m = 1.0f / odometry_config.dist_per_count_left;
  robot_simulator_config.encoders_period = 8192;

  odometry.setPeriod(1e-3f);
  odometry.setConfig(odometry_config);

  robot_simulator.m_config = robot_simulator_config;

  double target{0};
  double dt = 1e-3;
  double t = 0;

  const int num_steps = 10000;

  std::ofstream out_file("D:/cdr2020/sample_propulsion_controller.csv");
  out_file << "t,x,y,yaw,speed\n";

  propulsion_controller.setEnable(true);
  robot_simulator.m_motors_enable = true;
  propulsion_controller.resetPose(0, 0, 0);
  propulsion_controller.executeTranslation(0.5, 1);

  for (int i = 0; i < num_steps; i++) {
    robot_simulator.doStep();
    odometry.update(robot_simulator.encoderLeft(), robot_simulator.encoderRight());
    propulsion_controller.update();
    robot_simulator.m_left_pwm = propulsion_controller.leftMotorVelocityInput();
    robot_simulator.m_right_pwm = propulsion_controller.rightMotorVelocityInput();
    auto pose = odometry.pose();
    t += dt;
    out_file << t << "," << pose.position.x << "," << pose.position.y << "," << pose.yaw << ",";
    out_file << pose.speed << "\n";
  }
}
