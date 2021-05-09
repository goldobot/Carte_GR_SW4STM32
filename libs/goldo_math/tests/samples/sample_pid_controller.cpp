#include "goldobot/core/pid_controller.hpp"

#include <chrono>
#include <fstream>

int main(int argc, char* argv[]) {
  goldobot::PIDController pid_controller;

  goldobot::PIDConfig config;

  config.kp = 1.0f;
  config.ki = 0.2f;
  config.lim_iterm = 2.0;
  config.min_output = -100.0f;
  config.max_output = 100.0f;

  pid_controller.setPeriod(1e-3f);
  pid_controller.setConfig(config);

  double x{0};
  double v{0};

  double target{0};
  double dt = 1e-3;
  double t = 0;

  const int num_steps = 10000;

  std::ofstream out_file("sample_pid_controller.csv");
  out_file << "t,x,v,target,command\n";

  for (int i = 0; i < num_steps; i++) {
    target = t < 1.0 ? 0 : 1;
    float error = target - x;
    float command = pid_controller.step(error);
    v += (command - v * 1.0 - x * 1.0) * dt;
    x += v * dt;
    t += dt;
    out_file << t << "," << x << "," << v << "," << target << "," << command << "\n";
  }
}
