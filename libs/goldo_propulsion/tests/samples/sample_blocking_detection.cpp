#include "goldobot/propulsion/speed_controller.hpp"

#include <chrono>
#include <fstream>


int main(int argc, char* argv[]) {
  
  goldobot::SpeedController speed_controller;

  double target{0};
  double dt = 1e-3;
  double t = 0;

  const int num_steps = 10000;

  //speed_controller.setPeriod(1e-3f);
  speed_controller.reset(0, 0, 0);
  speed_controller.setParameterRange(0, 2, true);
  speed_controller.setRequestedSpeed(0.5);

  std::ofstream out_file("D:/cdr2020/sample_propulsion_controller.csv");
  out_file << "t,p,s,a\n";

  for (int i = 0; i < num_steps; i++) {
    speed_controller.update();
    t += dt;
    out_file << t << "," << speed_controller.parameter() << "," << speed_controller.speed()
             << "," << speed_controller.acceleration() <<"\n";
    if (i == 1000) {
      speed_controller.setRequestedSpeed(2);
    }
    if (i == 1500) {
      speed_controller.setAccelerationLimits(1, 4);
      speed_controller.setRequestedSpeed(0);
    }
    
  }
}
