#pragma once
#include "goldo/enums.hpp"
namespace goldobot {

  struct DbgGoldoVec
  {
    unsigned int clock_ms;
    short int x_mm;
    short int y_mm;
    int theta_deg_1000;
    unsigned short left_odo;
    unsigned short right_odo;
  };

  struct DbgGoldoVecSimple
  {
    unsigned int clock_ms;
    unsigned short left_odo;
    unsigned short right_odo;
  };

} /* namespace goldobot */
