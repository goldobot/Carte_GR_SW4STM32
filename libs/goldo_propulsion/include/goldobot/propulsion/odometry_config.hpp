#pragma once

namespace goldobot {
struct OdometryConfig {
  //! \brief length in meters of one encoder tick for left wheel.
  float dist_per_count_left;
  //! \brief length in meters of one encoder tick for right wheel.
  float dist_per_count_right;
  //! \brief left wheel distance from center in meters.
  float wheel_distance_left;
  //! \brief right wheel distance from center in meters.
  float wheel_distance_right;
  //! \brief speed filter cutoff frequency
  float speed_filter_frequency;
  //! \brief speed filter cutoff frequency
  float accel_filter_frequency;
};
}  // namespace goldobot
