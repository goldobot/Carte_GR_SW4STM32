#pragma once
#include <algorithm>
#include <cmath>

namespace goldobot {

class Vector2D {
 public:
  Vector2D& operator=(const Vector2D&) = default;
  float x;
  float y;
};

class StaticPose {
 public:
  Vector2D position{0, 0};
  float yaw{0};
};

class RobotPose {
 public:
  Vector2D position{0, 0};
  float yaw{0};
  float speed{0};
  float yaw_rate{0};
  float acceleration{0};
  float angular_acceleration{0};
};
}  // namespace goldobot
