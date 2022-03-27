#include "goldobot/core/trajectory_buffer.hpp"

#include <cmath>

using namespace goldobot;

TrajectoryBuffer::TrajectoryBuffer()
    : m_buffer_size(32), m_num_segments(0), m_current_first_index(0) {}

int TrajectoryBuffer::num_segments() const { return m_num_segments; }

bool TrajectoryBuffer::push_segment(Vector2D* points, unsigned num_points) {
  float parameter = 0;
  Vector2D current_point = points[0];
  for (unsigned i = 0; i < num_points; i++) {
    float dx = points[i].x - current_point.x;
    float dy = points[i].y - current_point.y;
    parameter += sqrtf(dx * dx + dy * dy);
    m_control_points[i] = points[i];
    m_knot_parameters[i] = parameter;
    current_point = points[i];
  }

  m_current_first_index = 0;
  m_current_last_index = num_points - 1;
  return true;
}

void TrajectoryBuffer::pop_segment() {
  if (m_num_segments) {
  }
}

TrajectoryPoint TrajectoryBuffer::compute_point(float parameter) const {
  unsigned i = 0;
  while (i < m_current_last_index && m_knot_parameters[i + 1] < parameter) {
    i++;
  }

  // temp version, droite
  float u = (parameter - m_knot_parameters[i]) / (m_knot_parameters[i + 1] - m_knot_parameters[i]);

  float x = m_control_points[i + 1].x * u + m_control_points[i].x * (1 - u);
  float y = m_control_points[i + 1].y * u + m_control_points[i].y * (1 - u);
  float dx = m_control_points[i + 1].x - m_control_points[i].x;
  float dy = m_control_points[i + 1].y - m_control_points[i].y;
  float d_norm_inv = 1.0 / sqrtf(dx * dx + dy * dy);
  return TrajectoryPoint{{x, y}, {dx * d_norm_inv, dy * d_norm_inv}};
}

float TrajectoryBuffer::min_parameter() const { return m_knot_parameters[m_current_first_index]; }

float TrajectoryBuffer::max_parameter() const { return m_knot_parameters[m_current_last_index]; }

float TrajectoryBuffer::closestParameter(const StaticPose& pose) const {
  // \todo: implement
  return 0;
}
