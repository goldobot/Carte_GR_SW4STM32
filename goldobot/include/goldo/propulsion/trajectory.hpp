#pragma once
#include "goldo/core/geometry.hpp"
#include "goldo/core/pid_controller.hpp"

#include <cstdint>

namespace goldobot
{
  class TrajectoryBuffer;

  struct TrajectoryPoint
    {
      Vector2D position;
      Vector2D tangent;
    };


  class TrajectoryBuffer
  {
  public:
    TrajectoryBuffer();

    void clear();
    int num_segments() const;
    bool push_segment(Vector2D* points, unsigned num_points);
    void pop_segment();

    //! \brief compute position of point on current segment
    TrajectoryPoint compute_point(float parameter) const;
    float min_parameter() const;
    float max_parameter() const;

  private:
    // Trajectory control points
    Vector2D m_control_points[32];

    // Curve parameters of control points
    float m_knot_parameters[32];

    uint16_t m_buffer_size;
    uint16_t m_num_segments;

    uint16_t m_current_first_index;
    uint16_t m_current_last_index;
    uint16_t m_last_end_index;
  };
}
