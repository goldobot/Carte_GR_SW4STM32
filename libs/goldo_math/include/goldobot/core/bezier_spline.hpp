#pragma once
#include "goldobot/core/geometry.hpp"

#include <cstdint>

namespace goldobot {

class BezierSpline {
 public:
  BezierSpline(float* knots, float* control_points, unsigned num_knots, unsigned dimension,
               unsigned order);
  
  float minParameter() const noexcept;
  float maxParameter() const noexcept;

  //! \brief compute position of point on current segment
   void computePoint(float p, float* positions, float* derivatives, float* second_derivatives) const;

 private:
  float* mControlPoints;
  float* mKnots;
  unsigned mNumKnots;
  unsigned mDimension;
  unsigned mOrder;
};
}  // namespace goldobot
