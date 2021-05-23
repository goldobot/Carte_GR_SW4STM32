#pragma once
#include "goldobot/core/geometry.hpp"

#include <cstdint>

namespace goldobot
{

	class SplineTrajectory
	{
	public:
          SplineTrajectory();

		void clear();
		void setPoints(Vector2D* points,float* knots, unsigned num_points);
		void setPoints(Vector2D* points, unsigned num_points);

		float minParameter() const;
		float maxParameter() const;

		//! \brief compute position of point on current segment
		TrajectoryPoint computePoint(float parameter) const;


	private:
		Vector2D mControlPoints[16];
		float mKnots[20];
		int mNumPoints;
	};
}
