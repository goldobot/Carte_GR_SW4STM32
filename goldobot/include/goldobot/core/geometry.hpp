#pragma once
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif


namespace goldobot
{
    class Vector2D
    {
    public:
    	Vector2D& operator=(const Vector2D&) = default;
        float x;
        float y;
    };

    class RobotPose
    {
    public:
    	Vector2D position;
    	float yaw;
    	float speed;
    	float yaw_rate;
    	float acceleration;
    	float angular_acceleration;
    };

    inline
    float clampAngle(float a)
    {
    	while(a > M_PI)
    	{
    		a = a - M_PI * 2;
    	}
    	while (a < -M_PI)
    	{
    		a = a + M_PI * 2;
    	}
    	return a;
    }

    inline
    float clamp(float val, float min_val, float max_val)
    {
    	if(val < min_val)
    	{
    		return min_val;
    	} else if(val > max_val)
    	{
    		return max_val;
    	} else
    	{
    		return val;
    	}
    }

    inline
    float angleDiff(float a, float b)
    {
    	float diff = a - b;
    	return clampAngle(diff);
    }
}

