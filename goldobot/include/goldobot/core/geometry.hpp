#pragma once

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
    };
}

