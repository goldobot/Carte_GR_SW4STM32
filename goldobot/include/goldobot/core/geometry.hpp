#pragma once

namespace goldobot
{
    class Vector2D
    {
    public:
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
