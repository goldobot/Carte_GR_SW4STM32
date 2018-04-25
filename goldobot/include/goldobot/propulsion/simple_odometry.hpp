#pragma once
#include <cstdint>
#include "goldobot/core/geometry.hpp"
#include "goldobot/propulsion/odometry_config.hpp"

namespace goldobot
{
    class SimpleOdometry
    {
    public:
    	SimpleOdometry();

		uint16_t leftEncoderValue() const;
		uint16_t rightEncoderValue() const;
		const RobotPose& pose() const;

		void setConfig(const OdometryConfig& config);
		void setPose(const RobotPose& pose);

		void reset(uint16_t left_encoder, uint16_t right_encoder);
		void update(uint16_t left_encoder, uint16_t right_encoder);



        
    private:
		uint16_t m_left_encoder;
		uint16_t m_right_encoder;
        RobotPose m_pose;
        OdometryConfig m_config;

        double m_x;
        double m_y;
        float m_yaw;

        void updatePose(float dx, float dtheta, float dt);
    };
}
