#pragma once
#include <cstdint>

namespace goldobot
{
    struct OdometryConfig
    {
    	//! \brief length in meters of one encoder tick for left wheel.
        float dist_per_count_left;
        //! \brief length in meters of one encoder tick for right wheel.
        float dist_per_count_right;
        //! \brief wheels spacing in meters.
        float wheel_spacing;
        //! \brief time between odometry updates, in seconds.
        float update_period;
        float speed_filter_period;
        //! \brief period of the encoders counter.
        uint16_t encoder_period;
    };
}
