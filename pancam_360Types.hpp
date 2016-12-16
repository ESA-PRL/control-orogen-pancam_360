#ifndef pancam_360_TYPES_HPP
#define pancam_360_TYPES_HPP

#include <base/Time.hpp>
#include <base/samples/Frame.hpp>

namespace pancam_360 {
    // Timestamped frame with angles and both images
    struct PanCamTimestampedFrame
    {
        // Time at which the frame was taken
        base::Time time;
        
        // Angles converted in degrees
        double angle_tilt_degrees;
        double angle_pan_degrees;
        
        // Image frames
        base::samples::frame::Frame left_frame;
        base::samples::frame::Frame right_frame;
        
        // Set to which the picture belongs to
        int set;
    };
}

#endif

