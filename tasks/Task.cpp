#include "Task.hpp"

using namespace pancam_360;

Task::Task(std::string const& name):
    TaskBase(name),
    save_frame(false),
    left_frame_saved(false),
    right_frame_saved(false),
    position_index(0),
    set_counter(0)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine):
    TaskBase(name, engine),
    save_frame(false),
    left_frame_saved(false),
    right_frame_saved(false),
    position_index(0),
    set_counter(0)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if(!TaskBase::configureHook())
    {
        return false;
    }
    
    panResolution = _panResolution.get() * 1000;
    tiltResolution = _tiltResolution.get() * 1000;
    
    // To get 90 degrees pan angle: 90/0.051428 = 1750
    // To get 45 degrees pan angle: 45/0.012857 = 3500
    
    // Transform angles (in degrees) to steps for the motor
    position_separation = _positionSeparation.get() / panResolution;
    panorama_pictures = _panoramaPictures.get();
    position_error_margin_pan = _positionErrorMargin.get() / panResolution;
    position_error_margin_tilt = _positionErrorMargin.get() / tiltResolution;
    
    position_order = new double[panorama_pictures];
    
    // Define all the positions
    for(int i = 0; i < panorama_pictures; i++)
    {
        position_order[i] = position_separation * (i - (panorama_pictures - 1) / 2);
    }
    
    // Set the first position
    position_goal = position_order[position_index];
    frame_delay_um.microseconds = _frameDelayTimeMs.get() * 1000LL;
    
    // Must return true to indicate that configuration was successful and the stopped state can be enabled
    return true;
}

bool Task::startHook()
{
    if(!TaskBase::startHook())
    {
        return false;
    }
    
    pan_target_set = false;
    tilt_target_set = false;
    
    // Reset PanCam position index so it would start from the beginning next time
    position_index = 0;
    position_goal = position_order[position_index];
    
    // Return true to call updateHook over and over
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
    
    if(_pan_angle_in.read(pan_angle_in) == RTT::NewData && state() == RUNNING)
    {
        tilt_angle = _positionTilt.get() / tiltResolution;
        // Got new data on the pan position
        // Check if the pan and tilt angles has arrived to requested positions
        if(fabs(pan_angle_in - position_goal) < position_error_margin_pan && fabs(tilt_angle_in - tilt_angle) < position_error_margin_tilt)
        {
            if(!save_frame)
            {
                // The pan and tilt have reached their destination, now the cameras can take a picture set
                goal_arrival_time = base::Time::now();
                save_frame = true;
            }
            else if(left_frame_saved && right_frame_saved)
            {
                // Save the timestamped frame with PTU angles
                pancam_frame.time = left_frame->time;
                pancam_frame.angle_pan_degrees = pan_angle_in * panResolution;
                pancam_frame.angle_tilt_degrees = tilt_angle_in * tiltResolution;
                pancam_frame.left_frame = *left_frame;
                pancam_frame.right_frame = *right_frame;
                pancam_frame.set = set_counter;
                _frame.write(pancam_frame);
                
                // Reset flags
                save_frame = false;
                left_frame_saved = false;
                right_frame_saved = false;
                
                // Final position reached
                if(position_index == panorama_pictures - 1)
                {
                    // Increment the set counter so that the next set would have a different identification number
                    set_counter++;
                    // 360 degree image was taken, stop, reset and set end flag
                    position_index = 0;
                    pan_target_set = false;
                    tilt_target_set = false;
                    
                    // Change the state to STOPPED
                    state(STOPPED);
                    // This must be explicitly called to stop the component at low level or there will be issues in ruby
                    stop();
                }
                else
                {
                    // Pictures have been taken, proceed to the next position
                    // Loop back to 0 instead of going to over panorama_pictures
                    position_index = (position_index + 1) % panorama_pictures;
                    
                    // Set the pointer to the next goal
                    position_goal = position_order[position_index];
                    
                    // Send signal to move to the next position to the PTU
                    _pan_angle_out.write(position_goal);
                }
            }
        }
        else if(!pan_target_set)
        {
            _pan_angle_out.write(position_goal);
            pan_target_set = true;
        }
    }
    
    if(_tilt_angle_in.read(tilt_angle_in) == RTT::NewData && state() == RUNNING)
    {
        tilt_angle = _positionTilt.get() / tiltResolution;
        // Got new data on the tilt position
        if(fabs(tilt_angle_in - tilt_angle) > position_error_margin_tilt && !tilt_target_set)
        {
            // Send the signal to the PTU until it reaches the requested tilt position
            _tilt_angle_out.write(tilt_angle);
            tilt_target_set = true;
        }
    }
    
    if(_left_frame_in.read(left_frame) == RTT::NewData && state() == RUNNING && save_frame)
    {
        if(left_frame->time > goal_arrival_time + frame_delay_um)
        {
            _left_frame_out.write(left_frame);
            left_frame_saved = true;
        }
    }
    
    if(_right_frame_in.read(right_frame) == RTT::NewData && state() == RUNNING && save_frame)
    {
        // Frames always come in pairs, no frame synchronisation is required
        if(right_frame->time > goal_arrival_time + frame_delay_um)
        {
            _right_frame_out.write(right_frame);
            right_frame_saved = true;
        }
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

