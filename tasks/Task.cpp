#include "Task.hpp"

using namespace pancam_360;

Task::Task(std::string const& name):
    TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine):
    TaskBase(name, engine)
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
    
    save_frame = false;
    left_frame_saved = false;
    right_frame_saved = false;
    position_index = 0;
    set_counter = 0;
    
    position_error_margin = _positionErrorMargin.get() * DEG2RAD;
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
    
    // Load all the positions from the configuration file
    /*
    The .yml configuration file contents must look like the following:
    cameraPositions:
      - data
        - 0.0
        - 0.0
      - data:
        - 30.0
        - 20.0
    First value is the pan position, second is for the tilt, in degrees
    */
    camera_positions = _cameraPositions.get();
    
    // Report an error when the list is empty
    if(camera_positions.empty())
    {
        std::cout << "pancam_360: cameraPositions is empty" << std::endl;
        return false;
    }
    
    // Reset PanCam position index so it would start from the beginning
    position_index = 0;
    
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
    
    // Got new data on the pan position
    if(_pan_angle_in.read(pan_angle_in) == RTT::NewData && state() == RUNNING)
    {
        // Get the current pan and tilt goal positions
        tilt_angle_goal = camera_positions[position_index][TILT] * DEG2RAD;
        pan_angle_goal = camera_positions[position_index][PAN] * DEG2RAD;
        
        // Check if the pan and tilt angles have arrived to requested positions
        if(fabs(pan_angle_in - pan_angle_goal) <= position_error_margin && fabs(tilt_angle_temp - tilt_angle_goal) <= position_error_margin)
        {
            if(!save_frame)
            {
                // The pan and tilt have reached their destination, now the cameras can take a picture set
                goal_arrival_time = base::Time::now();
                save_frame = true;
            }
            else if(left_frame_saved && right_frame_saved)
            {
                // Both cameras have taken a picture, output the pair with position information and set number
                _left_frame_out.write(left_frame);
                _right_frame_out.write(right_frame);
                _pan_angle_out_degrees.write(pan_angle_in / DEG2RAD);
                _tilt_angle_out_degrees.write(tilt_angle_temp / DEG2RAD);
                _set_id.write(set_counter);
                
                // Reset flags
                save_frame = false;
                left_frame_saved = false;
                right_frame_saved = false;
                
                // Final position reached
                if(position_index == camera_positions.size() - 1)
                {
                    // Increment the set counter so that the next set would have a different identification number
                    set_counter++;
                    
                    // 360 degree image was taken, stop, reset and set end flag
                    position_index = 0;
                    
                    // Change the state to STOPPED
                    state(STOPPED);
                    // This must be explicitly called to stop the component at low level or there will be issues in ruby
                    stop();
                }
                else
                {
                    // Picture pair has been taken, proceed to the next position
                    position_index++;
                    
                    // Send signal to move to the next position to the PTU
                    _pan_angle_out.write(camera_positions[position_index][PAN] * DEG2RAD);
                    _tilt_angle_out.write(camera_positions[position_index][TILT] * DEG2RAD * TILT_MULTIPLIER);
                }
            }
        }
        else
        {
            _pan_angle_out.write(camera_positions[position_index][PAN] * DEG2RAD);
        }
    }
    
    if(_tilt_angle_in.read(tilt_angle_in) == RTT::NewData && state() == RUNNING)
    {
        // Get the tilt position current value and goal
        // Tilt position has a multiplier because of the gearing
        tilt_angle_temp = tilt_angle_in / TILT_MULTIPLIER;
        tilt_angle_goal = camera_positions[position_index][TILT] * DEG2RAD;
        
        // Got new data on the tilt position
        if(fabs(tilt_angle_temp - tilt_angle_goal) > position_error_margin)
        {
            // Send the signal to the PTU until it reaches the requested tilt position
            _tilt_angle_out.write(tilt_angle_goal * TILT_MULTIPLIER);
        }
    }
    
    if(_left_frame_in.read(left_frame) == RTT::NewData && state() == RUNNING && save_frame)
    {
        if(left_frame->time > goal_arrival_time + frame_delay_um)
        {
            left_frame_saved = true;
        }
    }
    
    if(_right_frame_in.read(right_frame) == RTT::NewData && state() == RUNNING && save_frame)
    {
        // Frames always come in pairs, no frame synchronisation is required
        if(right_frame->time > goal_arrival_time + frame_delay_um)
        {
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

