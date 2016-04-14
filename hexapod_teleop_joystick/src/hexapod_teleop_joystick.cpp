
// ROS Hexapod Teleop Joystick Node
// Copyright (c) 2016, Kevin M. Ochs
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the Kevin Ochs nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL KEVIN OCHS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: Kevin M. Ochs


#include <hexapod_teleop_joystick.h>
#include <cmath>

//==============================================================================
// Constructor
//==============================================================================

HexapodTeleopJoystick::HexapodTeleopJoystick( void )
{
    state_.data = false;
    imu_override_.data = false;
    debug_servos_.data = false; 
    int max_button;
    NON_TELEOP = false; // Static but here for a safety precaution
    ros::param::get( "STANDUP_BUTTON", STANDUP_BUTTON );
    ros::param::get( "SITDOWN_BUTTON", SITDOWN_BUTTON );
    ros::param::get( "BODY_ROTATION_BUTTON", BODY_ROTATION_BUTTON );
    ros::param::get( "SPEED_UP_BUTTON", SPEED_UP_BUTTON );
    ros::param::get( "SLOW_DOWN_BUTTON", SLOW_DOWN_BUTTON );
    ros::param::get( "DEBUG_BUTTON", DEBUG_BUTTON );
    ros::param::get( "FORWARD_BACKWARD_AXES", FORWARD_BACKWARD_AXES );
    if (!ros::param::get("MAX_JOYSTICK_BUTTON", max_button))
        max_button = 15;        // some hopefully large size...
    ros::param::get( "LEFT_RIGHT_AXES", LEFT_RIGHT_AXES );
    ros::param::get( "YAW_ROTATION_AXES", YAW_ROTATION_AXES );
    ros::param::get( "PITCH_ROTATION_AXES", PITCH_ROTATION_AXES );
    ros::param::get( "MAX_METERS_PER_SEC", MAX_METERS_PER_SEC );
    ros::param::get( "MAX_RADIANS_PER_SEC", MAX_RADIANS_PER_SEC );
    ros::param::get( "NON_TELEOP", NON_TELEOP );

    ros::param::get( "VELOCITY_DIVISION", VELOCITY_DIVISION );
    if (!ros::param::get( "MAX_VELOCITY_DIVISION", MAX_VELOCITY_DIVISION ))
        MAX_VELOCITY_DIVISION = VELOCITY_DIVISION;

    if (!ros::param::get( "MIN_VELOCITY_DIVISION", MIN_VELOCITY_DIVISION ))
        MIN_VELOCITY_DIVISION = VELOCITY_DIVISION;

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 5, &HexapodTeleopJoystick::joyCallback, this);
    body_scalar_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("/body_scalar", 100);
    head_scalar_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("/head_scalar", 100);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    state_pub_ = nh_.advertise<std_msgs::Bool>("/state", 100);
    imu_override_pub_ = nh_.advertise<std_msgs::Bool>("/imu/imu_override", 100);
    servo_debug_pub_ = nh_.advertise<std_msgs::Bool>("/servo_debug", 1);
    velocity_division_pub_ = nh_.advertise<std_msgs::Float64>( "/velocity_division", 1 );

    buttons_prev_.assign(max_button, 0);  // zero this one out... 
    velocity_division_.data = VELOCITY_DIVISION;    

    if (std::abs(MAX_VELOCITY_DIVISION-MIN_VELOCITY_DIVISION) < 0.0001)
    {
        // Min/MAX not defined... so turn off changes...
        cmd_vel_speed_scaler_ = 1.0;
        SPEED_UP_BUTTON = max_button;   // out of range;
        SLOW_DOWN_BUTTON = max_button;  // 
    }
    else
    {
        // Setup scaler for cmd_vel values for the different speeds. 
        cmd_vel_speed_scaler_ = ( MAX_VELOCITY_DIVISION - velocity_division_.data ) / ( MAX_VELOCITY_DIVISION - MIN_VELOCITY_DIVISION );
    }
}

//==============================================================================
// Joystick call reading joystick topics
//==============================================================================

void HexapodTeleopJoystick::joyCallback( const sensor_msgs::Joy::ConstPtr &joy )
{
    ros::Time current_time = ros::Time::now();
    if ( buttonPressed( joy, STANDUP_BUTTON ) )
    {
        if ( state_.data == false)
        {
            state_.data = true;
        }
    }

    if (buttonPressed( joy, SITDOWN_BUTTON ))
    {
        if ( state_.data == true)
        {
            state_.data = false;
        }
    }

    // Lets process Debug button
    if (buttonPressed( joy, DEBUG_BUTTON ))
    {
        debug_servos_.data = (debug_servos_.data)? false : true;
        servo_debug_pub_.publish( debug_servos_ );
    }

    // likewise the speed up/slow down buttons. 
    if (buttonPressed( joy, SPEED_UP_BUTTON ))
    {
        // Don't do anything if we are already at minimum division
        // probably would be better to setup a table of divisions...
        if (std::abs(velocity_division_.data - MIN_VELOCITY_DIVISION) > 0.0001)
        {
            // lets try speeding up by 
            if (std::abs(velocity_division_.data-VELOCITY_DIVISION) < 0.0001)
            {
                velocity_division_.data = (VELOCITY_DIVISION + MIN_VELOCITY_DIVISION) / 2;
            }
            else if (std::abs(velocity_division_.data-MAX_VELOCITY_DIVISION) < 0.0001)
            {
                velocity_division_.data = (VELOCITY_DIVISION + MAX_VELOCITY_DIVISION) / 2;
            }
            else if (velocity_division_.data < VELOCITY_DIVISION)
            {
                velocity_division_.data = MIN_VELOCITY_DIVISION;
            }
            else
            {
                velocity_division_.data = VELOCITY_DIVISION;
            }
            ROS_INFO("Speed up Velocity Division: %f", (double)velocity_division_.data);
            velocity_division_pub_.publish( velocity_division_ );
            cmd_vel_speed_scaler_ = ( MAX_VELOCITY_DIVISION - velocity_division_.data ) / ( MAX_VELOCITY_DIVISION - MIN_VELOCITY_DIVISION );
        }
    }
    if (buttonPressed( joy, SLOW_DOWN_BUTTON ))
    {
        // Don't do anything if we are already at minimum division
        // probably would be better to setup a table of divisions...
        if (std::abs(velocity_division_.data - MAX_VELOCITY_DIVISION) > 0.0001)
        {
            // lets try speeding up by 
            if (std::abs(velocity_division_.data-VELOCITY_DIVISION) < 0.0001)
            {
                velocity_division_.data = (VELOCITY_DIVISION + MAX_VELOCITY_DIVISION) / 2;
            }
            else if (std::abs(velocity_division_.data-MIN_VELOCITY_DIVISION) < 0.0001)
            {
                velocity_division_.data = (VELOCITY_DIVISION + MIN_VELOCITY_DIVISION) / 2;
            }
            else if (velocity_division_.data < VELOCITY_DIVISION)
            {
                velocity_division_.data = VELOCITY_DIVISION;
            }
            else
            {
                velocity_division_.data = MAX_VELOCITY_DIVISION;
            }
            ROS_INFO("Slow down Velocity Division: %f", (double)velocity_division_.data);
            velocity_division_pub_.publish( velocity_division_ );
            cmd_vel_speed_scaler_ = ( MAX_VELOCITY_DIVISION - velocity_division_.data ) / ( MAX_VELOCITY_DIVISION - MIN_VELOCITY_DIVISION );
        }
    }


    // Body rotation L1 Button for testing
    if ( buttonDown( joy, BODY_ROTATION_BUTTON ) )
    {
        imu_override_.data = true;
        body_scalar_.header.stamp = current_time;
        body_scalar_.accel.angular.x = -joy->axes[LEFT_RIGHT_AXES];
        body_scalar_.accel.angular.y = -joy->axes[FORWARD_BACKWARD_AXES];
        head_scalar_.header.stamp = current_time;
        head_scalar_.accel.angular.z = joy->axes[YAW_ROTATION_AXES];
        head_scalar_.accel.angular.y = joy->axes[PITCH_ROTATION_AXES];
    }
    else
    {
        imu_override_.data = false;

        // Travelling
        cmd_vel_.linear.x = joy->axes[FORWARD_BACKWARD_AXES] * MAX_METERS_PER_SEC * cmd_vel_speed_scaler_;
        cmd_vel_.linear.y = -joy->axes[LEFT_RIGHT_AXES] * MAX_METERS_PER_SEC * cmd_vel_speed_scaler_;
        cmd_vel_.angular.z = joy->axes[YAW_ROTATION_AXES] * MAX_RADIANS_PER_SEC * cmd_vel_speed_scaler_;
    }

    // Lets remember the state of the buttons...
    for (int i = 0; i < buttons_prev_.size(); i++)
    {
        buttons_prev_[i] = joy->buttons[i]; 
    }
}


bool HexapodTeleopJoystick::buttonDown(  const sensor_msgs::Joy::ConstPtr &joy, int button_index )
{
    return (joy->buttons[ button_index ] == 1); 
}

bool HexapodTeleopJoystick::buttonPressed ( const sensor_msgs::Joy::ConstPtr &joy, int button_index )
{
    return ((button_index < buttons_prev_.size()) && (joy->buttons[ button_index ] == 1)
            && (buttons_prev_[button_index] == 0)); 
}

bool HexapodTeleopJoystick::buttonReleased ( const sensor_msgs::Joy::ConstPtr &joy, int button_index )
{
    return ((button_index < buttons_prev_.size()) && (joy->buttons[ button_index ] == 0)
            && (buttons_prev_[button_index] == 1)); 
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "hexapod_teleop_joystick");
    HexapodTeleopJoystick hexapodTeleopJoystick;

    ros::AsyncSpinner spinner(1); // Using 1 threads
    spinner.start();

    ros::Rate loop_rate( 100 ); // 100 hz
    while ( ros::ok() )
    {
        if( NON_TELEOP == false ) // If True, assumes you are sending these from other packages
        {
            hexapodTeleopJoystick.cmd_vel_pub_.publish( hexapodTeleopJoystick.cmd_vel_ );
            hexapodTeleopJoystick.body_scalar_pub_.publish( hexapodTeleopJoystick.body_scalar_ );
            hexapodTeleopJoystick.head_scalar_pub_.publish( hexapodTeleopJoystick.head_scalar_ );
        }
        hexapodTeleopJoystick.state_pub_.publish( hexapodTeleopJoystick.state_ ); // Always publish for means of an emergency shutdown type situation
        hexapodTeleopJoystick.imu_override_pub_.publish( hexapodTeleopJoystick.imu_override_ );
        loop_rate.sleep();
    }
}
