
// ROS Hexapod Controller Node
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


#include <servo_driver.h>

//==============================================================================
//  Constructor: Open USB2AX and get parameters
// If servos are not on, no worries we read them later just to be safe
//==============================================================================

ServoDriver::ServoDriver( void )
{
    // Initialize the USB2AX
    int baud= 1000000L;
    std::string servo_controller_device_name;

    if (!ros::param::get( "SERVO_CONTROLLER_ID", SERVO_CONTROLLER_ID ))
        SERVO_CONTROLLER_ID = 0xfd; // default assumes USB2AX

    ros::param::get("SERVO_CONTROLLER_DEVICE_NAME", servo_controller_device_name); 

    if (!ros::param::get("SERVO_CONTROLLER_BAUD", baud))
        baud = 1000000L;

    if (!ros::param::get("SERVO_CONTROLLER_ENABLE_SERVOS_REGISTER", ENABLE_SERVOS_REGISTER))
        ENABLE_SERVOS_REGISTER = -1;

    ROS_INFO("ServoDriver: Device: %s, Baud: %d", servo_controller_device_name.c_str(), baud);

    if( dxl_initialize( servo_controller_device_name.c_str(), baud ) == 0 )
    {
        ROS_WARN("Servo Communication Failed! Ignore if just running for Rviz or Gazebo.");
    }

    // Stating servos do not have torque applied
    servos_free_ = true;
    ros::param::get( "TORQUE_ENABLE", TORQUE_ENABLE );
    ros::param::get( "PRESENT_POSITION_L", PRESENT_POSITION_L );
    ros::param::get( "GOAL_POSITION_L", GOAL_POSITION_L );
    ros::param::get( "SERVOS", SERVOS );
    ros::param::get( "INTERPOLATION_LOOP_RATE", INTERPOLATION_LOOP_RATE );

    // See which Servo controller we are using as some of them
    // need us to do something to turn on the power to the servos
    int  controller_model_number = dxl_read_word(SERVO_CONTROLLER_ID, 0 );
    if (dxl_get_result() != COMM_RXSUCCESS)
    {
        ROS_WARN("Servo Controller %x Not Found! Ignore if just running for Rviz or Gazebo.", SERVO_CONTROLLER_ID);
    }

    // Do we need to turn power on to the servos? 
    if (ENABLE_SERVOS_REGISTER != -1)
    {
        dxl_write_byte(SERVO_CONTROLLER_ID, ENABLE_SERVOS_REGISTER, 1);
        ROS_INFO("Hexapod power to servos on.");
    }

    for( XmlRpc::XmlRpcValue::iterator it = SERVOS.begin(); it != SERVOS.end(); it++ )
    {
        servo_map_key_.push_back( it->first );
    }

    SERVO_COUNT = servo_map_key_.size();
    OFFSET.resize( SERVO_COUNT );
    ID.resize( SERVO_COUNT );
    TICKS.resize( SERVO_COUNT );
    CENTER.resize( SERVO_COUNT );
    MAX_RADIANS.resize( SERVO_COUNT );
    RAD_TO_SERVO_RESOLUTION.resize( SERVO_COUNT );
    servo_orientation_.resize( SERVO_COUNT );
    cur_pos_.resize( SERVO_COUNT );
    goal_pos_.resize( SERVO_COUNT );
    write_pos_.resize( SERVO_COUNT );
    pose_steps_.resize( SERVO_COUNT );
    for( int i = 0; i < SERVO_COUNT; i++ )
    {
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/offset", OFFSET[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/id", ID[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/ticks", TICKS[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/center", CENTER[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/max_radians", MAX_RADIANS[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/sign", servo_orientation_[i] );
        RAD_TO_SERVO_RESOLUTION[i] = TICKS[i] / MAX_RADIANS[i];
        // Fill vector containers with default value
        cur_pos_[i] = CENTER[i];
        goal_pos_[i] = CENTER[i];
        write_pos_[i] = CENTER[i];
        pose_steps_[i] = 1;
    }

    // Kurt's Debug
    servo_debug_sub_ =  nh_.subscribe<std_msgs::Bool>( "servo_debug", 1, &ServoDriver::servoDebugCallback, this );

}

//==============================================================================
// Destructor: Turn off the torque of the servos then close the serial port
//==============================================================================

ServoDriver::~ServoDriver( void )
{
    freeServos();

    // If the controller enables power to servos, turn it off now
    if (ENABLE_SERVOS_REGISTER != -1)
    {
        dxl_write_byte(SERVO_CONTROLLER_ID, ENABLE_SERVOS_REGISTER, 0);
        ROS_INFO("Hexapod power to servos off.");
    }
    dxl_terminate();
}

//==============================================================================
// Debug Message callback - Currently uses rosmsg to turn debug printing on/off
//==============================================================================

void ServoDriver::servoDebugCallback( const std_msgs::BoolConstPtr &msg )
{
    servo_debug_ = msg->data ? true : false;
    if (servo_debug_)
    {
        ROS_INFO("Servo Debug support enabled");
        count_calls_ = 0;
        count_dxl_packets_ = 0;
        sum_servo_infos_ = 0;
        debug_start_time_ = ros::Time::now();;

    }
    else 
    {
        ROS_INFO("Servo Debug support disabled");
    }
}


//==============================================================================
// Convert angles to servo resolution each leg and head pan
//==============================================================================

void ServoDriver::convertAngles( const sensor_msgs::JointState &joint_state )
{
    for( int i = 0; i < SERVO_COUNT; i++ )
    {
        goal_pos_[i] = CENTER[i] + round( ( joint_state.position[i] - ( servo_orientation_[i] * OFFSET[i] ) ) * RAD_TO_SERVO_RESOLUTION[i] );
    }
}

//==============================================================================
// Turn torque on and read current positions
//==============================================================================

void ServoDriver::makeSureServosAreOn( const sensor_msgs::JointState &joint_state )
{
    if( !servos_free_ )
    {
        // Servos are on so return
        return;
    }
    else
    {
        // Turn torque on
        dxl_write_word( 254, TORQUE_ENABLE, 1 );
        servos_free_ = false;
        ros::Duration( 0.5 ).sleep();

        // Initialize current position as cur since values would be 0 for all servos ( Possibly servos are off till now )
        for( int i = 0; i < SERVO_COUNT; i++ )
        {
            // dxl_write_word( ID[i], 5, 0 ); // Set return delay time to zero ( 1 usec delay actually ) EPROM register
            cur_pos_[i] = dxl_read_word( ID[i], PRESENT_POSITION_L );
        }
    }
}

//==============================================================================
// Updates the positions of the servos and sends USB2AX broadcast packet
//==============================================================================

void ServoDriver::transmitServoPositions( const sensor_msgs::JointState &joint_state, double velocity_division )
{
    convertAngles( joint_state ); // Convert angles to servo resolution

    makeSureServosAreOn( joint_state );

    count_calls_++; //KJE

#define MAX_POSE_STEPS 2   // Will differ for AX/MX...
    int interpolating = 0;

    int iteration_count = (int)((double)INTERPOLATION_LOOP_RATE * velocity_division);
    if (iteration_count < 1)
        iteration_count = 1;    // make sure we won't divide by zero.

    int max_servo_delta;
    // Calculate the delta pre iteration for each servo
    // Make sure we don't allow the servos to move faster than some
    // maximum...   Probably should be parameter...
    for(;;) 
    {
        max_servo_delta = 0;
        for( int i = 0; i < SERVO_COUNT; i++ )
        {
            // If any of these differ we need to indicate a new packet needs to be sent
            if( cur_pos_[i] != goal_pos_[i] )
                interpolating++;    // don't really need, but special case nothing has moved, return...
            int servo_delta = goal_pos_[i] - cur_pos_[i]; 
            write_pos_[i] = cur_pos_[i];
            pose_steps_[i] = ((double)(servo_delta)) / iteration_count;
            if (abs(servo_delta) > max_servo_delta)
                max_servo_delta = abs(max_servo_delta);
        }
        if (max_servo_delta/iteration_count > MAX_POSE_STEPS)
            iteration_count = (max_servo_delta+MAX_POSE_STEPS+1)/MAX_POSE_STEPS;
        else
            break;
    } 

    ros::Rate loop_rate( INTERPOLATION_LOOP_RATE ); // 900 Hz loop
    // If nothing moved we abort no need to send packet with same positions
    if( interpolating != 0 )
    {
        while( iteration_count-- )
        {
            // Prepare packet for broadcast
            dxl_set_txpacket_id( 254 );
            dxl_set_txpacket_instruction( 131 );
            dxl_set_txpacket_length( 3 * SERVO_COUNT + 4 );
            dxl_set_txpacket_parameter( 0, GOAL_POSITION_L );
            dxl_set_txpacket_parameter( 1, 2 );
            int dxl_parameter_index_out = 2;  // index into output parameters
            for( int i = 0; i < SERVO_COUNT; i++ )
            {
                int write_pos_int;  // integer value
                if (  pose_steps_[i] )
                {
                    // Still moving.
                    int write_pos_prev = (int)write_pos_[i];    // remember the last value we wrote out 
                    write_pos_[i] += pose_steps_[i];             // increment 
                    if ( iteration_count )
                        write_pos_int = (int)write_pos_[i];     // convert to integer
                    else
                        write_pos_int = goal_pos_[i];           // last iteration make sure we get to goal

                    if ( pose_steps_[i] > 0 ) 
                    {
                        if ( write_pos_int > goal_pos_[i] )
                        {
                            write_pos_int = goal_pos_[i];
                            pose_steps_[i] = 0;
                        }
                    }
                    
                    else if ( write_pos_int < goal_pos_[i] )
                    {
                        write_pos_int = goal_pos_[i];
                        pose_steps_[i] = 0;
                    }
                    
                    if ( write_pos_int != write_pos_prev )
                    {
                        sum_servo_infos_++ ;
                        // This servo actually is updating in this iteration
                        // Complete sync_write packet for broadcast
                        dxl_set_txpacket_parameter( dxl_parameter_index_out++, ID[i] );
                        dxl_set_txpacket_parameter( dxl_parameter_index_out++, dxl_get_lowbyte( write_pos_int ) );
                        dxl_set_txpacket_parameter( dxl_parameter_index_out++, dxl_get_highbyte( write_pos_int ) );
                    }
                }
            }
            // Now see if we actually updated any servos in this iteration. 
            if (dxl_parameter_index_out > 2)
            {
                dxl_set_txpacket_length( dxl_parameter_index_out + 2 );
                dxl_txrx_packet();
                count_dxl_packets_++;
            }
            loop_rate.sleep();
        }
        // Store write pose as current pose (goal) since we are now done
        for( int i = 0; i < SERVO_COUNT; i++ )
        {
            cur_pos_[i] = write_pos_[i];
        }
    }
    loop_rate.sleep();

    if ( servo_debug_ && (count_calls_ >= 1000) )
    {
        ros::Duration delta_t = ros::Time::now() - debug_start_time_;
        ROS_INFO("Servo Counts: %d %d %d %f", count_calls_, count_dxl_packets_, sum_servo_infos_, delta_t.toSec());
        count_calls_ = 0;
        count_dxl_packets_ = 0;
        sum_servo_infos_ = 0;
        debug_start_time_ = ros::Time::now();;
    }
}

//==============================================================================
// Turn torque off to all servos
//==============================================================================

void ServoDriver::freeServos( void )
{
    // Turn off torque
    dxl_write_word( 254, TORQUE_ENABLE, 0 );
    servos_free_ = true;
}

