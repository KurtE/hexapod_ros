
// ROS Hexapod Locomotion Node
// Copyright (c) 2014, Kevin M. Ochs
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the <organization> nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: Kevin M. Ochs


#ifndef IK_H_
#define IK_H_

#include <cmath> // acos atan
#include <ros/ros.h>
#include <hexapod_msgs/BodyJoint.h>
#include <hexapod_msgs/LegsJoints.h>
#include <hexapod_msgs/FeetPositions.h>

//=============================================================================
// Define structs and classes for gait system
//=============================================================================

struct Trig
{
    double sine;
    double cosine;
};

class Ik
{
    public:
        Ik( void );
        void calculateIK( const hexapod_msgs::FeetPositions &feet, const hexapod_msgs::BodyJoint &body, hexapod_msgs::LegsJoints *legs );
    private:
        Trig getSinCos( double angle_rad );
        double BODY_CENTER_TO_COXA_X[6], BODY_CENTER_TO_COXA_Y[6];
        double INIT_COXA_ANGLE[6];
        double INIT_FOOT_POS_X[6], INIT_FOOT_POS_Y[6], INIT_FOOT_POS_Z;
        double COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, TARSUS_LENGTH;
};

#endif // IK_H_

