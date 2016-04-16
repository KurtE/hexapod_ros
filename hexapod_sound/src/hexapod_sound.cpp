
// ROS Hexapod Sound Node
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


#include <hexapod_sound.h>
#include <ros/package.h>
#define MPLAYER_HACK   // I have problems with ROS Sound on ODroid - hack to get somethign working...


//==============================================================================
// Constructor
//==============================================================================

HexapodSound::HexapodSound( void )
{
#ifndef MPLAYER_HACK
    sound_pub_ = nh_.advertise<sound_play::SoundRequest>("/robotsound", 1, 0);
#endif
    sounds_sub_ = nh_.subscribe<std_msgs::Int32>( "/sounds", 1, &HexapodSound::soundsCallback, this);
    sound_package_path_ = ros::package::getPath("hexapod_sound");

    if (ros::param::get( "SOUNDS", SOUNDS ) )
    {
        // We have a sound section so lets load from there
        for( XmlRpc::XmlRpcValue::iterator it = SOUNDS.begin(); it != SOUNDS.end(); it++ )
        {
            sound_map_key_.push_back( it->first );
        }
        // resize our sound items... 
        int sound_count = sound_map_key_.size();
        sound_id_.resize(sound_count);
        sound_type_.resize(sound_count);
        sound_delay_time_.resize(sound_count);
        sound_file_name_.resize(sound_count);

        for( int i = 0; i < sound_count; i++ )
        {
            ros::param::get( "SOUNDS/" + static_cast<std::string>( sound_map_key_[i] ) + "/id", sound_id_[i] );
            ros::param::get( "SOUNDS/" + static_cast<std::string>( sound_map_key_[i] ) + "/type", sound_type_[i] );
            ros::param::get( "SOUNDS/" + static_cast<std::string>( sound_map_key_[i] ) + "/delay", sound_delay_time_[i] );
            ros::param::get( "SOUNDS/" + static_cast<std::string>( sound_map_key_[i] ) + "/file", sound_file_name_[i] );
        }

    }
    else
    {
        // sound section not found default values to default project
        // first pass have main function define the sound file mappings.
        for (int i = HexapodSounds::STARTUP; i <= HexapodSounds::AUTO_LEVEL; i++)
        {
            sound_id_.push_back(i);
            sound_type_.push_back(HexapodSounds::SOUND_FILE);  
            sound_delay_time_.push_back(i==HexapodSounds::AUTO_LEVEL? 6 : 3);
        }

        sound_file_name_.push_back("intelChime.ogg");
        sound_file_name_.push_back("activeAwaitingCommands.ogg");
        sound_file_name_.push_back("standingUp.ogg");
        sound_file_name_.push_back("shuttingDown.ogg");
        sound_file_name_.push_back("autoLevelingBody.ogg");
    }

}

void HexapodSound::soundsCallback( const std_msgs::Int32ConstPtr &sound_msg )
{
    playSound(sound_msg->data);
}


void HexapodSound::playSoundFile(std::string sound_file, int delay_time)
{
 #ifndef MPLAYER_HACK
    sound_req_.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_req_.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_req_.arg = sound_package_path_ + "/sounds/" + sound_file; // need to due this due to bug in sound_play
    sound_pub_.publish( sound_req_ );
#else
    std::string command_line = "mplayer " + sound_package_path_ + "/sounds/" + sound_file;
    int ret __attribute__((unused));
    ret = std::system(command_line.c_str());
#endif
    ros::Duration( delay_time ).sleep();
}


void HexapodSound::playSound(int sound_index)
{
    if (sound_index < sound_file_name_.size())
    {
        playSoundFile(sound_file_name_[sound_index], sound_delay_time_[sound_index]);
    }
    else
    {
        ROS_WARN("Sound index %d out of range", sound_index);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "hexapod_sound");
    HexapodSound hexapodSound;



    // Now Play the init sounds...
    hexapodSound.playSoundFile( "empty.ogg", 3 );

    hexapodSound.playSound( HexapodSounds::STARTUP);
    hexapodSound.playSound( HexapodSounds::WAITING);

    
    // Then let ROS handle the thread for us... 
    ros::spin();
}
