/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#ifndef ROSBAG_PLAYER_H
#define ROSBAG_PLAYER_H

#include <queue>
#include <sys/stat.h>
#include <string>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Empty.h>
#include <topic_tools/shape_shifter.h>

#include "rosbag/any_msg.h"
#include "rosbag/multi_player.h"
#include "rosbag/player.h"
#include "rosbag/time_publisher.h"

namespace rosbag
{

struct BagContent
{
    BagContent(std::string d, std::string m, std::string def)
		: datatype(d), md5sum(m), definition(def), count(1) { }

    std::string datatype;
    std::string md5sum;
    std::string definition;
    int         count;
};

class Player
{
public:
    Player();
    ~Player();

    void init(int argc, char** argv);

    bool spin();

private:
    ros::Time getSysTime();
    void      doPublish(std::string name, ros::Message* m, ros::Time play_time, ros::Time record_time,   void* n);
    void      checkFile(std::string name, ros::Message* m, ros::Time time_play, ros::Time time_recorded, void* n);

private:
    ros::NodeHandle* node_handle;    //!< pointer to allow player to start before node handle exists since this is where argument parsing happens

    bool   bag_time_initialized_;
    bool   at_once_;
    bool   quiet_;
    bool   paused_;
    bool   shifted_;
    bool   bag_time_;
    double time_scale_;

    ros::Time   start_time_;
    ros::Time   requested_start_time_;
    ros::Time   paused_time_;
    MultiPlayer player_;

    int          queue_size_;
    unsigned int advertise_sleep_;

    termios        orig_flags_;
    TimePublisher* bag_time_publisher_;  //!< internally contains a NodeHandle so we make a pointer for the same reason
    double         bag_time_frequency_;

    fd_set stdin_fdset_;
    int    maxfd_;

    std::map<std::string, BagContent> content_;
    unsigned long long                end_time_;

    std::map<std::string, ros::Publisher> publishers_;
};

}

#endif
