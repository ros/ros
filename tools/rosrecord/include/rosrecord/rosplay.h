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

#ifndef ROSPLAY_H
#define ROSPLAY_H

#include <time.h>
#include <sys/stat.h>
#include "ros/node.h"
#include "ros/time.h"
#include <string>

#include "rosrecord/Player.h"
#include "rosrecord/AnyMsg.h"
#include "rosrecord/time_publisher.h"

#include <unistd.h>
#include <termios.h>


class RosPlay
{
public:
  RosPlay(int i_argc, char **i_argv);
  ~RosPlay();
  bool spin();

private:
  ros::Time getSysTime();
  void doPublish(std::string name, ros::Message* m, ros::Time play_time, ros::Time record_time, void* n);

  ros::NodeHandle node_handle;

  bool bag_time_initialized_, at_once_, quiet_, paused_, shifted_, bag_time_;
  double time_scale_;
  ros::Time start_time_, requested_start_time_, paused_time_;
  ros::record::MultiPlayer player_;

  int queue_size_;
  unsigned int advertise_sleep_;

  termios orig_flags_;
  TimePublisher bag_time_publisher_;
  double bag_time_frequency_;

  fd_set stdin_fdset_;
  int maxfd_;

};


#endif
