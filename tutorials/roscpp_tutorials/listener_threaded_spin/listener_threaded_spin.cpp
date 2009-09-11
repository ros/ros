/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <boost/thread.hpp>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system, using
 * a threaded Spinner object to receive callbacks in multiple threads at the same time.
 */

ros::Duration d(0.01);

class Listener
{
public:
  void chatter1(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO_STREAM("chatter1: [" << msg->data << "] [thread=" << boost::this_thread::get_id() << "]");
    d.sleep();
  }
  void chatter2(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO_STREAM("chatter1: [" << msg->data << "] [thread=" << boost::this_thread::get_id() << "]");
    d.sleep();
  }
  void chatter3(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO_STREAM("chatter1: [" << msg->data << "] [thread=" << boost::this_thread::get_id() << "]");
    d.sleep();
  }
};

void chatter4(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("chatter1: [" << msg->data << "] [thread=" << boost::this_thread::get_id() << "]");
  d.sleep();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  Listener l;
  ros::Subscriber sub1 = n.subscribe("chatter", 10, &Listener::chatter1, &l);
  ros::Subscriber sub2 = n.subscribe("chatter", 10, &Listener::chatter2, &l);
  ros::Subscriber sub3 = n.subscribe("chatter", 10, &Listener::chatter3, &l);
  ros::Subscriber sub4 = n.subscribe("chatter", 10, chatter4);

  /**
   * The MultiThreadedSpinner object allows you to specify a number of threads to use
   * to call callbacks.  If no explicit # is specified, it will use the # of hardware
   * threads available on your system.  Here we explicitly specify 4 threads.
   */
  ros::MultiThreadedSpinner s(4);
  ros::spin(s);

  return 0;
}

