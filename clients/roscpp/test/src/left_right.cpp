/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

/* Author: Troy Straszheim */

/*
 * Basic publisher of two messages.
 */

#include <string>
#include <sstream>
#include <fstream>

#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

int
main(int argc, char** argv)
{
  ros::init( argc, argv, "left_right");
  ros::NodeHandle nh;

  ros::Publisher left = nh.advertise<std_msgs::String>("left", 0);
  ros::Publisher right = nh.advertise<std_msgs::String>("right", 0);

  std_msgs::String msg_l, msg_r;
  msg_l.data = "left";
  msg_r.data = "right";
  
  ros::Rate loop_rate(2);

  for (unsigned j=0; j<5; ++j)
    {
      assert(ros::ok());

      left.publish(msg_l);
      right.publish(msg_r);

      ROS_INFO("ping!");

      ros::spinOnce();
      loop_rate.sleep();
      ros::spinOnce();
    }


  return 0;
}
