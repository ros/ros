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
#include <gtest/gtest.h>
#include <ros/time.h>
#include <ros/console.h>

int num_messages_received=0;


/**
 * The cpp listener, adapted to test roslisp talker+listener
 */

void chatterCallback(const std_msgs::StringConstPtr& msg)
{
  num_messages_received++;
}


TEST(Roslisp, TalkerListener)
{
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter-echo", 1000, chatterCallback);
  ros::Duration d(1);

  int wait_max=180;

  for (int i=0; i<wait_max && n.ok(); i++) {
    d.sleep();
    ros::spinOnce();
    ROS_INFO_STREAM ("Have waited " << i << " out of " << wait_max << " seconds to receive 5 messages from chatter-echo");
    if (num_messages_received>5)
      break;
  }
  EXPECT_TRUE(num_messages_received>5);
  ROS_INFO_STREAM ("Successfully received " << num_messages_received << " echoed messages");
}




int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "roslisp-tester");
  return RUN_ALL_TESTS();
}
