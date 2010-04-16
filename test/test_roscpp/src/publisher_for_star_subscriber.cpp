/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

/*
 * Author: Josh Faust
 */

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <test_roscpp/TestEmpty.h>
#include <test_roscpp/TestArray.h>

ros::Publisher g_pub;
int8_t type = 0;

bool switchPublisherType(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
{
  ros::NodeHandle nh;
  g_pub.shutdown();
  type = (type + 1) % 2;
  switch (type)
  {
  case 0:
    g_pub = nh.advertise<test_roscpp::TestEmpty>("test_star_inter", 0);
    break;
  case 1:
    g_pub = nh.advertise<test_roscpp::TestArray>("test_star_inter", 0);
    break;
  }
  return true;
}

void pubTimer(const ros::TimerEvent&)
{
  g_pub.publish(test_roscpp::TestEmpty());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publisher_for_star_subscriber");
  ros::NodeHandle nh;

  g_pub = nh.advertise<test_roscpp::TestEmpty>("test_star_inter", 0);
  ros::Timer t = nh.createTimer(ros::Duration(0.01), pubTimer);
  ros::ServiceServer s = nh.advertiseService("switch_publisher_type", switchPublisherType);
  ros::spin();
}
