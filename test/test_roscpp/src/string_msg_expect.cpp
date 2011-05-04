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

#include <string>
#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

std::string expect_what, on_topic;
unsigned nseen = 0;

void callback(const std_msgs::String::ConstPtr& msg)
{
  EXPECT_STREQ(expect_what.c_str(), msg->data.c_str());
  nseen++;
}

TEST(Expecter, impl)
{
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe(on_topic, 0, callback);

  ros::Rate loop_rate(10);

  while(ros::ok() && nseen == 0)
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

  ASSERT_GT(nseen, 0);
}

int
main(int argc, char** argv)
{
  expect_what = argv[1];
  on_topic = argv[2];

  std::cout << "Will look for string \"" << expect_what << "\" on topic " << on_topic << "\n";

  testing::InitGoogleTest(&argc, argv);
  ros::init( argc, argv, "params" );
//  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
