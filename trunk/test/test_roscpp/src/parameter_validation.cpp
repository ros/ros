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

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <test_roscpp/TestEmpty.h>

#define ASSERT_THROWS(expr) \
  try \
  { \
    expr; \
    FAIL(); \
  } \
  catch (std::exception& e) \
  { \
    ROS_INFO("Caught exception: %s", e.what());\
  }

void callback(const test_roscpp::TestEmptyConstPtr&)
{

}

TEST(ParameterValidation, subscribeEmptyMD5Sum)
{
  ros::NodeHandle nh;
  ros::SubscribeOptions ops;
  ops.init<test_roscpp::TestEmpty>("blah", 0, callback);
  ops.md5sum.clear();
  ASSERT_THROWS(nh.subscribe(ops));
}

TEST(ParameterValidation, subscribeEmptyDataType)
{
  ros::NodeHandle nh;
  ros::SubscribeOptions ops;
  ops.init<test_roscpp::TestEmpty>("blah", 0, callback);
  ops.datatype.clear();
  ASSERT_THROWS(nh.subscribe(ops));
}

TEST(ParameterValidation, subscribeNoCallback)
{
  ros::NodeHandle nh;
  ros::SubscribeOptions ops("blah", 0, "blah", "blah");
  ASSERT_THROWS(nh.subscribe(ops));
}

TEST(ParameterValidation, advertiseEmptyMD5Sum)
{
  ros::NodeHandle nh;
  ros::AdvertiseOptions ops("blah", 0, "", "blah", "blah");
  ASSERT_THROWS(nh.advertise(ops));
}

TEST(ParameterValidation, advertiseEmptyDataType)
{
  ros::NodeHandle nh;
  ros::AdvertiseOptions ops("blah", 0, "blah", "", "blah");
  ASSERT_THROWS(nh.advertise(ops));
}

TEST(ParameterValidation, advertiseStarMD5Sum)
{
  ros::NodeHandle nh;
  ros::AdvertiseOptions ops("blah", 0, "*", "blah", "blah");
  ASSERT_THROWS(nh.advertise(ops));
}

TEST(ParameterValidation, advertiseStarDataType)
{
  ros::NodeHandle nh;
  ros::AdvertiseOptions ops("blah", 0, "blah", "*", "blah");
  ASSERT_THROWS(nh.advertise(ops));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "parameter_validation");

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
