/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

/* Author: Josh Faust */

/*
 * Test serialization templates
 */

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <std_msgs/builtin_bool.h>
#include <std_msgs/builtin_double.h>
#include <std_msgs/builtin_float.h>
#include <std_msgs/builtin_int16.h>
#include <std_msgs/builtin_int32.h>
#include <std_msgs/builtin_int64.h>
#include <std_msgs/builtin_int8.h>
#include <std_msgs/builtin_string.h>
#include <std_msgs/builtin_uint16.h>
#include <std_msgs/builtin_uint32.h>
#include <std_msgs/builtin_uint64.h>
#include <std_msgs/builtin_uint8.h>

TEST(BuiltinTypes, advertise)
{
  ros::NodeHandle nh;
  nh.advertise<bool>("test_bool", 1);
  nh.advertise<double>("test_double", 1);
  nh.advertise<float>("test_float", 1);
  nh.advertise<int16_t>("test_int16", 1);
  nh.advertise<int32_t>("test_int32", 1);
  nh.advertise<int64_t>("test_int64", 1);
  nh.advertise<int8_t>("test_int8", 1);
  nh.advertise<std::string>("test_string", 1);
  nh.advertise<uint16_t>("test_uint16", 1);
  nh.advertise<uint32_t>("test_uint32", 1);
  nh.advertise<uint64_t>("test_uint64", 1);
  nh.advertise<uint8_t>("test_uint8", 1);
}

template<typename T>
void callback(const boost::shared_ptr<T const>&)
{}

TEST(BuiltinTypes, subscribe)
{
  ros::NodeHandle nh;
  nh.subscribe("test_bool", 1, callback<bool>);
  nh.subscribe("test_double", 1, callback<double>);
  nh.subscribe("test_float", 1, callback<float>);
  nh.subscribe("test_int16", 1, callback<int16_t>);
  nh.subscribe("test_int32", 1, callback<int32_t>);
  nh.subscribe("test_int64", 1, callback<int64_t>);
  nh.subscribe("test_int8", 1, callback<int8_t>);
  nh.subscribe("test_string", 1, callback<std::string>);
  nh.subscribe("test_uint16", 1, callback<uint16_t>);
  nh.subscribe("test_uint32", 1, callback<uint32_t>);
  nh.subscribe("test_uint64", 1, callback<uint64_t>);
  nh.subscribe("test_uint8", 1, callback<uint8_t>);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "builtin_types");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}




