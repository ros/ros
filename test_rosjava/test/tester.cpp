/*
 * Copyright (C) 2008, Jason Wolfe and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
#include "test_rosjava/TestDataTypes.h"
#include "test_rosjava/TestTwoInts.h"
#include "std_msgs/String.h"
#include <gtest/gtest.h>
#include <ros/time.h>
#include <ros/console.h>

using namespace test_rosjava;

int num_messages_received=0;
std::string last_message = "";
TestDataTypes test_in;

void chatterCallback(const std_msgs::StringConstPtr& msg)
{
  num_messages_received++;
  last_message = msg->data;
}

void tdtCallback(const boost::shared_ptr<const TestDataTypes>& msg)
{
  test_in = *msg;
}

bool add(TestTwoInts::Request  &req,
         TestTwoInts::Response &res )
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("  sending back response: [%ld]", (long int)res.sum);
  return true;
}



TEST(Rosjava, rosjava)
{
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("talk", 1000, chatterCallback);
  ros::Subscriber sub2 = n.subscribe("test-talk", 1000, tdtCallback);
  ros::Duration d(1);

  ros::ServiceServer server = n.advertiseService("add_two_ints_cpp", add);

  int wait_max=180;

  for (int i=0; i<wait_max && n.ok(); i++) {
    d.sleep();
    ros::spinOnce();
    ROS_INFO_STREAM ("Have waited " << i << " out of " << wait_max << " seconds to receive a message from chatter-echo");
    if (num_messages_received>0)
      break;
  }
  EXPECT_TRUE(num_messages_received>0);
  ROS_INFO_STREAM ("Successfully received a startup message.");

  ros::Publisher pub = n.advertise<std_msgs::String>("listen", 10);
  ros::Publisher pub2 = n.advertise<TestDataTypes>("test-listen", 1);
  std_msgs::String msg;
  msg.data = "Hi";

  n.setParam("int_param", 1);
  n.setParam("double_param", 1.0);
  n.setParam("string_param", "hello");

  int i = 0;
  while (pub.getNumSubscribers() < 1 && i++ < wait_max) {
	  ros::spinOnce();
	  d.sleep();
	  ROS_INFO_STREAM("Waiting for subscriber...");
  }

  EXPECT_TRUE(pub.getNumSubscribers() == 1);
  pub.publish(msg);

  d.sleep();
  ros::spinOnce();
  num_messages_received=0;
  while(num_messages_received == 0 && i++ < wait_max) {
	  ros::spinOnce();
	  d.sleep();
  }
  EXPECT_EQ(last_message, "good"); // Result for testint params in Java
  ROS_INFO_STREAM("Successfully tested reading parameters");

  // Test parameters
  int ip; double dp; std::string sp;
  n.getParam("int_param2", ip);
  n.getParam("double_param2", dp);
  n.getParam("string_param2", sp);
  EXPECT_EQ(ip, 1);
  EXPECT_EQ(dp, 1.0);
  EXPECT_EQ(sp, "hello");

  ROS_INFO_STREAM("Successfully tested writing parameters");

  // Test service client
  TestTwoInts::Request req;
  TestTwoInts::Response res;
  req.a = 7; req.b = 4;
  ros::ServiceClient service = n.serviceClient<TestTwoInts>("add_two_ints_java");
  EXPECT_TRUE(service.call(req, res));
  EXPECT_EQ(res.sum, 11);

  service = n.serviceClient<TestTwoInts>("add_two_ints_java", true);
  req.b = 5;
  EXPECT_TRUE(service.call(req, res));
  EXPECT_EQ(res.sum, 12);
  req.a = 8;
  EXPECT_TRUE(service.call(req, res));
  EXPECT_EQ(res.sum, 13);

  num_messages_received=0;
  while(num_messages_received == 0 && i++ < wait_max) {
	  ros::spinOnce();
	  d.sleep();
  }
  EXPECT_EQ(last_message, "good"); // Result for calling C++ service from java.

  ROS_INFO_STREAM("Successfully tested services");

  TestDataTypes test;
  test.byte_ = 0xab;
  test.char_ = 0xbc;
  test.uint8_ = 0xcd;
  test.int8_  = 0xde;
  test.uint16_ = 0xabcd;
  test.int16_  = 0xbcde;
  test.uint32_ = 0xdeadbeef;
  test.int32_  = 0xcabcabbe;
  test.uint64_ = 0xbeefcabedeaddeedLL;
  test.int64_  = 0xfeedbabedeadbeefLL;
  test.float32_ = 1.0;
  test.float64_  = -1.0;
  test.string_ = "hello";
  test.time_ = ros::Time(123,456);
  test.duration_ = ros::Duration(789,987);

  test.byte_v.push_back(11);
  test.byte_f.resize(0);
  test.byte_f.push_back(22);
  test.byte_f.push_back(33);

  test.float64_v.push_back(1.0);
  test.float64_f.resize(0);
  test.float64_f.push_back(2.0);
  test.float64_f.push_back(3.0);

  test.string_v.push_back("test1");
  test.string_f.resize(0);
  test.string_f.push_back("");
  test.string_f.push_back("test3");

  test.time_v.push_back(ros::Time(222,333));
  test.time_f.resize(0);
  test.time_f.push_back(ros::Time(444,555));
  test.time_f.push_back(ros::Time(666,777));

  test.Byte_.data = 1;
  std_msgs::Byte tmp;
  tmp.data = 2;
  test.Byte_v.push_back(tmp);
  tmp.data = 3;
  test.Byte_v.push_back(tmp);

  std_msgs::ByteMultiArray tmp2;
  std_msgs::MultiArrayDimension tmp3;
  tmp3.label="test";
  tmp3.size=1;
  tmp3.stride=1;
  tmp2.layout.dim.push_back(tmp3);
  tmp2.layout.data_offset=0;
  tmp2.data.push_back(11);

  test.ByteMultiArray_ = tmp2;
  test.ByteMultiArray_v.push_back(tmp2);

  pub2.publish(test);

  num_messages_received=0;
  while(num_messages_received == 0 && i++ < wait_max) {
	  ros::spinOnce();
	  d.sleep();
  }
  EXPECT_EQ(num_messages_received, 1);
  EXPECT_EQ(last_message, "good"); // Result for reading message in Java.

  // Now, test we got things back as expected.
  EXPECT_EQ(test_in.byte_, test.byte_);
  EXPECT_EQ(test_in.char_, test.char_);
  EXPECT_EQ(test_in.uint8_, test.uint8_);
  EXPECT_EQ(test_in.int8_, test.int8_);
  EXPECT_EQ(test_in.uint16_, test.uint16_);
  EXPECT_EQ(test_in.int16_, test.int16_);
  EXPECT_EQ(test_in.uint32_, test.uint32_);
  EXPECT_EQ(test_in.int32_, test.int32_);
  EXPECT_EQ(test_in.uint64_, test.uint64_);
  EXPECT_EQ(test_in.int64_, test.int64_);
  EXPECT_EQ(test_in.float32_, test.float32_);
  EXPECT_EQ(test_in.float64_, test.float64_);
  EXPECT_EQ(test_in.string_, test.string_);
  EXPECT_EQ(test_in.time_, test.time_);
  EXPECT_EQ(test_in.duration_, test.duration_);

  EXPECT_EQ(test_in.byte_v.size(), test.byte_v.size());
  EXPECT_EQ(test_in.byte_v[0], test.byte_v[0]);
  EXPECT_EQ(test_in.byte_f[0], test.byte_f[0]);
  EXPECT_EQ(test_in.byte_f[1], test.byte_f[1]);

  EXPECT_EQ(test_in.float64_v.size(), test.float64_v.size());
  EXPECT_EQ(test_in.float64_v[0], test.float64_v[0]);
  EXPECT_EQ(test_in.float64_f[0], test.float64_f[0]);
  EXPECT_EQ(test_in.float64_f[1], test.float64_f[1]);

  EXPECT_EQ(test_in.string_v.size(), test.string_v.size());
  EXPECT_EQ(test_in.string_v[0], test.string_v[0]);
  EXPECT_EQ(test_in.string_f[0], test.string_f[0]);
  EXPECT_EQ(test_in.string_f[1], test.string_f[1]);

  EXPECT_EQ(test_in.time_v.size(), test.time_v.size());
  EXPECT_EQ(test_in.time_v[0], test.time_v[0]);
  EXPECT_EQ(test_in.time_f[0], test.time_f[0]);
  EXPECT_EQ(test_in.time_f[1], test.time_f[1]);

  EXPECT_EQ(test_in.Byte_.data, test.Byte_.data);
  EXPECT_EQ(test_in.Byte_v.size(), test.Byte_v.size());
  EXPECT_EQ(test_in.Byte_v[0].data, test.Byte_v[0].data);
  EXPECT_EQ(test_in.Byte_v[1].data, test.Byte_v[1].data);

  EXPECT_EQ(test_in.ByteMultiArray_.layout.dim.size(), test.ByteMultiArray_.layout.dim.size());
  EXPECT_EQ(test_in.ByteMultiArray_.layout.dim[0].label, test.ByteMultiArray_.layout.dim[0].label);
  EXPECT_EQ(test_in.ByteMultiArray_.layout.dim[0].size, test.ByteMultiArray_.layout.dim[0].size);
  EXPECT_EQ(test_in.ByteMultiArray_.layout.dim[0].stride, test.ByteMultiArray_.layout.dim[0].stride);
  EXPECT_EQ(test_in.ByteMultiArray_.layout.data_offset, test.ByteMultiArray_.layout.data_offset);
  EXPECT_EQ(test_in.ByteMultiArray_.data.size(), test.ByteMultiArray_.data.size());
  EXPECT_EQ(test_in.ByteMultiArray_.data[0], test.ByteMultiArray_.data[0]);

  EXPECT_EQ(test_in.ByteMultiArray_v.size(), test.ByteMultiArray_v.size());
  EXPECT_EQ(test_in.ByteMultiArray_v[0].layout.dim.size(), test.ByteMultiArray_v[0].layout.dim.size());
  EXPECT_EQ(test_in.ByteMultiArray_v[0].layout.dim[0].label, test.ByteMultiArray_v[0].layout.dim[0].label);
  EXPECT_EQ(test_in.ByteMultiArray_v[0].layout.dim[0].size, test.ByteMultiArray_v[0].layout.dim[0].size);
  EXPECT_EQ(test_in.ByteMultiArray_v[0].layout.dim[0].stride, test.ByteMultiArray_v[0].layout.dim[0].stride);
  EXPECT_EQ(test_in.ByteMultiArray_v[0].layout.data_offset, test.ByteMultiArray_v[0].layout.data_offset);
  EXPECT_EQ(test_in.ByteMultiArray_v[0].data.size(), test.ByteMultiArray_v[0].data.size());
  EXPECT_EQ(test_in.ByteMultiArray_v[0].data[0], test.ByteMultiArray_v[0].data[0]);

  ROS_INFO_STREAM("Successfully tested messages");

}



int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");

//  return -1;
  return RUN_ALL_TESTS();
}
