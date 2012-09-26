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

/* Author: Josh Faust */

/*
 * Test PreDeserialize trait
 */

#include <gtest/gtest.h>

#include <ros/ros.h>

// To avoid the intraprocess optimization we use two messages

struct OutgoingMsg
{
};

struct IncomingMsg
{
  IncomingMsg()
  : touched(false)
  {}

  bool touched;
};

ROS_IMPLEMENT_SIMPLE_TOPIC_TRAITS(IncomingMsg, "my_md5sum", "test_roscpp_serialization/IncomingMsg", "\n");
ROS_IMPLEMENT_SIMPLE_TOPIC_TRAITS(OutgoingMsg, "my_md5sum", "test_roscpp_serialization/OutgoingMsg", "\n");

namespace ros
{

namespace serialization
{
template<>
struct Serializer<IncomingMsg>
{
  template<typename Stream>
  inline static void write(Stream& stream, const IncomingMsg& v)
  {
  }

  template<typename Stream>
  inline static void read(Stream& stream, IncomingMsg& v)
  {
  }

  inline static uint32_t serializedLength(const IncomingMsg& v)
  {
    return 0;
  }
};

template<>
struct Serializer<OutgoingMsg>
{
  template<typename Stream>
  inline static void write(Stream& stream, const OutgoingMsg& v)
  {
  }

  template<typename Stream>
  inline static void read(Stream& stream, OutgoingMsg& v)
  {
  }

  inline static uint32_t serializedLength(const OutgoingMsg& v)
  {
    return 0;
  }
};

template<>
struct PreDeserialize<IncomingMsg>
{
  static void notify(const PreDeserializeParams<IncomingMsg>& params)
  {
    params.message->touched = true;
  }
};
} // namespace serialization
} // namespace ros

boost::shared_ptr<IncomingMsg const> g_msg;
void callback(const boost::shared_ptr<IncomingMsg const>& msg)
{
  g_msg = msg;
}

TEST(PreDeserialize, preDeserialize)
{
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<OutgoingMsg>("test", 0);
  ros::Subscriber sub = nh.subscribe("test", 0, callback);

  pub.publish(OutgoingMsg());

  while (!g_msg)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  EXPECT_TRUE(g_msg->touched);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "builtin_types");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}





