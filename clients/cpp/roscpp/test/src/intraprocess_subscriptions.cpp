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
 * Subscribe intraprocess, ensuring that no copy happens
 */

#include <string>

#include <gtest/gtest.h>

#include <stdlib.h>

#include "ros/ros.h"

uint32_t g_msg_constructor = 0;
uint32_t g_msg2_constructor = 0;

struct Msg
{
  Msg()
  : serialized(false)
  , deserialized(false)
  {
    ++g_msg_constructor;
  }

  mutable bool serialized;
  bool deserialized;
};
typedef boost::shared_ptr<Msg const> MsgConstPtr;

namespace ros
{
namespace message_traits
{
  template<>
  struct MD5Sum<Msg>
  {
    static const char* value() { return "MsgMD5Sum"; }
    static const char* value(const Msg&) { return "MsgMD5Sum"; }
  };

  template<>
  struct DataType<Msg>
  {
    static const char* value() { return "roscpp/MsgDataType"; }
    static const char* value(const Msg&) { return "roscpp/MsgDataType"; }
  };

  template<>
  struct Definition<Msg>
  {
    static const char* value() { return ""; }
    static const char* value(const Msg&) { return ""; }
  };
} // namespace message_traits

namespace serialization
{
template<>
struct Serializer<Msg>
{
  template<typename Stream>
  inline static void write(Stream& stream, const Msg& v)
  {
    v.serialized = true;
  }

  template<typename Stream>
  inline static void read(Stream& stream, Msg& v)
  {
    v.deserialized = true;
  }

  inline static uint32_t serializedLength(const Msg& v)
  {
    return 0;
  }
};
} // namespace serialization
} // namespace ros

MsgConstPtr g_msg;

struct Msg2
{
  Msg2()
  : serialized(false)
  , deserialized(false)
  {
    ++g_msg2_constructor;
  }

  mutable bool serialized;
  bool deserialized;
};
typedef boost::shared_ptr<Msg2 const> Msg2ConstPtr;

namespace ros
{
namespace message_traits
{
  template<>
  struct MD5Sum<Msg2>
  {
    static const char* value() { return "MsgMD5Sum"; }
    static const char* value(const Msg2&) { return "MsgMD5Sum"; }
  };

  template<>
  struct DataType<Msg2>
  {
    static const char* value() { return "roscpp/MsgDataType"; }
    static const char* value(const Msg2&) { return "roscpp/MsgDataType"; }
  };

  template<>
  struct Definition<Msg2>
  {
    static const char* value() { return ""; }
    static const char* value(const Msg2&) { return ""; }
  };
} // namespace message_traits

namespace serialization
{
template<>
struct Serializer<Msg2>
{
  template<typename Stream>
  inline static void write(Stream& stream, const Msg2& v)
  {
    v.serialized = true;
  }

  template<typename Stream>
  inline static void read(Stream& stream, Msg2& v)
  {
    v.deserialized = true;
  }

  inline static uint32_t serializedLength(const Msg2& v)
  {
    return 0;
  }
};
} // namespace serialization
} // namespace ros

void messageCallback(const MsgConstPtr& msg)
{
  g_msg = msg;
}

TEST(IntraprocessSubscriptions, noCopy)
{
  g_msg.reset();
  g_msg_constructor = 0;

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("test", 0, messageCallback);
  ros::Publisher pub = nh.advertise<Msg>("test", 0);

  MsgConstPtr msg(new Msg);

  while (pub.getNumSubscribers() == 0)
  {
    ros::Duration(0.01).sleep();
  }

  pub.publish(msg);

  while (!g_msg)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  ASSERT_TRUE(g_msg);
  EXPECT_EQ(g_msg.get(), msg.get());
  EXPECT_FALSE(g_msg->serialized);
  EXPECT_FALSE(g_msg->deserialized);
  EXPECT_EQ(g_msg_constructor, 1ULL);
}

TEST(IntraprocessSubscriptions, differentRTTI)
{
  g_msg_constructor = 0;
  g_msg.reset();

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("test", 0, messageCallback);
  ros::Publisher pub = nh.advertise<Msg2>("test", 0);

  Msg2ConstPtr msg(new Msg2);

  while (pub.getNumSubscribers() == 0)
  {
    ros::Duration(0.01).sleep();
  }

  pub.publish(msg);

  while (!g_msg)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  ASSERT_TRUE(g_msg);
  EXPECT_NE((void*)g_msg.get(), (void*)msg.get());
  EXPECT_TRUE(msg->serialized);
  EXPECT_TRUE(g_msg->deserialized);
  EXPECT_EQ(g_msg_constructor, 1ULL);
  EXPECT_EQ(g_msg2_constructor, 1ULL);
}

Msg2ConstPtr g_msg2;
void messageCallback2(const Msg2ConstPtr& msg)
{
  g_msg2 = msg;
}

TEST(IntraprocessSubscriptions, noCopyAndDifferentRTTI)
{
  g_msg.reset();

  ros::NodeHandle nh;
  ros::Subscriber sub1 = nh.subscribe("test", 0, messageCallback);
  ros::Subscriber sub2 = nh.subscribe("test", 0, messageCallback2);
  ros::Publisher pub = nh.advertise<Msg2>("test", 0);

  Msg2ConstPtr msg(new Msg2);

  while (pub.getNumSubscribers() == 0)
  {
    ros::Duration(0.01).sleep();
  }

  pub.publish(msg);

  while (!g_msg || !g_msg2)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  ASSERT_TRUE(g_msg);
  EXPECT_NE((void*)g_msg.get(), (void*)msg.get());
  EXPECT_TRUE(msg->serialized);
  EXPECT_TRUE(g_msg->deserialized);

  ASSERT_TRUE(g_msg2);
  EXPECT_EQ(g_msg2.get(), msg.get());
  EXPECT_TRUE(g_msg2->serialized);
  EXPECT_FALSE(g_msg2->deserialized);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "intraprocess_subscriptions");

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}

