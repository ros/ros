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

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/connection_manager.h>

#include "test_roscpp/TestEmpty.h"
#include "test_roscpp/TestArray.h"

#include <std_srvs/Empty.h>

class AnyMessage
{
};
typedef boost::shared_ptr<AnyMessage> AnyMessagePtr;
typedef boost::shared_ptr<AnyMessage const> AnyMessageConstPtr;

namespace ros
{
namespace message_traits
{

template<>
struct MD5Sum<AnyMessage>
{
  static const char* value() { return "*"; }
  static const char* value(const AnyMessage&) { return "*"; }
};

template<>
struct DataType<AnyMessage>
{
  static const char* value() { return "*"; }
  static const char* value(const AnyMessage&) { return "*"; }
};

template<>
struct Definition<AnyMessage>
{
};

}

namespace serialization
{
template<>
struct Serializer<AnyMessage>
{
  template<typename Stream, typename T>
  static void allInOne(Stream s, T t)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
};
}
}

struct AnyHelper
{
  AnyHelper()
  : count(0)
  {
  }

  void cb(const AnyMessageConstPtr& msg)
  {
    ++count;
  }

  uint32_t count;
};


TEST(SubscribeStar, simpleSubFirstIntra)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub = nh.subscribe("test_star_intra", 0, &AnyHelper::cb, &h);
  ros::Publisher pub = nh.advertise<test_roscpp::TestEmpty>("test_star_intra", 0);

  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);

  AnyMessagePtr msg(new AnyMessage);
  pub.publish(msg);
  ros::spinOnce();
  EXPECT_EQ(h.count, 1U);
}

TEST(SubscribeStar, simplePubFirstIntra)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Publisher pub = nh.advertise<test_roscpp::TestEmpty>("test_star_intra", 0);
  ros::Subscriber sub = nh.subscribe("test_star_intra", 0, &AnyHelper::cb, &h);

  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);

  AnyMessagePtr msg(new AnyMessage);
  pub.publish(msg);
  ros::spinOnce();
  EXPECT_EQ(h.count, 1U);
}

void emptyCallback(const test_roscpp::TestEmptyConstPtr&)
{

}

TEST(SubscribeStar, multipleSubsStarFirstIntra)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub = nh.subscribe("test_star_intra", 0, &AnyHelper::cb, &h);
  ros::Subscriber sub2 = nh.subscribe("test_star_intra", 0, emptyCallback);

  ros::Publisher pub = nh.advertise<test_roscpp::TestEmpty>("test_star_intra", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);
  EXPECT_EQ(sub2.getNumPublishers(), 1U);

  pub.shutdown();
  pub = nh.advertise<test_roscpp::TestArray>("test_star_intra", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 0U);
  EXPECT_EQ(sub.getNumPublishers(), 0U);
  EXPECT_EQ(sub2.getNumPublishers(), 0U);
}

TEST(SubscribeStar, multipleSubsConcreteFirstIntra)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub2 = nh.subscribe("test_star_intra", 0, emptyCallback);
  ros::Subscriber sub = nh.subscribe("test_star_intra", 0, &AnyHelper::cb, &h);

  ros::Publisher pub = nh.advertise<test_roscpp::TestEmpty>("test_star_intra", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);
  EXPECT_EQ(sub2.getNumPublishers(), 1U);

  pub.shutdown();
  pub = nh.advertise<test_roscpp::TestArray>("test_star_intra", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 0U);
  EXPECT_EQ(sub.getNumPublishers(), 0U);
  EXPECT_EQ(sub2.getNumPublishers(), 0U);
}

TEST(SubscribeStar, multipleShutdownConcreteIntra)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub = nh.subscribe("test_star_intra", 0, &AnyHelper::cb, &h);
  ros::Subscriber sub2 = nh.subscribe("test_star_intra", 0, emptyCallback);
  sub2.shutdown();

  ros::Publisher pub = nh.advertise<test_roscpp::TestEmpty>("test_star_intra", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);

  pub.shutdown();
  pub = nh.advertise<test_roscpp::TestArray>("test_star_intra", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 0U);
  EXPECT_EQ(sub.getNumPublishers(), 0U);
}

TEST(SubscribeStar, simpleInter)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub = nh.subscribe("test_star_inter", 0, &AnyHelper::cb, &h);

  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  EXPECT_EQ(sub.getNumPublishers(), 1U);
  EXPECT_GT(h.count, 0U);
}

TEST(SubscribeStar, simpleInterUDP)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub = nh.subscribe("test_star_inter", 0, &AnyHelper::cb, &h, ros::TransportHints().udp());

  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  EXPECT_EQ(sub.getNumPublishers(), 1U);
  EXPECT_GT(h.count, 0U);
}

TEST(SubscribeStar, switchTypeInter)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub = nh.subscribe("test_star_inter", 0, &AnyHelper::cb, &h);

  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  ASSERT_EQ(sub.getNumPublishers(), 1U);

  std_srvs::Empty srv;
  ASSERT_TRUE(ros::service::call("switch_publisher_type", srv));

  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  ASSERT_EQ(sub.getNumPublishers(), 0U);
}

TEST(SubscribeStar, switchTypeInterUDP)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub = nh.subscribe("test_star_inter", 0, &AnyHelper::cb, &h, ros::TransportHints().udp());

  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  ASSERT_EQ(sub.getNumPublishers(), 1U);

  std_srvs::Empty srv;
  ASSERT_TRUE(ros::service::call("switch_publisher_type", srv));

  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  ASSERT_EQ(sub.getNumPublishers(), 0U);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "subscribe_star");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
