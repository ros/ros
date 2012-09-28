/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 * Test handles
 */

#include <string>
#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <roscpp/TestArray.h>
#include <roscpp/TestStringString.h>

#include <boost/thread.hpp>

using namespace ros;
using namespace roscpp;

TEST(RoscppHandles, nodeHandleConstructionDestruction)
{
  {
    ASSERT_FALSE(ros::isStarted());

    ros::NodeHandle n1;
    ASSERT_TRUE(ros::isStarted());

    {
      ros::NodeHandle n2;
      ASSERT_TRUE(ros::isStarted());

      {
        ros::NodeHandle n3(n2);
        ASSERT_TRUE(ros::isStarted());

        {
          ros::NodeHandle n4 = n3;
          ASSERT_TRUE(ros::isStarted());
        }
      }
    }

    ASSERT_TRUE(ros::isStarted());
  }

  ASSERT_FALSE(ros::isStarted());

  {
    ros::NodeHandle n;
    ASSERT_TRUE(ros::isStarted());
  }

  ASSERT_FALSE(ros::isStarted());
}

TEST(RoscppHandles, nodeHandleParentWithRemappings)
{
  ros::M_string remappings;
  remappings["a"] = "b";
  remappings["c"] = "d";
  ros::NodeHandle n1("", remappings);

  // sanity checks
  EXPECT_STREQ(n1.resolveName("a").c_str(), "/b");
  EXPECT_STREQ(n1.resolveName("/a").c_str(), "/b");
  EXPECT_STREQ(n1.resolveName("c").c_str(), "/d");
  EXPECT_STREQ(n1.resolveName("/c").c_str(), "/d");

  ros::NodeHandle n2(n1, "my_ns");
  EXPECT_STREQ(n2.resolveName("a").c_str(), "/my_ns/a");
  EXPECT_STREQ(n2.resolveName("/a").c_str(), "/b");
  EXPECT_STREQ(n2.resolveName("c").c_str(), "/my_ns/c");
  EXPECT_STREQ(n2.resolveName("/c").c_str(), "/d");

  ros::NodeHandle n3(n2);
  EXPECT_STREQ(n3.resolveName("a").c_str(), "/my_ns/a");
  EXPECT_STREQ(n3.resolveName("/a").c_str(), "/b");
  EXPECT_STREQ(n3.resolveName("c").c_str(), "/my_ns/c");
  EXPECT_STREQ(n3.resolveName("/c").c_str(), "/d");

  ros::NodeHandle n4;
  n4 = n3;
  EXPECT_STREQ(n4.resolveName("a").c_str(), "/my_ns/a");
  EXPECT_STREQ(n4.resolveName("/a").c_str(), "/b");
  EXPECT_STREQ(n4.resolveName("c").c_str(), "/my_ns/c");
  EXPECT_STREQ(n4.resolveName("/c").c_str(), "/d");
}

int32_t g_recv_count = 0;
void subscriberCallback(const roscpp::TestArray::ConstPtr& msg)
{
  ++g_recv_count;
}

class SubscribeHelper
{
public:
  SubscribeHelper()
  : recv_count_(0)
  {}

  void callback(const roscpp::TestArray::ConstPtr& msg)
  {
    ++recv_count_;
  }

  int32_t recv_count_;
};

TEST(RoscppHandles, subscriberValidity)
{
  ros::NodeHandle n;

  ros::Subscriber sub;
  ASSERT_FALSE(sub);

  sub = n.subscribe("test", 0, subscriberCallback);
  ASSERT_TRUE(sub);
}

TEST(RoscppHandles, subscriberDestructionMultipleCallbacks)
{
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<roscpp::TestArray>("test", 0);
  roscpp::TestArray msg;

  {
    SubscribeHelper helper;
    ros::Subscriber sub_class = n.subscribe("test", 0, &SubscribeHelper::callback, &helper);

    ros::Duration d(0.05);
    int32_t last_class_count = helper.recv_count_;
    while (last_class_count == helper.recv_count_)
    {
      pub.publish(msg);
      ros::spinOnce();
      d.sleep();
    }

    int32_t last_fn_count = g_recv_count;
    {
      ros::Subscriber sub_fn = n.subscribe("test", 0, subscriberCallback);

      ASSERT_TRUE(sub_fn != sub_class);

      last_fn_count = g_recv_count;
      while (last_fn_count == g_recv_count)
      {
        pub.publish(msg);
        ros::spinOnce();
        d.sleep();
      }
    }

    last_fn_count = g_recv_count;
    last_class_count = helper.recv_count_;
    while (last_class_count == helper.recv_count_)
    {
      pub.publish(msg);
      ros::spinOnce();
      d.sleep();
    }
    d.sleep();

    ASSERT_EQ(last_fn_count, g_recv_count);
  }
}

TEST(RoscppHandles, subscriberSpinAfterSubscriberShutdown)
{
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<roscpp::TestArray>("test", 0);
  roscpp::TestArray msg;

  int32_t last_fn_count = g_recv_count;

  {
    ros::Subscriber sub_fn = n.subscribe("test", 0, subscriberCallback);

    last_fn_count = g_recv_count;
    for (int i = 0; i < 10; ++i)
    {
      pub.publish(msg);
    }

    ros::WallDuration(0.1).sleep();
  }

  ros::spinOnce();

  ASSERT_EQ(last_fn_count, g_recv_count);
}

TEST(RoscppHandles, subscriberGetNumPublishers)
{
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<roscpp::TestArray>("test", 0);

	ros::Subscriber sub = n.subscribe("test", 0, subscriberCallback);

	ros::WallTime begin = ros::WallTime::now();
	while (sub.getNumPublishers() < 1 && (ros::WallTime::now() - begin < ros::WallDuration(1)))
	{
		ros::spinOnce();
		ros::WallDuration(0.1).sleep();
	}

	ASSERT_EQ(sub.getNumPublishers(), 1ULL);
}

TEST(RoscppHandles, subscriberCopy)
{
  ros::NodeHandle n;

  g_recv_count = 0;

  {
    ros::Subscriber sub1 = n.subscribe("/test", 0, subscriberCallback);

    {
      ros::Subscriber sub2 = sub1;

      {
        ros::Subscriber sub3(sub2);

        ASSERT_TRUE(sub3 == sub2);

        V_string topics;
        this_node::getSubscribedTopics(topics);
        ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
      }

      ASSERT_TRUE(sub2 == sub1);

      V_string topics;
      this_node::getSubscribedTopics(topics);
      ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
    }

    V_string topics;
    this_node::getSubscribedTopics(topics);
    ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
  }

  V_string topics;
  this_node::getSubscribedTopics(topics);
  ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") == topics.end());
}

TEST(RoscppHandles, publisherCopy)
{
  ros::NodeHandle n;

  g_recv_count = 0;

  {
    ros::Publisher pub1 = n.advertise<roscpp::TestArray>("/test", 0);

    {
      ros::Publisher pub2 = pub1;

      {
        ros::Publisher pub3(pub2);

        ASSERT_TRUE(pub3 == pub2);

        V_string topics;
        this_node::getAdvertisedTopics(topics);
        ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
      }

      ASSERT_TRUE(pub2 == pub1);

      V_string topics;
      this_node::getAdvertisedTopics(topics);
      ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
    }

    V_string topics;
    this_node::getAdvertisedTopics(topics);
    ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
  }

  V_string topics;
  this_node::getAdvertisedTopics(topics);
  ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") == topics.end());
}

TEST(RoscppHandles, publisherMultiple)
{
  ros::NodeHandle n;

  g_recv_count = 0;

  {
    ros::Publisher pub1 = n.advertise<roscpp::TestArray>("/test", 0);

    {
      ros::Publisher pub2 = n.advertise<roscpp::TestArray>("/test", 0);

      ASSERT_TRUE(pub1 != pub2);

      V_string topics;
      this_node::getAdvertisedTopics(topics);
      ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
    }

    V_string topics;
    this_node::getAdvertisedTopics(topics);
    ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
  }

  V_string topics;
  this_node::getAdvertisedTopics(topics);
  ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") == topics.end());
}

bool serviceCallback(TestStringString::Request& req, TestStringString::Response& res)
{
  return true;
}

void pump(ros::CallbackQueue* queue)
{
  while (queue->isEnabled())
  {
    queue->callAvailable(ros::WallDuration(0.1));
  }
}

TEST(RoscppHandles, serviceAdv)
{
  ros::NodeHandle n;
  TestStringString t;

  ros::CallbackQueue queue;
  n.setCallbackQueue(&queue);
  boost::thread th(boost::bind(pump, &queue));
  {
    ros::ServiceServer srv = n.advertiseService("/test_srv", serviceCallback);

    EXPECT_TRUE(ros::service::call("/test_srv", t));
  }

  queue.disable();
  th.join();

  ASSERT_FALSE(ros::service::call("/test_srv", t));
}

TEST(RoscppHandles, serviceAdvCopy)
{
  ros::NodeHandle n;
  TestStringString t;

  ros::CallbackQueue queue;
  n.setCallbackQueue(&queue);
  boost::thread th(boost::bind(pump, &queue));

  {
    ros::ServiceServer srv1 = n.advertiseService("/test_srv", serviceCallback);

    {
      ros::ServiceServer srv2 = srv1;

      {
        ros::ServiceServer srv3(srv2);

        ASSERT_TRUE(srv3 == srv2);

        EXPECT_TRUE(ros::service::call("/test_srv", t));
      }

      ASSERT_TRUE(srv2 == srv1);

      EXPECT_TRUE(ros::service::call("/test_srv", t));
    }

    EXPECT_TRUE(ros::service::call("/test_srv", t));
  }

  ASSERT_FALSE(ros::service::call("/test_srv", t));

  queue.disable();
  th.join();
}

TEST(RoscppHandles, serviceAdvMultiple)
{
  ros::NodeHandle n;

  ros::ServiceServer srv = n.advertiseService("/test_srv", serviceCallback);
  ros::ServiceServer srv2 = n.advertiseService("/test_srv", serviceCallback);
  ASSERT_TRUE(srv);
  ASSERT_FALSE(srv2);

  ASSERT_TRUE(srv != srv2);
}

int32_t g_sub_count = 0;
void connectedCallback(const ros::SingleSubscriberPublisher& pub)
{
  ++g_sub_count;
}

TEST(RoscppHandles, trackedObjectWithAdvertiseSubscriberCallback)
{
  ros::NodeHandle n;

  boost::shared_ptr<char> tracked(new char);

  ros::Publisher pub = n.advertise<roscpp::TestArray>("/test", 0, connectedCallback, SubscriberStatusCallback(), tracked);

  g_recv_count = 0;
  g_sub_count = 0;
  ros::Subscriber sub = n.subscribe("/test", 0, subscriberCallback);

  Duration d(0.01);
  while (g_sub_count == 0)
  {
    d.sleep();
    ros::spinOnce();
  }
  ASSERT_EQ(g_sub_count, 1);

  sub.shutdown();

  tracked.reset();
  sub = n.subscribe("/test", 0, subscriberCallback);

  Duration d2(0.01);
  for (int i = 0; i < 10; ++i)
  {
    d2.sleep();
    ros::spinOnce();
  }

  ASSERT_EQ(g_sub_count, 1);
}

TEST(RoscppHandles, spinAfterHandleShutdownWithAdvertiseSubscriberCallback)
{
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<roscpp::TestArray>("/test", 0, connectedCallback, SubscriberStatusCallback());

  g_sub_count = 0;
  ros::Subscriber sub = n.subscribe("/test", 0, subscriberCallback);

  while (pub.getNumSubscribers() == 0)
  {
    ros::WallDuration(0.01).sleep();
  }

  pub.shutdown();

  ros::spinOnce();

  ASSERT_EQ(g_sub_count, 0);
}

TEST(RoscppHandles, multiplePublishersWithSubscriberConnectCallback)
{
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<roscpp::TestArray>("/test", 0, connectedCallback, SubscriberStatusCallback());

  g_sub_count = 0;
  ros::Subscriber sub = n.subscribe("/test", 0, subscriberCallback);

  while (g_sub_count == 0)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  ASSERT_EQ(g_sub_count, 1);
  g_sub_count = 0;

  ros::Publisher pub2 = n.advertise<roscpp::TestArray>("/test", 0, connectedCallback, SubscriberStatusCallback());
  ros::spinOnce();

  ASSERT_EQ(g_sub_count, 1);
}

class ServiceClass
{
public:
  bool serviceCallback(TestStringString::Request& req, TestStringString::Response& res)
  {
    return true;
  }
};

TEST(RoscppHandles, trackedObjectWithServiceCallback)
{
  ros::NodeHandle n;

  ros::CallbackQueue queue;
  n.setCallbackQueue(&queue);
  boost::thread th(boost::bind(pump, &queue));

  boost::shared_ptr<ServiceClass> tracked(new ServiceClass);
  ros::ServiceServer srv = n.advertiseService("/test_srv", &ServiceClass::serviceCallback, tracked);

  TestStringString t;
  EXPECT_TRUE(ros::service::call("/test_srv", t));

  tracked.reset();

  ASSERT_FALSE(ros::service::call("/test_srv", t));

  queue.disable();
  th.join();
}

TEST(RoscppHandles, trackedObjectWithSubscriptionCallback)
{
  ros::NodeHandle n;

  boost::shared_ptr<SubscribeHelper> tracked(new SubscribeHelper);

  g_recv_count = 0;
  ros::Subscriber sub = n.subscribe("/test", 0, &SubscribeHelper::callback, tracked);

  ros::Publisher pub = n.advertise<roscpp::TestArray>("/test", 0);

  roscpp::TestArray msg;
  Duration d(0.01);
  while (tracked->recv_count_ == 0)
  {
    pub.publish(msg);
    d.sleep();
    ros::spinOnce();
  }
  ASSERT_GE(tracked->recv_count_, 1);

  tracked.reset();

  pub.publish(msg);
  Duration d2(0.01);
  for (int i = 0; i < 10; ++i)
  {
    d2.sleep();
    ros::spinOnce();
  }
}

TEST(RoscppHandles, nodeHandleNames)
{
  ros::NodeHandle n1;
  EXPECT_STREQ(n1.resolveName("blah").c_str(), "/blah");

  try
  {
    n1.resolveName("~blah");
    FAIL();
  }
  catch (ros::InvalidNameException&)
  {
  }

  ros::NodeHandle n2("internal_ns");
  EXPECT_STREQ(n2.resolveName("blah").c_str(), "/internal_ns/blah");

  ros::NodeHandle n3(n2, "2");
  EXPECT_STREQ(n3.resolveName("blah").c_str(), "/internal_ns/2/blah");

  ros::NodeHandle n4("~");
  EXPECT_STREQ(n4.resolveName("blah").c_str(), (ros::this_node::getName() + "/blah").c_str());
  
  try {
    ros::NodeHandle n5(n2, "illegal_name!!!");
    FAIL();
  } catch (ros::InvalidNameException&) { }

}

TEST(RoscppHandles, nodeHandleShutdown)
{
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/test", 0, subscriberCallback);
  ros::Publisher pub = n.advertise<roscpp::TestArray>("/test", 0);
  ros::ServiceServer srv = n.advertiseService("/test_srv", serviceCallback);

  n.shutdown();

  ASSERT_FALSE(pub);
  ASSERT_FALSE(sub);
  ASSERT_FALSE(srv);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_handles");

  return RUN_ALL_TESTS();
}

