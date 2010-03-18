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
#include "std_msgs/Int16.h"
#include "roslisp/ComplexMessage.h"
#include <gtest/gtest.h>
#include <ros/time.h>
#include <ros/console.h>

using std::vector;

int num_messages_received=0;
int service_client_status=0;
int service_client_response=0;

using roslisp::ComplexMessage;
using roslisp::Foo;
using roslisp::Bar;

inline bool operator== (const Bar& b, const Bar& b2)
{
  return ((b.y == b2.y) && (b.x==b2.x));
}

inline bool operator== (const Foo& f, const Foo& f2)
{
  return ((f.y==f2.y) && (f.z[0]==f2.z[0]) && (f.z[1]==f2.z[1]));
}

inline
bool operator== (const ComplexMessage& c, const ComplexMessage& c2)
{
  unsigned i=0;
  
  if (c.ind != c2.ind)
    return false;
  
  for ( i=0; i<4; i++)
    if (!(c.x[i] == c2.x[i]))
      return false;
  
  for ( i=0; i<c.y.size(); i++)
    if (!(c.y[i] == c2.y[i]))
      return false;
  
  for ( i=0; i<c2.z.size(); i++) 
    if (!(c.z[i] == c2.z[i]))
      return false;

  for ( i=0; i<c.w.size(); i++)
    if (!(c.w[i].data == c2.w[i].data))
      return false;

  if (c.qux != c2.qux)
  {
    ROS_INFO ("Qux 1 is %u and qux 2 is %u", c.qux, c2.qux);
    return false;
  }
  
  return true;
}

class ComplexTester
{
public:
  ComplexTester() : ok_(true), num_received_(0)
  {
    ping_sub_ = nh_.subscribe("ping", 1000, &ComplexTester::pingCallback, this);
    pong_pub_ = nh_.advertise<ComplexMessage>("pong", 1000);
    pang_sub_ = nh_.subscribe("pang", 1000, &ComplexTester::pangCallback, this);
    ros::Duration d(3);
    d.sleep();
    ping_vec_.resize(8);
  }
  
  void pingCallback (const roslisp::ComplexMessage::ConstPtr& msg)
  {
    ping_vec_[msg->ind] = *msg;
    pong_pub_.publish(*msg);
  }

  void pangCallback (const roslisp::ComplexMessage::ConstPtr& msg)
  {
    ok_ &= (ping_vec_[msg->ind] == *msg);
    num_received_++;
  }
  
  bool done()
  {
    return (num_received_ >= 10);
  }

  bool isOk()
  {
    return ok_;
  }
  
private:
  
  vector<ComplexMessage> ping_vec_;
  bool ok_;
  unsigned num_received_;
  ros::Subscriber pang_sub_, ping_sub_;
  ros::Publisher pong_pub_;
  ros::NodeHandle nh_;
};  




TEST(Roslisp, ComplexTalkerListener)
{
  ros::Duration d(1);
  unsigned wait_max = 180;

  ComplexTester t;
  
  for (unsigned i=0; (i<wait_max) && (!t.done()); i++) {
    d.sleep();
    ros::spinOnce();
  }

  EXPECT_TRUE(t.done());
  EXPECT_TRUE(t.isOk());
}



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
  ros::Subscriber sub = n.subscribe("chatter_echo", 1000, chatterCallback);
  ros::Duration d(1);

  int wait_max=180;

  for (int i=0; i<wait_max && n.ok(); i++) {
    d.sleep();
    ros::spinOnce();
    ROS_INFO_STREAM ("Have waited " << i << " out of " << wait_max << " seconds to receive 5 messages from chatter_echo");
    if (num_messages_received>5)
      break;
  }
  EXPECT_TRUE(num_messages_received>5);
  ROS_INFO_STREAM ("Successfully received " << num_messages_received << " echoed messages");
}

void clientStatusCallback (const std_msgs::Int16ConstPtr& msg)
{
  service_client_response = msg->data;
  service_client_status = 1;
}

TEST(Roslisp, Service)
{
  ros::NodeHandle n;
  ros::Subscriber sub=n.subscribe("client_status", 1000, clientStatusCallback);
  ros::Duration d(1);
  unsigned wait_max=180;
  for (unsigned i=0; i<wait_max && n.ok(); i++) {
    d.sleep();
    ros::spinOnce();
    ROS_INFO_STREAM ("Have waited " << i << " out of " << wait_max << " seconds to receive confirmation from service client test");
    if (service_client_status)
      break;
  }

  EXPECT_EQ (66, service_client_response);
}

    

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "roslisp-tester");
  return RUN_ALL_TESTS();
}
