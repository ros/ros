/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <gtest/gtest.h>

#include "ros/time.h"
#include "message_filters/time_sequencer.h"

using namespace message_filters;

struct Header
{
  ros::Time stamp;
};


struct Msg
{
  Header header;
  int data;
};
typedef boost::shared_ptr<Msg> MsgPtr;
typedef boost::shared_ptr<Msg const> MsgConstPtr;

namespace ros
{
namespace message_traits
{
template<>
struct TimeStamp<Msg>
{
  static ros::Time value(const Msg& m)
  {
    return m.header.stamp;
  }
};
}
}

class Helper
{
public:
  Helper()
  : count_(0)
  {}

  void cb(const MsgConstPtr&)
  {
    ++count_;
  }

  int32_t count_;
};

TEST(TimeSequencer, simple)
{
  TimeSequencer<Msg> seq(ros::Duration(1.0), ros::Duration(0.01), 10);
  Helper h;
  seq.registerCallback(boost::bind(&Helper::cb, &h, _1));
  MsgPtr msg(new Msg);
  msg->header.stamp = ros::Time::now();
  seq.add(msg);

  ros::WallDuration(0.1).sleep();
  ros::spinOnce();
  ASSERT_EQ(h.count_, 0);

  ros::Time::setNow(ros::Time::now() + ros::Duration(2.0));

  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSequencer, compilation)
{
  TimeSequencer<Msg> seq(ros::Duration(1.0), ros::Duration(0.01), 10);
  TimeSequencer<Msg> seq2(ros::Duration(1.0), ros::Duration(0.01), 10);
  seq2.connectInput(seq);
}

struct EventHelper
{
public:
  void cb(const ros::MessageEvent<Msg const>& evt)
  {
    event_ = evt;
  }

  ros::MessageEvent<Msg const> event_;
};

TEST(TimeSequencer, eventInEventOut)
{
  TimeSequencer<Msg> seq(ros::Duration(1.0), ros::Duration(0.01), 10);
  TimeSequencer<Msg> seq2(seq, ros::Duration(1.0), ros::Duration(0.01), 10);
  EventHelper h;
  seq2.registerCallback(&EventHelper::cb, &h);

  ros::MessageEvent<Msg const> evt(MsgConstPtr(new Msg), ros::Time::now());
  seq.add(evt);

  ros::Time::setNow(ros::Time::now() + ros::Duration(2));
  while (!h.event_.getMessage())
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  EXPECT_EQ(h.event_.getReceiptTime(), evt.getReceiptTime());
  EXPECT_EQ(h.event_.getMessage(), evt.getMessage());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "time_sequencer_test");
  ros::NodeHandle nh;
  ros::Time::setNow(ros::Time());

  return RUN_ALL_TESTS();
}


