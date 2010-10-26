/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
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
#include <ros/init.h>
#include "message_filters/simple_filter.h"

using namespace message_filters;

struct Msg
{
};
typedef boost::shared_ptr<Msg> MsgPtr;
typedef boost::shared_ptr<Msg const> MsgConstPtr;

struct Filter : public SimpleFilter<Msg>
{
  typedef ros::MessageEvent<Msg const> EventType;

  void add(const EventType& evt)
  {
    signalMessage(evt);
  }
};

class Helper
{
public:
  Helper()
  {
    counts_.assign(0);
  }

  void cb0(const MsgConstPtr& msg)
  {
    ++counts_[0];
  }

  void cb1(const Msg& msg)
  {
    ++counts_[1];
  }

  void cb2(MsgConstPtr msg)
  {
    ++counts_[2];
  }

  void cb3(const ros::MessageEvent<Msg const>& evt)
  {
    ++counts_[3];
  }

  void cb4(Msg msg)
  {
    ++counts_[4];
  }

  void cb5(const MsgPtr& msg)
  {
    ++counts_[5];
  }

  void cb6(MsgPtr msg)
  {
    ++counts_[6];
  }

  void cb7(const ros::MessageEvent<Msg>& evt)
  {
    ++counts_[7];
  }

  boost::array<int32_t, 30> counts_;
};

TEST(SimpleFilter, callbackTypes)
{
  Helper h;
  Filter f;
  f.registerCallback(boost::bind(&Helper::cb0, &h, _1));
  f.registerCallback<const Msg&>(boost::bind(&Helper::cb1, &h, _1));
  f.registerCallback<MsgConstPtr>(boost::bind(&Helper::cb2, &h, _1));
  f.registerCallback<const ros::MessageEvent<Msg const>&>(boost::bind(&Helper::cb3, &h, _1));
  f.registerCallback<Msg>(boost::bind(&Helper::cb4, &h, _1));
  f.registerCallback<const MsgPtr&>(boost::bind(&Helper::cb5, &h, _1));
  f.registerCallback<MsgPtr>(boost::bind(&Helper::cb6, &h, _1));
  f.registerCallback<const ros::MessageEvent<Msg>&>(boost::bind(&Helper::cb7, &h, _1));

  f.add(Filter::EventType(MsgPtr(new Msg)));
  EXPECT_EQ(h.counts_[0], 1);
  EXPECT_EQ(h.counts_[1], 1);
  EXPECT_EQ(h.counts_[2], 1);
  EXPECT_EQ(h.counts_[3], 1);
  EXPECT_EQ(h.counts_[4], 1);
  EXPECT_EQ(h.counts_[5], 1);
  EXPECT_EQ(h.counts_[6], 1);
  EXPECT_EQ(h.counts_[7], 1);
}

struct OldFilter
{
  Connection registerCallback(const boost::function<void(const MsgConstPtr&)>& func)
  {
    return Connection();
  }
};

TEST(SimpleFilter, oldRegisterWithNewFilter)
{
  OldFilter f;
  Helper h;
  f.registerCallback(boost::bind(&Helper::cb3, &h, _1));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "blah");
  ros::Time::init();

  return RUN_ALL_TESTS();
}



