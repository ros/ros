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
#include "message_filters/time_synchronizer.h"

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

class Helper
{
public:
  Helper()
  : count_(0)
  {}

  void cb()
  {
    ++count_;
  }

  int32_t count_;
};

TEST(TimeSynchronizer, compile2)
{
  NullFilter<Msg> f0, f1;
  TimeSynchronizer<Msg, Msg> sync(f0, f1, 1);
}

TEST(TimeSynchronizer, compile3)
{
  NullFilter<Msg> f0, f1, f2;
  TimeSynchronizer<Msg, Msg, Msg> sync(f0, f1, f2, 1);
}

TEST(TimeSynchronizer, compile4)
{
  NullFilter<Msg> f0, f1, f2, f3;
  TimeSynchronizer<Msg, Msg, Msg, Msg> sync(f0, f1, f2, f3, 1);
}

TEST(TimeSynchronizer, compile5)
{
  NullFilter<Msg> f0, f1, f2, f3, f4;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg> sync(f0, f1, f2, f3, f4, 1);
}

TEST(TimeSynchronizer, compile6)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg> sync(f0, f1, f2, f3, f4, f5, 1);
}

TEST(TimeSynchronizer, compile7)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5, f6;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(f0, f1, f2, f3, f4, f5, f6, 1);
}

TEST(TimeSynchronizer, compile8)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5, f6, f7;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(f0, f1, f2, f3, f4, f5, f6, f7, 1);
}

TEST(TimeSynchronizer, compile9)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5, f6, f7, f8;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(f0, f1, f2, f3, f4, f5, f6, f7, f8, 1);
}

void function2(const MsgConstPtr&, const MsgConstPtr&) {}
void function3(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function4(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function5(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function6(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function7(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function8(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function9(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}

TEST(TimeSynchronizer, compileFunction2)
{
  TimeSynchronizer<Msg, Msg> sync(1);
  sync.registerCallback(function2);
}

TEST(TimeSynchronizer, compileFunction3)
{
  TimeSynchronizer<Msg, Msg, Msg> sync(1);
  sync.registerCallback(function3);
}

TEST(TimeSynchronizer, compileFunction4)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(function4);
}

TEST(TimeSynchronizer, compileFunction5)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(function5);
}

TEST(TimeSynchronizer, compileFunction6)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(function6);
}

TEST(TimeSynchronizer, compileFunction7)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(function7);
}

TEST(TimeSynchronizer, compileFunction8)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(function8);
}

TEST(TimeSynchronizer, compileFunction9)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(function9);
}

TEST(TimeSynchronizer, immediate2)
{
  TimeSynchronizer<Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  MsgPtr m(new Msg);
  m->header.stamp = ros::Time::now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate3)
{
  TimeSynchronizer<Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  MsgPtr m(new Msg);
  m->header.stamp = ros::Time::now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate4)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  MsgPtr m(new Msg);
  m->header.stamp = ros::Time::now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
  sync.add3(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate5)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  MsgPtr m(new Msg);
  m->header.stamp = ros::Time::now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
  sync.add3(m);
  ASSERT_EQ(h.count_, 0);
  sync.add4(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate6)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  MsgPtr m(new Msg);
  m->header.stamp = ros::Time::now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
  sync.add3(m);
  ASSERT_EQ(h.count_, 0);
  sync.add4(m);
  ASSERT_EQ(h.count_, 0);
  sync.add5(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate7)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  MsgPtr m(new Msg);
  m->header.stamp = ros::Time::now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
  sync.add3(m);
  ASSERT_EQ(h.count_, 0);
  sync.add4(m);
  ASSERT_EQ(h.count_, 0);
  sync.add5(m);
  ASSERT_EQ(h.count_, 0);
  sync.add6(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate8)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  MsgPtr m(new Msg);
  m->header.stamp = ros::Time::now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
  sync.add3(m);
  ASSERT_EQ(h.count_, 0);
  sync.add4(m);
  ASSERT_EQ(h.count_, 0);
  sync.add5(m);
  ASSERT_EQ(h.count_, 0);
  sync.add6(m);
  ASSERT_EQ(h.count_, 0);
  sync.add7(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate9)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  MsgPtr m(new Msg);
  m->header.stamp = ros::Time::now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
  sync.add3(m);
  ASSERT_EQ(h.count_, 0);
  sync.add4(m);
  ASSERT_EQ(h.count_, 0);
  sync.add5(m);
  ASSERT_EQ(h.count_, 0);
  sync.add6(m);
  ASSERT_EQ(h.count_, 0);
  sync.add7(m);
  ASSERT_EQ(h.count_, 0);
  sync.add8(m);
  ASSERT_EQ(h.count_, 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// From here on we assume that testing the 3-message version is sufficient, so as not to duplicate
// tests for everywhere from 2-9
//////////////////////////////////////////////////////////////////////////////////////////////////
TEST(TimeSynchronizer, multipleTimes)
{
  TimeSynchronizer<Msg, Msg, Msg> sync(2);
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  MsgPtr m(new Msg);
  m->header.stamp = ros::Time();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);

  m.reset(new Msg);
  m->header.stamp = ros::Time(0.1);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, queueSize)
{
  TimeSynchronizer<Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  MsgPtr m(new Msg);
  m->header.stamp = ros::Time();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);

  m.reset(new Msg);
  m->header.stamp = ros::Time(0.1);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);

  m.reset(new Msg);
  m->header.stamp = ros::Time(0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  ros::Time::setNow(ros::Time());

  return RUN_ALL_TESTS();
}


