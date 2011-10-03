/*
 * Copyright (c) 2NULLNULL8, Willow Garage, Inc.
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
 * Test timer callbacks
 */

#include <string>
#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <test_roscpp/TestArray.h>
#include <test_roscpp/TestStringString.h>

#include <boost/scoped_ptr.hpp>

using namespace ros;
using namespace test_roscpp;

std::string g_node_name = "test_timer_callbacks";

class WallTimerHelper
{
public:
  WallTimerHelper(float period, bool oneshot = false)
  : expected_period_(period)
  , failed_(false)
  , total_calls_(0)
  {
    NodeHandle n;
    timer_ = n.createWallTimer(expected_period_, &WallTimerHelper::callback, this, oneshot);
  }

  void callback(const WallTimerEvent& e)
  {
    bool first = last_call_.isZero();
    WallTime last_call = last_call_;
    last_call_ = WallTime::now();
    WallTime start = last_call_;

    if (!first)
    {
      if (fabsf(expected_next_call_.toSec() - start.toSec()) > 0.1f)
      {
        ROS_ERROR("Call came at wrong time (%f vs. %f)", expected_next_call_.toSec(), start.toSec());
        failed_ = true;
      }
    }

    expected_next_call_ = e.current_expected + expected_period_;

    WallTime end = WallTime::now();
    last_duration_ = end - start;

    ++total_calls_;
  }

  WallTime last_call_;
  WallTime expected_next_call_;
  WallDuration expected_period_;
  WallDuration last_duration_;

  bool failed_;

  WallTimer timer_;
  int32_t total_calls_;
};

TEST(RoscppTimerCallbacks, singleWallTimeCallback)
{
  NodeHandle n;
  WallTimerHelper helper1(0.01);

  WallDuration d(0.001f);
  for (int32_t i = 0; i < 1000 && n.ok(); ++i)
  {
    spinOnce();
    d.sleep();
  }

  if (helper1.failed_)
  {
    FAIL();
  }

  if (helper1.total_calls_ < 99)
  {
    ROS_ERROR("Total calls: %d (expected at least 100)", helper1.total_calls_);
    FAIL();
  }
}

TEST(RoscppTimerCallbacks, multipleWallTimeCallbacks)
{
  NodeHandle n;
  const int count = 100;
  typedef boost::scoped_ptr<WallTimerHelper> HelperPtr;
  HelperPtr helpers[count];
  for (int i = 0; i < count; ++i)
  {
    helpers[i].reset(new WallTimerHelper((float)(i + 1) * 0.1f));
  }

  WallDuration d(0.01f);
  const int spin_count = 1000;
  for (int32_t i = 0; i < spin_count && n.ok(); ++i)
  {
    spinOnce();
    d.sleep();
  }

  for (int i = 0; i < count; ++i)
  {
    if (helpers[i]->failed_)
    {
      ROS_ERROR("Helper %d failed", i);
      FAIL();
    }

    int32_t expected_count = (spin_count * d.toSec()) / helpers[i]->expected_period_.toSec();
    if (helpers[i]->total_calls_ < (expected_count - 1))
    {
      ROS_ERROR("Helper %d total calls: %d (at least %d expected)", i, helpers[i]->total_calls_, expected_count);
      FAIL();
    }
  }
}

TEST(RoscppTimerCallbacks, stopWallTimer)
{
  NodeHandle n;
  WallTimerHelper helper(0.001);

  for (int32_t i = 0; i < 1000 && n.ok(); ++i)
  {
    WallDuration(0.001).sleep();
    spinOnce();
  }

  ASSERT_GT(helper.total_calls_, 0);
  int32_t last_count = helper.total_calls_;
  helper.timer_.stop();

  for (int32_t i = 0; i < 1000 && n.ok(); ++i)
  {
    WallDuration(0.001).sleep();
    spinOnce();
  }

  ASSERT_EQ(last_count, helper.total_calls_);
}

int32_t g_count = 0;
void timerCallback(const ros::WallTimerEvent& evt)
{
  ++g_count;
}

TEST(RoscppTimerCallbacks, stopThenSpin)
{
  g_count = 0;
  NodeHandle n;
  ros::WallTimer timer = n.createWallTimer(ros::WallDuration(0.001), timerCallback);

  WallDuration(0.1).sleep();
  timer.stop();

  spinOnce();

  ASSERT_EQ(g_count, 0);
}

TEST(RoscppTimerCallbacks, oneShotWallTimer)
{
  NodeHandle n;
  WallTimerHelper helper(0.001, true);

  for (int32_t i = 0; i < 1000 && n.ok(); ++i)
  {
    WallDuration(0.001).sleep();
    spinOnce();
  }

  ASSERT_EQ(helper.total_calls_, 1);
}

class TimerHelper
{
public:
  TimerHelper(Duration period, bool oneshot = false)
    : failed_(false)
    , expected_period_(period)
    , total_calls_(0)
  {
    NodeHandle n;
    timer_ = n.createTimer(expected_period_, &TimerHelper::callback, this, oneshot);
  }

  TimerHelper(Rate r, bool oneshot = false)
    : failed_(false)
    , expected_period_(r.expectedCycleTime())
    , total_calls_(0)
  {
    NodeHandle n;
    timer_ = n.createTimer(r, &TimerHelper::callback, this, oneshot);
  }

  void callback(const TimerEvent& e)
  {
    ++total_calls_;
  }

  bool failed_;

  Duration expected_period_;

  Timer timer_;
  int32_t total_calls_;
};

TEST(RoscppTimerCallbacks, singleROSTimeCallback)
{
  NodeHandle n;

  Time now(1, 0);
  Time::setNow(now);

  TimerHelper helper(Duration(0, 10000000));

  Duration d(0, 1000000);
  for (int32_t i = 0; i < 1000 && n.ok(); ++i)
  {
    now += d;
    Time::setNow(now);

    while (helper.timer_.hasPending())
    {
      WallDuration(0.001).sleep();
      spinOnce();
    }
  }

  if (helper.failed_)
  {
    FAIL();
  }

  if (helper.total_calls_ != 100)
  {
    ROS_ERROR("Total calls: %d (expected 100)", helper.total_calls_);
    FAIL();
  }
}

TEST(RoscppTimerCallbacks, singleROSTimeCallbackFromRate)
{
  NodeHandle n;

  Time now(1, 0);
  Time::setNow(now);

  TimerHelper helper(Rate(100));

  Duration d(0, 1000000);
  for (int32_t i = 0; i < 1000 && n.ok(); ++i)
  {
    now += d;
    Time::setNow(now);

    while (helper.timer_.hasPending())
    {
      WallDuration(0.00025).sleep();
      spinOnce();
    }
  }

  if (helper.failed_)
  {
    FAIL();
  }

  if (helper.total_calls_ != 100)
  {
    ROS_ERROR("Total calls: %d (expected 100)", helper.total_calls_);
    FAIL();
  }
}

TEST(RoscppTimerCallbacks, oneshotROSTimer)
{
  NodeHandle n;

  Time now(1, 0);
  Time::setNow(now);

  TimerHelper helper(Duration(0, 10000000), true);

  Duration d(0, 1000000);
  for (int32_t i = 0; i < 1000 && n.ok(); ++i)
  {
    now += d;
    Time::setNow(now);

    while (helper.timer_.hasPending())
    {
      WallDuration(0.001).sleep();
      spinOnce();
    }
  }

  if (helper.failed_)
  {
    FAIL();
  }

  ASSERT_EQ(helper.total_calls_, 1);
}

TEST(RoscppTimerCallbacks, singleROSTimeCallbackLargeTimestep)
{
  NodeHandle n;

  Time now(1, 0);
  Time::setNow(now);

  TimerHelper helper(Duration(0, 10000000));

  Duration d(0, 100000000);
  for (int32_t i = 0; i < 100 && n.ok(); ++i)
  {
    now += d;
    Time::setNow(now);

    while (helper.timer_.hasPending())
    {
      WallDuration(0.001).sleep();
      spinOnce();
    }
  }

  if (helper.failed_)
  {
    FAIL();
  }

  if (helper.total_calls_ != 200)
  {
    ROS_ERROR("Total calls: %d (expected 200)", helper.total_calls_);
    FAIL();
  }
}

TEST(RoscppTimerCallbacks, multipleROSTimeCallbacks)
{
  NodeHandle n;

  Time now(1, 0);
  Time::setNow(now);

  const int count = 100;
  typedef boost::scoped_ptr<TimerHelper> HelperPtr;
  HelperPtr helpers[count];
  for (int i = 0; i < count; ++i)
  {
    helpers[i].reset(new TimerHelper(Duration((float)(i + 1) * 0.01f)));
  }

  Duration d(0.001f);
  const int spin_count = 1000;
  for (int32_t i = 0; i < spin_count && n.ok(); ++i)
  {
    now += d;
    Time::setNow(now);

    bool pending = false;

    do
    {
      pending = false;
      for (int i = 0; i < count; ++i)
      {
        pending |= helpers[i]->timer_.hasPending();
      }

      WallDuration(0.001).sleep();
      spinOnce();
    } while (pending);
  }

  for (int i = 0; i < count; ++i)
  {
    if (helpers[i]->failed_)
    {
      ROS_ERROR("Helper %d failed", i);
      FAIL();
    }

    int32_t expected_count = (spin_count * d.toSec()) / helpers[i]->expected_period_.toSec();
    if (helpers[i]->total_calls_ < (expected_count - 1) || helpers[i]->total_calls_ > (expected_count + 1))
    {
      ROS_ERROR("Helper %d total calls: %d (%d expected)", i, helpers[i]->total_calls_, expected_count);
      FAIL();
    }
  }
}

class Tracked
{
public:
  Tracked()
  {
    g_count = 0;
  }

  void callback(const TimerEvent& e)
  {
    ++g_count;
  }
};

TEST(RoscppTimerCallbacks, trackedObject)
{
  NodeHandle n;
  Time now(1, 0);
  Time::setNow(now);

  boost::shared_ptr<Tracked> tracked(new Tracked);
  Timer timer = n.createTimer(Duration(0.001), &Tracked::callback, tracked);

  now += Duration(0.1);
  Time::setNow(now);

  while (timer.hasPending())
  {
    WallDuration(0.001).sleep();
    spinOnce();
  }

  ASSERT_GT(g_count, 0);
  int32_t last_count = g_count;
  tracked.reset();

  now += Duration(0.1);
  Time::setNow(now);

  while (timer.hasPending())
  {
    WallDuration(0.001).sleep();
    spinOnce();
  }

  ASSERT_EQ(last_count, g_count);
}

TEST(RoscppTimerCallbacks, stopROSTimer)
{
  NodeHandle n;
  Time now(1, 0);
  Time::setNow(now);

  TimerHelper helper(Duration(0.001));

  now += Duration(0.1);
  Time::setNow(now);

  while (helper.timer_.hasPending())
  {
    WallDuration(0.001).sleep();
    spinOnce();
  }

  ASSERT_GT(helper.total_calls_, 0);
  int32_t last_count = helper.total_calls_;
  helper.timer_.stop();

  now += Duration(0.1);
  Time::setNow(now);

  while (helper.timer_.hasPending())
  {
    WallDuration(0.001).sleep();
    spinOnce();
  }

  ASSERT_EQ(last_count, helper.total_calls_);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, g_node_name);

  return RUN_ALL_TESTS();
}

