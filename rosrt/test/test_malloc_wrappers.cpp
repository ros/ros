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

#include "ros/rt/malloc_wrappers.h"
#include "ros/atomic.h"
#include <ros/time.h>
#include <ros/console.h>

#include <boost/thread.hpp>

using namespace ros::rt;
using namespace ros;

void allocateThread(const atomic<bool>& done)
{
  while (!done.load())
  {
    void* mem = malloc(500);
    free(mem);
    ros::WallDuration(0.001).sleep();
  }
}

TEST(MallocWrappers, statsMainThread)
{
  atomic<bool> done(false);
  boost::thread t(boost::bind(allocateThread, boost::ref(done)));

  initThreadAllocInfo();
  resetThreadAllocInfo();

  for (uint32_t i = 1; i <= 1000; ++i)
  {
    void* mem = malloc(500);
    free(mem);
    ros::WallDuration(0.001).sleep();

    const AllocInfo* info = getThreadAllocInfo();
    ASSERT_TRUE(info);

    ASSERT_EQ(info->mallocs, i);
    ASSERT_EQ(info->frees, i);
    ASSERT_EQ(info->total_ops, i * 2);
  }

  done.store(true);
  t.join();
}

void statsThread(atomic<bool>& failed)
{
  initThreadAllocInfo();
  resetThreadAllocInfo();

  for (uint32_t i = 1; i <= 1000; ++i)
  {
    void* mem = malloc(500);
    free(mem);
    ros::WallDuration(0.001).sleep();

    const AllocInfo* info = getThreadAllocInfo();
    if (!info)
    {
      ROS_ERROR("getThreadAllocInfo returned NULL");
      failed.store(true);
      return;
    }

    if (info->mallocs != i)
    {
      ROS_ERROR_STREAM("mallocs is " << info->mallocs << " should be " << i);
      failed.store(true);
      return;
    }

    if (info->frees != i)
    {
      ROS_ERROR_STREAM("mallocs is " << info->frees << " should be " << i);
      failed.store(true);
      return;
    }
  }
}

TEST(MallocWrappers, statsNewThread)
{
  atomic<bool> failed(false);
  boost::thread t(boost::bind(statsThread, boost::ref(failed)));
  t.join();

  ASSERT_FALSE(failed.load());
}

void doBreakOnMalloc()
{
  setThreadBreakOnAllocOrFree(true);
  void* mem = malloc(500);
  mem = 0;
  setThreadBreakOnAllocOrFree(false);
}

TEST(MallocWrappersDeathTest, breakOnAllocFree)
{
  initThreadAllocInfo();
  resetThreadAllocInfo();

  // TODO: Re-enable once ROS 1.1 goes out with the updated version of gtest
  //ASSERT_DEATH_IF_SUPPORTED(doBreakOnMalloc(), "Issuing break due to break_on_alloc_or_free being set");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  return RUN_ALL_TESTS();
}

