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

//#define FREE_LIST_DEBUG 1
#include "ros/rt/free_list.h"

#include <ros/ros.h>

#include <boost/thread.hpp>

using namespace ros::rt;

TEST(FreeList, oneElement)
{
  FreeList pool(4, 1);
  pool.constructAll<uint32_t>(5);

  uint32_t* item = static_cast<uint32_t*>(pool.allocate());
  ASSERT_TRUE(item);
  EXPECT_EQ(*item, 5UL);
  *item = 6;
  ASSERT_FALSE(pool.allocate());
  pool.free(item);
  item = static_cast<uint32_t*>(pool.allocate());
  ASSERT_TRUE(item);
  EXPECT_EQ(*item, 6UL);
}

TEST(FreeList, multipleElements)
{
  const uint32_t count = 5;
  FreeList pool(4, count);
  pool.constructAll<uint32_t>(5);

  std::vector<uint32_t*> items;
  for (uint32_t i = 0; i < count; ++i)
  {
    items.push_back(static_cast<uint32_t*>(pool.allocate()));
    ASSERT_TRUE(items[i]);
    EXPECT_EQ(*items[i], 5UL);
    *items[i] = i;
  }

  ASSERT_FALSE(pool.allocate());
  pool.free(items.back());
  items.pop_back();
  items.push_back(static_cast<uint32_t*>(pool.allocate()));
  ASSERT_TRUE(items.back());
  ASSERT_FALSE(pool.allocate());

  std::set<uint32_t*> set;
  set.insert(items.begin(), items.end());
  EXPECT_EQ(set.size(), count);
}

#if FREE_LIST_DEBUG
boost::mutex g_debug_mutex;
std::vector<FreeList::Debug> g_debug;

std::ostream& operator<<(std::ostream& o, const FreeList::Debug::Item& i)
{
  o.width(20);
  o << (i.success ? 1 : 0);
  o << " ";
  o << i.time;
  o << ": ";
  o << ((i.op == FreeList::Debug::Alloc) ? std::string("alloc") : std::string("free "));
  o << std::string("    head: ");
  o << std::hex;
  o << i.head;
  o << std::string("    new_head: ");
  o << i.new_head;
  o << "   addr:" << (uint32_t*)i.addr;
  return o;
}
#endif

struct PerfCounter
{
  PerfCounter()
  : start(ros::WallTime::now())
  {}

  double elapsed() { return (ros::WallTime::now() - start).toSec(); }
  void reset() { start = ros::WallTime::now(); }

  ros::WallTime start;
};

void threadFunc(FreeList& pool, ros::atomic<bool>& done, ros::atomic<bool>& failed, boost::barrier& b)
{
  b.wait();

  //ROS_INFO_STREAM("Thread " << boost::this_thread::get_id() << " starting");

  uint32_t* vals[10];
  uint64_t alloc_count = 0;
  while (!done.load())
  {
    for (uint32_t i = 0; i < 10; ++i)
    {
      vals[i] = static_cast<uint32_t*>(pool.allocate());

      if (vals[i])
      {
        ++alloc_count;
        *vals[i] = i;
      }
      else
      {
        ROS_ERROR_STREAM("Thread " << boost::this_thread::get_id() << " failed to allocate");
      }
    }

    for (uint32_t i = 0; i < 10; ++i)
    {
      if (vals[i])
      {
        if (*vals[i] != i)
        {
          ROS_ERROR_STREAM("Thread " << boost::this_thread::get_id() << " val " << vals[i] << " " << i << " = " << *vals[i]);
          failed.store(true);
        }

        pool.free(vals[i]);
      }
    }

    if (failed.load())
    {
#if FREE_LIST_DEBUG
      boost::mutex::scoped_lock lock(g_debug_mutex);
      g_debug.push_back(*pool.getDebug());
#endif
      return;
    }
  }

  //ROS_INFO_STREAM("Thread " << boost::this_thread::get_id() << " allocated " << alloc_count << " blocks");
}

TEST(FreeList, multipleThreads)
{
  FreeList pool(4, 100);
  ros::atomic<bool> done(false);
  ros::atomic<bool> failed(false);
  boost::thread_group tg;
  const uint32_t thread_count = boost::thread::hardware_concurrency();
  boost::barrier bar(thread_count);
  for (uint32_t i = 0; i < thread_count; ++i)
  {
    tg.create_thread(boost::bind(threadFunc, boost::ref(pool), boost::ref(done), boost::ref(failed), boost::ref(bar)));
  }

  ros::WallTime start = ros::WallTime::now();
  while (ros::WallTime::now() - start < ros::WallDuration(10.0))
  {
    ros::WallDuration(0.01).sleep();

    if (failed.load())
    {
      break;
    }
  }
  done.store(true);
  tg.join_all();

#if FREE_LIST_DEBUG
  if (failed.load())
  {
    std::vector<std::vector<FreeList::Debug::Item>::iterator> its;
    its.resize(g_debug.size());
    for (size_t i = 0; i < its.size(); ++i)
    {
      int32_t start = std::max((int32_t)g_debug[i].items.size() - 10000, 0);
      ROS_INFO_STREAM(start << " " << g_debug[i].items.size());
      its[i] = g_debug[i].items.begin() + start;
    }
    while (true)
    {
      size_t ind = 999999;
      ros::WallTime time(0xffffffff, 0);
      for (size_t j = 0; j < its.size(); ++j)
      {
        if (its[j] != g_debug[j].items.end() && its[j]->time < time)
        {
          ind = j;
          time = its[j]->time;
        }
      }

      if (ind == 999999)
      {
        break;
      }

      ROS_ERROR_STREAM("[" << std::setw(20) << g_debug[ind].thread << "] " << *its[ind]);
      ++its[ind];
    }
  }
#endif

  ASSERT_TRUE(!failed.load());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_freelist");
  return RUN_ALL_TESTS();
}

