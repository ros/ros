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
#include <ros/init.h>
#include "message_filters/cache.h"

using namespace std ;
using namespace message_filters ;

struct Header
{
  ros::Time stamp ;
} ;


struct Msg
{
  Header header ;
  int data ;
} ;
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


void fillCacheEasy(Cache<Msg>& cache, unsigned int start, unsigned int end)

{
  for (unsigned int i=start; i < end; i++)
  {
    Msg* msg = new Msg ;
    msg->data = i ;
    msg->header.stamp.fromSec(i*10) ;

    boost::shared_ptr<Msg const> msg_ptr(msg) ;
    cache.add(msg_ptr) ;
  }
}

TEST(Cache, easyInterval)
{
  Cache<Msg> cache(10) ;
  fillCacheEasy(cache, 0, 5) ;

  vector<boost::shared_ptr<Msg const> > interval_data = cache.getInterval(ros::Time().fromSec(5), ros::Time().fromSec(35)) ;

  ASSERT_EQ(interval_data.size(), (unsigned int) 3) ;
  EXPECT_EQ(interval_data[0]->data, 1) ;
  EXPECT_EQ(interval_data[1]->data, 2) ;
  EXPECT_EQ(interval_data[2]->data, 3) ;

  // Look for an interval past the end of the cache
  interval_data = cache.getInterval(ros::Time().fromSec(55), ros::Time().fromSec(65)) ;
  EXPECT_EQ(interval_data.size(), (unsigned int) 0) ;

  // Look for an interval that fell off the back of the cache
  fillCacheEasy(cache, 5, 20) ;
  interval_data = cache.getInterval(ros::Time().fromSec(5), ros::Time().fromSec(35)) ;
  EXPECT_EQ(interval_data.size(), (unsigned int) 0) ;
}

TEST(Cache, easySurroundingInterval)
{
  Cache<Msg> cache(10);
  fillCacheEasy(cache, 1, 6);

  vector<boost::shared_ptr<Msg const> > interval_data;
  interval_data = cache.getSurroundingInterval(ros::Time(15,0), ros::Time(35,0)) ;
  ASSERT_EQ(interval_data.size(), (unsigned int) 4);
  EXPECT_EQ(interval_data[0]->data, 1);
  EXPECT_EQ(interval_data[1]->data, 2);
  EXPECT_EQ(interval_data[2]->data, 3);
  EXPECT_EQ(interval_data[3]->data, 4);

  interval_data = cache.getSurroundingInterval(ros::Time(0,0), ros::Time(35,0)) ;
  ASSERT_EQ(interval_data.size(), (unsigned int) 4);
  EXPECT_EQ(interval_data[0]->data, 1);

  interval_data = cache.getSurroundingInterval(ros::Time(35,0), ros::Time(35,0)) ;
  ASSERT_EQ(interval_data.size(), (unsigned int) 2);
  EXPECT_EQ(interval_data[0]->data, 3);
  EXPECT_EQ(interval_data[1]->data, 4);

  interval_data = cache.getSurroundingInterval(ros::Time(55,0), ros::Time(55,0)) ;
  ASSERT_EQ(interval_data.size(), (unsigned int) 1);
  EXPECT_EQ(interval_data[0]->data, 5);
}


boost::shared_ptr<Msg const> buildMsg(double time, int data)
{
  Msg* msg = new Msg ;
  msg->data = data ;
  msg->header.stamp.fromSec(time) ;

  boost::shared_ptr<Msg const> msg_ptr(msg) ;
  return msg_ptr ;
}

TEST(Cache, easyUnsorted)
{
  Cache<Msg> cache(10) ;

  cache.add(buildMsg(10.0, 1)) ;
  cache.add(buildMsg(30.0, 3)) ;
  cache.add(buildMsg(70.0, 7)) ;
  cache.add(buildMsg( 5.0, 0)) ;
  cache.add(buildMsg(20.0, 2)) ;

  vector<boost::shared_ptr<Msg const> > interval_data = cache.getInterval(ros::Time().fromSec(3), ros::Time().fromSec(15)) ;

  ASSERT_EQ(interval_data.size(), (unsigned int) 2) ;
  EXPECT_EQ(interval_data[0]->data, 0) ;
  EXPECT_EQ(interval_data[1]->data, 1) ;

  // Grab all the data
  interval_data = cache.getInterval(ros::Time().fromSec(0), ros::Time().fromSec(80)) ;
  ASSERT_EQ(interval_data.size(), (unsigned int) 5) ;
  EXPECT_EQ(interval_data[0]->data, 0) ;
  EXPECT_EQ(interval_data[1]->data, 1) ;
  EXPECT_EQ(interval_data[2]->data, 2) ;
  EXPECT_EQ(interval_data[3]->data, 3) ;
  EXPECT_EQ(interval_data[4]->data, 7) ;
}


TEST(Cache, easyElemBeforeAfter)
{
  Cache<Msg> cache(10) ;
  boost::shared_ptr<Msg const> elem_ptr ;

  fillCacheEasy(cache, 5, 10) ;

  elem_ptr = cache.getElemAfterTime( ros::Time().fromSec(85.0)) ;

  ASSERT_FALSE(!elem_ptr) ;
  EXPECT_EQ(elem_ptr->data, 9) ;

  elem_ptr = cache.getElemBeforeTime( ros::Time().fromSec(85.0)) ;
  ASSERT_FALSE(!elem_ptr) ;
  EXPECT_EQ(elem_ptr->data, 8) ;

  elem_ptr = cache.getElemBeforeTime( ros::Time().fromSec(45.0)) ;
  EXPECT_TRUE(!elem_ptr) ;
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

TEST(Cache, eventInEventOut)
{
  Cache<Msg> c0(10);
  Cache<Msg> c1(c0, 10);
  EventHelper h;
  c1.registerCallback(&EventHelper::cb, &h);

  ros::MessageEvent<Msg const> evt(MsgConstPtr(new Msg), ros::Time(4));
  c0.add(evt);

  EXPECT_EQ(h.event_.getReceiptTime(), evt.getReceiptTime());
  EXPECT_EQ(h.event_.getMessage(), evt.getMessage());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "blah");
  ros::Time::init();
  return RUN_ALL_TESTS();
}

