/*
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */

/* Author: Tony Pratkanis */

/*
 * Subscribe to a topic multiple times
 */

#include <string>
#include <gtest/gtest.h>
#include <time.h>
#include <stdlib.h>
#include "ros/node.h"
#include <roslib/Time.h>

int g_argc;
char** g_argv;


class TimeReader : public ros::Node
{
  public:
    ros::Time getTime()
    {
      return ros::Time::now();
    }
    void setTime(ros::Time t)
    {
      roslib::Time message;
      message.rostime = t;
      publish("time", message);
    }
    void subCB(const ros::SingleSubscriberPublisher &)
    {
      has_subscriber_ = true;
    }
    TimeReader(std::string name) : ros::Node(name)
    {
      has_subscriber_ = false;
      roslib::Time msg;
      advertise("time", msg, &TimeReader::subCB, 1);
      while (!has_subscriber_) {
          struct timespec sleep_time = {0, 10000000};
          nanosleep(&sleep_time,NULL);
      }
    }
    bool has_subscriber_;
};


class RosTimeTest : public testing::Test
{
  public:
    TimeReader *n;

  protected:
    RosTimeTest()
    {
    }
    void SetUp()
    {
      ros::init(g_argc, g_argv);
      n = new TimeReader("timereader");
    }
    void TearDown()
    {

      delete n;
    }

};

TEST_F(RosTimeTest, RealTimeTest)
{
  //Get the start time.
  ros::Time start = n->getTime();

  //Checks to see if the time is larger than a thousand seconds
  //this is a good indication that we are getting the system time.
  ASSERT_TRUE(start.toSec() > 1000.0);

  //Wait a second
  ros::Duration wait(1, 0); wait.sleep();
  ros::Time end = n->getTime();
  ros::Duration d = end - start;

  //After waiting one second, see if we really waited on second.
  ASSERT_LT(d.toSec(), 1.1);
  ASSERT_GT(d.toSec(), 0.9);

  //Publish a rostime of 42.
  n->setTime(ros::Time(42, 0));

  //Wait half a second to get the message.
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));

  //Make sure that it is really set
  ASSERT_EQ(n->getTime().toSec(), 42.0);


  SUCCEED();
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}

