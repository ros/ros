/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Troy Straszheim */

/*
 * Spinny spinner basic tests.
 
 these do NOT ACTUALLY TEST ANYTHING as it is diffcult to restart all
 of ros in a unit test, and there is no way to test (that I can see
 right now) that certain error messages are emitted, nor any way to
 specify that a test should fail or return an error.  So this is just
 a placeholder to be run manually, one test at a time (via
 --gtest_filter) when the next problem occurs.  Those that end with
 'fail' actually success, but they send a fatal error message that
 you're trying to spin from multiple threads.
*/

#include <gtest/gtest.h>
#include "ros/spinner.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include <boost/thread.hpp>

using namespace ros;

int argc_;
char** argv_;

void fire_shutdown(const ros::WallTimerEvent& event) {
  ROS_INFO("Asking for shutdown");
  ros::shutdown();
}

#define DOIT()                                                  \
  ros::init(argc_, argv_, "test_spinners");                     \
  NodeHandle nh;                                                \
  ros::WallTimer t = nh.createWallTimer(ros::WallDuration(2.0), \
                                        &fire_shutdown);        \
  
TEST(Spinners, spin)
{
  DOIT();
  ros::spin();
}

TEST(Spinners, spinfail)
{
  DOIT();
  boost::thread th(boost::bind(&ros::spin));
  ros::spin();
}

TEST(Spinners, single)
{
  DOIT();
  SingleThreadedSpinner s;
  ros::spin(s);
}

TEST(Spinners, singlefail)
{
  DOIT();
  boost::thread th(boost::bind(&ros::spin));
  SingleThreadedSpinner s;
  ros::spin(s);
}

TEST(Spinners, singlefail2)
{
  DOIT();
  SingleThreadedSpinner s;
  boost::thread th(boost::bind(&ros::spin, s));
  ros::spin(s);
}

TEST(Spinners, multi)
{
  DOIT();
  MultiThreadedSpinner s;
  ros::spin(s);
}

TEST(Spinners, multifail)
{
  DOIT();
  boost::thread th(boost::bind(&ros::spin));
  MultiThreadedSpinner s;
  ros::spin(s);
}

TEST(Spinners, multifail2)
{
  DOIT();
  MultiThreadedSpinner s;
  boost::thread th(boost::bind(&ros::spin, s));
  ros::spin(s);
}

TEST(Spinners, async)
{
  DOIT();
  AsyncSpinner s(2);
  s.start();
  ros::waitForShutdown();
}

TEST(Spinners, asyncfail)
{
  DOIT();
  boost::thread th(boost::bind(&ros::spin));
  AsyncSpinner s(2);
  s.start();
  ros::waitForShutdown();
}


int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  argc_ = argc;
  argv_ = argv;
  return RUN_ALL_TESTS();
}


