/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Brian Gerkey */

#include <stdexcept> // for std::runtime_error
#include <string>
#include <vector>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <signal.h>
#include <gtest/gtest.h>
#include <boost/thread/thread.hpp>
#include "rospack/rospack.h"

/////////////////////////////////////////////////////////////
// Code-level tests against rospack's snarf_flags() method
TEST(rospack, lflags_embedded_newline)
{
  rospack::ROSPack rp;
  std::string input1("-lloki \n-lfoo");
  std::string output1("loki foo");

  std::string lflags;

  lflags = rp.snarf_libs(input1);
  EXPECT_EQ(output1, lflags);
}

TEST(rospack, lflags_escaped_string)
{
  rospack::ROSPack rp;
  std::string input1("-lloki \\-lfoo");
  std::string output1("loki");

  std::string lflags;

  lflags = rp.snarf_libs(input1);
  EXPECT_EQ(output1, lflags);
}

TEST(rospack, lflags_embedded_whitespace)
{
  rospack::ROSPack rp;
  std::string input1("-lloki -lfoo\\ bar");
  std::string output1("loki foo\\ bar");

  std::string lflags;

  lflags = rp.snarf_libs(input1);
  EXPECT_EQ(output1, lflags);
}

TEST(rospack, lflags_no_trailing_whitespace)
{
  rospack::ROSPack rp;
  std::string input1("-lloki");
  std::string input2("-lloki -lfoo");
  std::string output1("loki");
  std::string output2("loki foo");

  std::string lflags;

  lflags = rp.snarf_libs(input1);
  EXPECT_EQ(output1, lflags);

  lflags = rp.snarf_libs(input2);
  EXPECT_EQ(output2, lflags);
}

TEST(rospack, lflags_embedded_dashl)
{
  rospack::ROSPack rp;
  std::string input("-L/u/wheeler/ros/ros-pkg/3rdparty/loki-lib/loki-svn/lib -lloki ");
  std::string lib_output("loki");
  std::string libdir_output("/u/wheeler/ros/ros-pkg/3rdparty/loki-lib/loki-svn/lib");

  std::string lflags;

  lflags = rp.snarf_flags(input, "-L");
  EXPECT_EQ(libdir_output, lflags);

  lflags = rp.snarf_libs(input);
  EXPECT_EQ(lib_output, lflags);
}

TEST(rospack, lflags_static_libs)
{
  rospack::ROSPack rp;
  std::string input("-L/blah/blah -lfoo /bar/baz/bom.a -lzom -Wl,other");
  std::string lib_output("foo /bar/baz/bom.a zom");
  std::string libdir_output("/blah/blah");
  std::string other_output("-Wl,other");

  std::string lflags;

  lflags = rp.snarf_flags(input, "-L");
  EXPECT_EQ(libdir_output, lflags);

  lflags = rp.snarf_libs(input);
  EXPECT_EQ(lib_output, lflags);

  lflags = rp.snarf_libs(input, true);
  lflags = rp.snarf_flags(lflags, "-L", true);
  EXPECT_EQ(other_output, lflags);
}

void handler(int sig)
{
}

void signaler()
{
  struct timespec ts = {0, 100000};
  for(int i=0; i<20000; i++)
  {
    nanosleep(&ts, NULL);
    raise(SIGALRM);
  }
}

TEST(rospack, signal_interruption)
{
  char buf[1024];
  std::string rr = std::string(getcwd(buf, sizeof(buf))) + "/test2";
  setenv("ROS_ROOT", rr.c_str(), 1);
  unsetenv("ROS_PACKAGE_PATH");

  signal(SIGALRM, handler);
  boost::thread t(boost::bind(signaler));
  rospack::ROSPack rp;
  int ret = rp.run(std::string("list-names"));
  t.join();
  ASSERT_EQ(ret, 0);
  std::vector<std::string> output_list;
  rospack::string_split(rp.getOutput(), output_list, "\n");
  ASSERT_EQ((int)output_list.size(), 4);
}

// Code-level tests against rospack's snarf_flags() method
/////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
