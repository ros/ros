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

#include <string>
#include <vector>
#include <stdlib.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <gtest/gtest.h>
#include "ros/package.h"

#ifdef _WIN32
int setenv(const char *name, const char *value, int overwrite)
{
  if(!overwrite)
  {
    size_t envsize = 0;
    errno_t errcode = getenv_s(&envsize, NULL, 0, name);
    if(errcode || envsize)
      return errcode;
  }
  return _putenv_s(name, value);
}
#endif

void string_split(const std::string &s, std::vector<std::string> &t, const std::string &d)
{                                                                                 t.clear();
  size_t start = 0, end;
  while ((end = s.find_first_of(d, start)) != std::string::npos)
  {     
    if((end-start) > 0)
      t.push_back(s.substr(start, end-start));
    start = end + 1;
  }   
  if(start < s.size())
    t.push_back(s.substr(start));
}

std::string g_rr_path;
void set_env_vars(void)
{
  // Point ROS_PACKAGE_PATH at the roslib directory, and point
  // ROS_ROOT into an empty directory.
  char rr_buf[1024];
  getcwd(rr_buf, sizeof(rr_buf));
  setenv("ROS_PACKAGE_PATH", rr_buf, 1);

  boost::filesystem::path temp_path = boost::filesystem::unique_path();
  boost::filesystem::create_directories(temp_path);
  g_rr_path = temp_path.string();
  setenv("ROS_ROOT", g_rr_path.c_str(), 1);
}
void cleanup_env_vars(void)
{
  // Remove the empty directory that we created in set_env_vars().
  rmdir(g_rr_path.c_str());
  g_rr_path.clear();
}

TEST(roslib, commandListNames)
{
  set_env_vars();

  std::string output = ros::package::command("list-names");
  std::vector<std::string> output_list;
  string_split(output, output_list, "\n");
  ASSERT_EQ((int)output_list.size(), 1);
  ASSERT_STREQ(output_list[0].c_str(), "roslib");

  cleanup_env_vars();
}

TEST(roslib, commandList)
{
  set_env_vars();

  std::string output = ros::package::command("list");
  std::vector<std::string> output_list;
  std::vector<std::string> name_path;
  string_split(output, output_list, "\n");
  ASSERT_EQ((int)output_list.size(), 1);
  string_split(output_list[0], name_path, " ");
  ASSERT_EQ((int)name_path.size(), 2);
  ASSERT_STREQ(name_path[0].c_str(), "roslib");

  cleanup_env_vars();
}

TEST(roslib, getAll)
{
  set_env_vars();

  std::vector<std::string> output_list;
  ASSERT_TRUE(ros::package::getAll(output_list));
  ASSERT_EQ((int)output_list.size(), 1);
  ASSERT_STREQ(output_list[0].c_str(), "roslib");

  cleanup_env_vars();
}

void
roslib_caller()
{
  struct timespec ts = {0, 100000};
  std::string output;
  for(int i=0;i<100;i++)
  {
    output = ros::package::command("plugins --attrib=foo roslib");
#ifndef _WIN32
    nanosleep(&ts, NULL);
#else
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
#endif
  }
}

TEST(roslib, concurrent_access)
{
  set_env_vars();
  const int size = 10;
  boost::thread t[size];
  for(int i=0;i<size; i++)
    t[i] = boost::thread(boost::bind(roslib_caller));
  for(int i=0;i<size; i++)
    t[i].join();
  cleanup_env_vars();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

