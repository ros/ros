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

/* Author: Josh Faust */

/*
 * Test logging functionality
 */

#include <string>
#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "ros/node.h"
#include <test_roscpp/TestArray.h>

ros::Node* g_node;
const char* g_node_name = "logTest";

TEST(roscpp, logToFile)
{
  const std::string log_string = "Testing 1 2 3";

  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  log4cxx::LevelPtr old_level = logger->getLevel();

  logger->setLevel(log4cxx::Level::getInfo());
  ROS_INFO("%s", log_string.c_str());
  logger->setLevel(old_level);

  // Open the log file, read, and try to find the log string
  std::string log_file = g_node->getLogFilePath();
  std::ifstream ifs( log_file.c_str() );

  ASSERT_TRUE( ifs.is_open() );

  bool found = false;

  while ( !ifs.eof() )
  {
    char line_cstr[2048];
    ifs.getline( line_cstr, 2048 );

    std::string line = line_cstr;
    if ( line.find( log_string ) != std::string::npos )
    {
      found = true;
      break;
    }
  }

  ASSERT_TRUE( found );
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init( argc, argv );

  g_node = new ros::Node( g_node_name );

  int ret = RUN_ALL_TESTS();


  delete g_node;

  return ret;
}
