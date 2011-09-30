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
********************************************************************/

#include "ros/console.h"
#include <log4cxx/logger.h>

void print(ros::console::Level level, const std::string& s)
{
  ROS_LOG(level, ROSCONSOLE_DEFAULT_NAME, "%s", s.c_str());
}

int main(int /*argc*/, char** /*argv*/)
{
  // This needs to happen before we start fooling around with logger levels.  Otherwise the level we set may be overwritten by
  // a configuration file
  ROSCONSOLE_AUTOINIT;

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

  // Set the logger for this package to output all statements
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  // All these should print
  ROS_DEBUG("This is a debug statement, and should print");
  ROS_INFO("This is an info statement, and should print");
  ROS_WARN("This is a warn statement, and should print");
  ROS_ERROR("This is an error statement, and should print");
  ROS_FATAL("This is a fatal statement, and should print");

  // This should also print
  print(ros::console::levels::Debug, "Hello, this should print");

  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Error]);
  // This will STILL print, because the logger's enabled state has been cached
  print(ros::console::levels::Debug, "Hello, this will also print");

  // Calling notifyLoggerLevelsChanged() will force a reevaluation of which logging statements are enabled
  ros::console::notifyLoggerLevelsChanged();
  // Which will cause this to not print
  print(ros::console::levels::Debug, "Hello, this will NOT print");

  log4cxx::LoggerPtr ros_logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);

  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  ros_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Error]);
  ROS_DEBUG("This will still print, because the child logger's level overrides its ancestor loggers' levels");

  log4cxx::LoggerPtr test_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME".test");
  test_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Error]);
  ROS_INFO_NAMED("test", "This will not print, because the ros.rosconsole.test logger has been set to Error verbosity");
  test_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  ROS_INFO_NAMED("test", "Now everything sent to the ros.rosconsole.test logger will be printed (including this)");

  return 0;
}
