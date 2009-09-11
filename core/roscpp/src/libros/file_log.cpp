/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#include <cstdio>
#include "ros/file_log.h"
#include "ros/this_node.h"

#include "log4cxx/rollingfileappender.h"
#include "log4cxx/patternlayout.h"

#include <ros/console.h>

namespace ros
{

namespace file_log
{

std::string g_log_filename;

const std::string& getLogFilename()
{
  return g_log_filename;
}

void init(const M_string& remappings)
{
  std::string log_file_name;
  M_string::const_iterator it = remappings.find("__log");
  if (it != remappings.end())
  {
    log_file_name = it->second;
  }

  {
    // Log filename can be specified on the command line through __log
    // If it's been set, don't create our own name
    if (log_file_name.empty())
    {
      // Setup the logfile appender
      // Can't do this in rosconsole because the node name is not known
      pid_t pid = getpid();
      char* ros_log_env = getenv("ROS_LOG_DIR");
      if (ros_log_env)
      {
        log_file_name = ros_log_env + std::string("/");
      }
      else
      {
        ros_log_env = getenv("ROS_ROOT");

        if (ros_log_env)
        {
          log_file_name = ros_log_env + std::string("/log/");
        }
      }

      // sanitize the node name and tack it to the filename
      for (size_t i = 1; i < this_node::getName().length(); i++)
      {
        if (!isalnum(this_node::getName()[i]))
        {
          log_file_name += '_';
        }
        else
        {
          log_file_name += this_node::getName()[i];
        }
      }

      char pid_str[100];
      snprintf(pid_str, sizeof(pid_str), "%d", pid);
      log_file_name += std::string("_") + std::string(pid_str) + std::string(".log");
    }

    g_log_filename = log_file_name;

    const log4cxx::LoggerPtr& logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);
    log4cxx::LayoutPtr layout = new log4cxx::PatternLayout("[%c] [%d] [thread %t]: [%p] %m\n");
    log4cxx::RollingFileAppenderPtr appender = new log4cxx::RollingFileAppender(layout, log_file_name, false);
    appender->setMaximumFileSize(100*1024*1024);
    appender->setMaxBackupIndex(10);
    log4cxx::helpers::Pool pool;
    appender->activateOptions(pool);
    logger->addAppender(appender);
  }
}

} // namespace file_log

} // namespace ros
