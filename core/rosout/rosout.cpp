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

#include <cstring>
#include <cstdlib>

#include "ros/ros.h"
#include "roslib/Log.h"

#include "log4cxx/logger.h"
#include "log4cxx/rollingfileappender.h"
#include "log4cxx/patternlayout.h"
#include "log4cxx/helpers/pool.h"

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 *
 * @b rosout logs messages sent to the /rosout topic,
 * which is a system-wide string logging mechanism.
 */

/**
 * the rosout node subscribes to /rosout, logs the messages to file, and re-broadcasts the messages to /rosout_agg
 */
class Rosout
{
public:
  log4cxx::LoggerPtr logger_;
  ros::NodeHandle node_;
  ros::Subscriber rosout_sub_;
  ros::Publisher agg_pub_;

  Rosout()
  {
    init();
  }

  void init()
  {
    //calculate log directory
    std::string log_file_name;
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

    if (!ros_log_env)
      throw std::runtime_error("Neither ROS_ROOT nor ROS_LOG_DIR are defined");

    log_file_name += std::string("/rosout.log");

    logger_ = log4cxx::Logger::getRootLogger();
    log4cxx::LayoutPtr layout = new log4cxx::PatternLayout("");
    log4cxx::RollingFileAppenderPtr appender = new log4cxx::RollingFileAppender(layout, log_file_name, true);
    logger_->addAppender( appender );
    appender->setMaximumFileSize(100*1024*1024);
    appender->setMaxBackupIndex(10);
    log4cxx::helpers::Pool pool;
    appender->activateOptions(pool);

    std::cout << "logging to " << log_file_name << std::endl;

    LOG4CXX_INFO(logger_, "\n\n" << ros::Time::now() << "  Node Startup\n");

    agg_pub_ = node_.advertise<roslib::Log>("/rosout_agg", 0);
    std::cout << "re-publishing aggregated messages to /rosout_agg" << std::endl;

    rosout_sub_ = node_.subscribe("/rosout", 0, &Rosout::rosoutCallback, this);
    std::cout << "subscribed to /rosout" << std::endl;
  }

  void rosoutCallback(const roslib::Log::ConstPtr& msg)
  {
    agg_pub_.publish(msg);

    std::stringstream ss;
    ss << msg->header.stamp << " ";
    switch (msg->level)
    {
    case roslib::Log::FATAL:
      ss << "FATAL ";
      break;
    case roslib::Log::ERROR:
      ss << "ERROR ";
      break;
    case roslib::Log::WARN:
      ss << "WARN ";
      break;
    case roslib::Log::DEBUG:
      ss << "DEBUG ";
      break;
    case roslib::Log::INFO:
      ss << "INFO ";
      break;
    default:
      ss << msg->level << " ";
    }

    ss << "[" << msg->file << ":" << msg->line << "(" << msg->function << ") ";

    ss << "[topics: ";
    std::vector<std::string>::const_iterator it = msg->topics.begin();
    std::vector<std::string>::const_iterator end = msg->topics.end();
    for ( ; it != end; ++it )
    {
      const std::string& topic = *it;

      if ( it != msg->topics.begin() )
      {
        ss << ", ";
      }

      ss << topic;
    }
    ss << "] ";

    ss << msg->msg;
    LOG4CXX_INFO(logger_, ss.str());
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosout", ros::init_options::NoRosout);
  ros::NodeHandle n;
  Rosout r;

  ros::spin();

  return 0;
}

