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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

// Author: Josh Faust

#include "ros/console.h"
#include "ros/assert.h"
#include <ros/time.h>
#include "log4cxx/appenderskeleton.h"
#include "log4cxx/spi/loggingevent.h"
#include "log4cxx/level.h"
#include "log4cxx/propertyconfigurator.h"

#include <boost/thread.hpp>
#include <boost/shared_array.hpp>

#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <stdexcept>

namespace ros
{
namespace console
{

bool g_initialized = false;
boost::mutex g_init_mutex;

log4cxx::LevelPtr g_level_lookup[ levels::Count ] =
{
  log4cxx::Level::getDebug(),
  log4cxx::Level::getInfo(),
  log4cxx::Level::getWarn(),
  log4cxx::Level::getError(),
  log4cxx::Level::getFatal(),
};

#define COLOR_NORMAL "\033[0m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"

class ROSConsoleStdioAppender : public log4cxx::AppenderSkeleton
{
public:
  ~ROSConsoleStdioAppender()
  {
  }

protected:
  virtual void append(const log4cxx::spi::LoggingEventPtr& event, log4cxx::helpers::Pool& pool)
  {
    const char* color = NULL;
    const char* prefix = NULL;
    FILE* f = stdout;

    if (event->getLevel() == log4cxx::Level::getFatal())
    {
      color = COLOR_RED;
      prefix = "FATAL";
      f = stderr;
    }
    else if (event->getLevel() == log4cxx::Level::getError())
    {
      color = COLOR_RED;
      prefix = "ERROR";
      f = stderr;
    }
    else if (event->getLevel() == log4cxx::Level::getWarn())
    {
      color = COLOR_YELLOW;
      prefix = "WARN";
    }
    else if (event->getLevel() == log4cxx::Level::getInfo())
    {
      color = COLOR_NORMAL;
      prefix = "INFO";
    }
    else if (event->getLevel() == log4cxx::Level::getDebug())
    {
      color = COLOR_GREEN;
      prefix = "DEBUG";
    }

    ROS_ASSERT(color != NULL);
    ROS_ASSERT(prefix != NULL);

    std::stringstream ss;
    ss << ros::Time::now();
    fprintf(f, "%s[%5s] %s: %s%s\n", color, prefix, ss.str().c_str(), event->getMessage().c_str(), COLOR_NORMAL);
  }

  virtual void close()
  {
  }
  virtual bool requiresLayout() const
  {
    return false;
  }
};

/**
 * \brief For internal use only. Does the actual initialization of the rosconsole system.  Should only be called once.
 */
void do_initialize()
{
  // First load the default config file from ROS_ROOT/config/rosconsole.config
  const char* ros_root_cstr = getenv("ROS_ROOT");
  if (!ros_root_cstr)
  {
    fprintf(stderr, "ROS_ROOT is not set!  Using default rosconsole configuration values\n");

    log4cxx::LoggerPtr ros_logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);
    ros_logger->setLevel(log4cxx::Level::getInfo());

    log4cxx::LoggerPtr roscpp_superdebug = log4cxx::Logger::getLogger("ros.roscpp.superdebug");
    roscpp_superdebug->setLevel(log4cxx::Level::getWarn());
  }
  else
  {
    std::string config_file = std::string(ros_root_cstr) + "/config/rosconsole.config";
    log4cxx::PropertyConfigurator::configure(config_file);
    
    const char* config_file_cstr = getenv("ROSCONSOLE_CONFIG_FILE");
    if ( config_file_cstr )
    {
      config_file = config_file_cstr;
      
      log4cxx::PropertyConfigurator::configure(config_file);
    }
  }

  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);
  logger->addAppender(new ROSConsoleStdioAppender);
}

void initialize()
{
  boost::mutex::scoped_lock lock(g_init_mutex);

  if (!g_initialized)
  {
    do_initialize();
    g_initialized = true;
  }
}

#define INITIAL_BUFFER_SIZE 4096
static boost::mutex g_print_mutex;
static boost::shared_array<char> g_print_buffer(new char[INITIAL_BUFFER_SIZE]);
static int g_print_buffer_size = INITIAL_BUFFER_SIZE;
static boost::thread::id g_printing_thread_id;
void print(log4cxx::LoggerPtr& logger, const log4cxx::LevelPtr& level, const log4cxx::spi::LocationInfo& location,
    const char* fmt, ...)
{
  if (g_printing_thread_id == boost::this_thread::get_id())
  {
    fprintf(stderr, "Warning: recursive print statement has occurred.  Throwing out recursive print.\n");
    return;
  }

  boost::mutex::scoped_lock lock(g_print_mutex);

  g_printing_thread_id = boost::this_thread::get_id();

  va_list args;
  va_start(args, fmt);

  int total = vsnprintf(g_print_buffer.get(), g_print_buffer_size, fmt, args);
  if (total >= g_print_buffer_size)
  {
    va_end(args);
    va_start(args, fmt);

    g_print_buffer_size = total + 1;
    g_print_buffer.reset(new char[g_print_buffer_size]);

    vsnprintf(g_print_buffer.get(), g_print_buffer_size, fmt, args);
  }

  va_end(args);

  logger->forcedLog(level, g_print_buffer.get(), location);

  g_printing_thread_id = boost::thread::id();
}

typedef std::vector<LogLocation*> V_LogLocation;
V_LogLocation g_log_locations;
boost::mutex g_locations_mutex;
void registerLogLocation(LogLocation* loc)
{
  boost::mutex::scoped_lock lock(g_locations_mutex);

  g_log_locations.push_back(loc);
}

void notifyLoggerLevelsChanged()
{
  boost::mutex::scoped_lock lock(g_locations_mutex);

  V_LogLocation::iterator it = g_log_locations.begin();
  V_LogLocation::iterator end = g_log_locations.end();
  for ( ; it != end; ++it )
  {
    LogLocation* loc = *it;
    loc->checkEnabled();
  }
}

class StaticInit
{
public:
  StaticInit()
  {
    ROSCONSOLE_AUTOINIT;
  }
};
StaticInit g_static_init;

} // namespace console
} // namespace ros
