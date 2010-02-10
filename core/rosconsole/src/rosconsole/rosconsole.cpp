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
#include <boost/regex.hpp>

#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <cstring>
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

const char* g_format_string = "[${severity}] [${time}]: ${message}";

struct Token
{
  virtual const char* getString(const log4cxx::spi::LoggingEventPtr& event) = 0;
};
typedef boost::shared_ptr<Token> TokenPtr;
typedef std::vector<TokenPtr> V_Token;

struct FixedToken : public Token
{
  FixedToken(const std::string& str)
  : str_(str)
  {}

  virtual const char* getString(const log4cxx::spi::LoggingEventPtr& event)
  {
    return str_.c_str();
  }

  std::string str_;
};

struct PlaceHolderToken : public Token
{
  virtual const char* getString(const log4cxx::spi::LoggingEventPtr& event)
  {
    return "PLACEHOLDER";
  }
};

struct SeverityToken : public Token
{
  virtual const char* getString(const log4cxx::spi::LoggingEventPtr& event)
  {
    if (event->getLevel() == log4cxx::Level::getFatal())
    {
      return "FATAL";
    }
    else if (event->getLevel() == log4cxx::Level::getError())
    {
      return "ERROR";
    }
    else if (event->getLevel() == log4cxx::Level::getWarn())
    {
      return " WARN";
    }
    else if (event->getLevel() == log4cxx::Level::getInfo())
    {
      return " INFO";
    }
    else if (event->getLevel() == log4cxx::Level::getDebug())
    {
      return "DEBUG";
    }

    return "UNKNO";
  }
};

struct MessageToken : public Token
{
  virtual const char* getString(const log4cxx::spi::LoggingEventPtr& event)
  {
    return event->getMessage().c_str();
  }
};

struct TimeToken : public Token
{
  virtual const char* getString(const log4cxx::spi::LoggingEventPtr& event)
  {
    std::stringstream ss;
    ss << ros::Time::now();
    return ss.str().c_str();
  }
};

struct ThreadToken : public Token
{
  virtual const char* getString(const log4cxx::spi::LoggingEventPtr& event)
  {
    std::stringstream ss;
    ss << boost::this_thread::get_id();
    return ss.str().c_str();
  }
};

struct LoggerToken : public Token
{
  virtual const char* getString(const log4cxx::spi::LoggingEventPtr& event)
  {
    return event->getLoggerName().c_str();
  }
};

struct FileToken : public Token
{
  virtual const char* getString(const log4cxx::spi::LoggingEventPtr& event)
  {
    return event->getLocationInformation().getFileName();
  }
};

struct FunctionToken : public Token
{
  virtual const char* getString(const log4cxx::spi::LoggingEventPtr& event)
  {
    return event->getLocationInformation().getMethodName().c_str();
  }
};

struct LineToken : public Token
{
  virtual const char* getString(const log4cxx::spi::LoggingEventPtr& event)
  {
    std::stringstream ss;
    ss << event->getLocationInformation().getLineNumber();
    return ss.str().c_str();
  }
};

TokenPtr createTokenFromType(const std::string& type)
{
  if (type == "severity")
  {
    return TokenPtr(new SeverityToken());
  }
  else if (type == "message")
  {
    return TokenPtr(new MessageToken());
  }
  else if (type == "time")
  {
    return TokenPtr(new TimeToken());
  }
  else if (type == "thread")
  {
    return TokenPtr(new ThreadToken());
  }
  else if (type == "logger")
  {
    return TokenPtr(new LoggerToken());
  }
  else if (type == "file")
  {
    return TokenPtr(new FileToken());
  }
  else if (type == "line")
  {
    return TokenPtr(new LineToken());
  }
  else if (type == "function")
  {
    return TokenPtr(new FunctionToken());
  }

  return TokenPtr(new FixedToken("${" + type + "}"));
}

struct Formatter
{
  void init(const char* fmt)
  {
    format_ = fmt;

    boost::regex e("\\$\\{([a-z|A-Z]+)\\}");
    boost::match_results<std::string::const_iterator> results;
    std::string::const_iterator start, end;
    start = format_.begin();
    end = format_.end();
    bool matched_once = false;
    std::string last_suffix;
    while (boost::regex_search(start, end, results, e))
    {
#if 0
      for (size_t i = 0; i < results.size(); ++i)
      {
        std::cout << i << "|" << results.prefix() << "|" <<  results[i] << "|" << results.suffix() << std::endl;
      }
#endif

      std::string token = results[1];
      last_suffix = results.suffix();
      tokens_.push_back(TokenPtr(new FixedToken(results.prefix())));
      tokens_.push_back(createTokenFromType(token));

      start = results[0].second;
      matched_once = true;
    }

    if (matched_once)
    {
      tokens_.push_back(TokenPtr(new FixedToken(last_suffix)));
    }
    else
    {
      tokens_.push_back(TokenPtr(new FixedToken(format_)));
    }
  }

  void print(const log4cxx::spi::LoggingEventPtr& event)
  {
    const char* color = NULL;
    FILE* f = stdout;

    if (event->getLevel() == log4cxx::Level::getFatal())
    {
      color = COLOR_RED;
      f = stderr;
    }
    else if (event->getLevel() == log4cxx::Level::getError())
    {
      color = COLOR_RED;
      f = stderr;
    }
    else if (event->getLevel() == log4cxx::Level::getWarn())
    {
      color = COLOR_YELLOW;
    }
    else if (event->getLevel() == log4cxx::Level::getInfo())
    {
      color = COLOR_NORMAL;
    }
    else if (event->getLevel() == log4cxx::Level::getDebug())
    {
      color = COLOR_GREEN;
    }

    ROS_ASSERT(color != NULL);

    std::stringstream ss;
    ss << color;
    V_Token::iterator it = tokens_.begin();
    V_Token::iterator end = tokens_.end();
    for (; it != end; ++it)
    {
      ss << (*it)->getString(event);
    }
    ss << COLOR_NORMAL;

    fprintf(f, "%s\n", ss.str().c_str());
  }

  std::string format_;
  V_Token tokens_;

};
Formatter g_formatter;

class ROSConsoleStdioAppender : public log4cxx::AppenderSkeleton
{
public:
  ~ROSConsoleStdioAppender()
  {
  }

protected:
  virtual void append(const log4cxx::spi::LoggingEventPtr& event, log4cxx::helpers::Pool& pool)
  {
    g_formatter.print(event);
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

  // Check for the format string environment variable
  const char* format_string = getenv("ROSCONSOLE_FORMAT");
  if (format_string)
  {
    g_format_string = format_string;
  }

  g_formatter.init(g_format_string);

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
static size_t g_print_buffer_size = INITIAL_BUFFER_SIZE;
static boost::thread::id g_printing_thread_id;
void print(FilterBase* filter, log4cxx::Logger* logger, Level level, const char* file, int line, const char* function, const char* fmt, ...)
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

  size_t total = vsnprintf(g_print_buffer.get(), g_print_buffer_size, fmt, args);
  if (total >= g_print_buffer_size)
  {
    va_end(args);
    va_start(args, fmt);

    g_print_buffer_size = total + 1;
    g_print_buffer.reset(new char[g_print_buffer_size]);

    vsnprintf(g_print_buffer.get(), g_print_buffer_size, fmt, args);
  }

  va_end(args);

  log4cxx::LoggerPtr logger_ptr(logger);
  bool enabled = true;

  if (filter)
  {
    FilterParams params;
    params.file = file;
    params.function = function;
    params.line = line;
    params.level = level;
    params.logger = logger_ptr;
    params.message = g_print_buffer.get();
    params.out_level = level;
    enabled = filter->isEnabled(params);
    level = params.out_level;
    if (params.out_logger != 0)
    {
      logger_ptr = params.out_logger;
    }

    if (!params.out_message.empty())
    {
      size_t msg_size = params.out_message.size();
      if (g_print_buffer_size <= msg_size)
      {
        g_print_buffer_size = msg_size + 1;
        g_print_buffer.reset(new char[g_print_buffer_size]);
      }

      memcpy(g_print_buffer.get(), params.out_message.c_str(), msg_size + 1);
    }
  }

  if (enabled)
  {
    try
    {
      logger_ptr->forcedLog(g_level_lookup[level], g_print_buffer.get(), log4cxx::spi::LocationInfo(file, function, line));
    }
    catch (std::exception& e)
    {
      fprintf(stderr, "Caught exception while logging: [%s]\n", e.what());
    }
  }

  g_printing_thread_id = boost::thread::id();
}

void print(FilterBase* filter, log4cxx::Logger* logger, Level level, const std::stringstream& ss, const char* file, int line, const char* function)
{
  if (g_printing_thread_id == boost::this_thread::get_id())
  {
    fprintf(stderr, "Warning: recursive print statement has occurred.  Throwing out recursive print.\n");
    return;
  }

  boost::mutex::scoped_lock lock(g_print_mutex);

  g_printing_thread_id = boost::this_thread::get_id();

  log4cxx::LoggerPtr logger_ptr(logger);
  bool enabled = true;
  std::string str = ss.str();

  if (filter)
  {
    FilterParams params;
    params.file = file;
    params.function = function;
    params.line = line;
    params.level = level;
    params.logger = logger_ptr;
    params.message = g_print_buffer.get();
    params.out_level = level;
    enabled = filter->isEnabled(params);
    level = params.out_level;
    if (params.out_logger != 0)
    {
      logger_ptr = params.out_logger;
    }

    if (!params.out_message.empty())
    {
      str = params.out_message;
    }
  }

  if (enabled)
  {
    try
    {
      logger->forcedLog(g_level_lookup[level], str, log4cxx::spi::LocationInfo(file, function, line));
    }
    catch (std::exception& e)
    {
      fprintf(stderr, "Caught exception while logging: [%s]\n", e.what());
    }
  }

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

void checkLogLocationEnabledNoLock(LogLocation* loc)
{
  loc->logger_enabled_ = loc->logger_->isEnabledFor(g_level_lookup[loc->level_]);
}

void initializeLogLocation(LogLocation* loc, const std::string& name, Level level)
{
  boost::mutex::scoped_lock lock(g_locations_mutex);

  if (loc->initialized_)
  {
    return;
  }

  loc->logger_ = &(*log4cxx::Logger::getLogger(name));
  loc->level_ = level;

  g_log_locations.push_back(loc);

  checkLogLocationEnabledNoLock(loc);

  loc->initialized_ = true;
}

void setLogLocationLevel(LogLocation* loc, Level level)
{
  boost::mutex::scoped_lock lock(g_locations_mutex);
  loc->level_ = level;
}

void checkLogLocationEnabled(LogLocation* loc)
{
  boost::mutex::scoped_lock lock(g_locations_mutex);
  checkLogLocationEnabledNoLock(loc);
}

void notifyLoggerLevelsChanged()
{
  boost::mutex::scoped_lock lock(g_locations_mutex);

  V_LogLocation::iterator it = g_log_locations.begin();
  V_LogLocation::iterator end = g_log_locations.end();
  for ( ; it != end; ++it )
  {
    LogLocation* loc = *it;
    checkLogLocationEnabledNoLock(loc);
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
