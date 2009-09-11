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

// Author: Josh Faust

#ifndef ROSCONSOLE_ROSCONSOLE_H
#define ROSCONSOLE_ROSCONSOLE_H

#include "log4cxx/logger.h"

#ifdef __GNUC__
#if __GNUC__ >= 3
#define ROSCONSOLE_PRINTF_ATTRIBUTE(a, b) __attribute__ ((__format__ (__printf__, a, b)));
#endif
#endif

#ifndef ROSCONSOLE_PRINTF_ATTRIBUTE
#define ROSCONSOLE_PRINTF_ATTRIBUTE(a, b)
#endif

namespace ros
{
namespace console
{

namespace levels
{
enum Level
{
  Debug,
  Info,
  Warn,
  Error,
  Fatal,

  Count
};
}
typedef levels::Level Level;

extern log4cxx::LevelPtr g_level_lookup[];

/**
 * \brief Only exported because the macros need it.  Do not use directly.
 */
extern bool g_initialized;

/**
 * \brief Don't call this directly.  Performs any required initialization/configuration.  Happens automatically when using the macro API.
 *
 * If you're going to be using log4cxx or any of the ros::console functions, and need the system to be initialized, use the
 * ROSCONSOLE_AUTOINIT macro.
 */
void initialize();

/**
 * \brief Don't call this directly.  Use the ROS_LOG() macro instead.
 * @param level Logging level
 * @param file File this logging statement is from (usually generated with __FILE__)
 * @param line Line of code this logging statement is from (usually generated with __LINE__)
 * @param fmt Format string
 */
void print(log4cxx::LoggerPtr& logger, const log4cxx::LevelPtr& level, const log4cxx::spi::LocationInfo& location, const char* fmt, ...) ROSCONSOLE_PRINTF_ATTRIBUTE(4, 5);

struct LogLocation;

/**
 * \brief Registers a logging location with the system.
 *
 * This is used for the case where a logger's verbosity level changes, and we need to reset the enabled status of
 * all the logging statements.
 * @param loc The location to add
 */
void registerLogLocation(LogLocation* loc);

/**
 * \brief Tells the system that a logger's level has changed
 *
 * This must be called if a log4cxx::Logger's level has been changed in the middle of an application run.
 * Because of the way the static guard for enablement works, if a logger's level is changed and this
 * function is not called, only logging statements which are first hit *after* the change will be correct wrt
 * that logger.
 */
void notifyLoggerLevelsChanged();

struct LogLocation
{
  inline void initialize(const std::string& name)
  {
    logger_ = log4cxx::Logger::getLogger(name);
    level_ = levels::Count;
    logger_enabled_ = false;

    registerLogLocation(this);
  }

  inline void setLevel(Level level)
  {
    level_ = level;
    log4cxx_level_ = &g_level_lookup[level];
  }

  inline void checkEnabled()
  {
    logger_enabled_ = logger_->isEnabledFor(*log4cxx_level_);
  }

  log4cxx::LoggerPtr logger_;
  bool logger_enabled_;
  ros::console::Level level_;
  log4cxx::LevelPtr* log4cxx_level_;
};

} // namespace console
} // namespace ros

#ifdef WIN32
#define ROS_LIKELY(x)       (x)
#define ROS_UNLIKELY(x)     (x)
#else
#define ROS_LIKELY(x)       __builtin_expect((x),1)
#define ROS_UNLIKELY(x)     __builtin_expect((x),0)
#endif

#ifdef ROS_PACKAGE_NAME
#define ROSCONSOLE_PACKAGE_NAME ROS_PACKAGE_NAME
#else
#define ROSCONSOLE_PACKAGE_NAME "unknown_package"
#endif

#define ROSCONSOLE_ROOT_LOGGER_NAME "ros"
#define ROSCONSOLE_NAME_PREFIX ROSCONSOLE_ROOT_LOGGER_NAME "." ROSCONSOLE_PACKAGE_NAME
#define ROSCONSOLE_DEFAULT_NAME ROSCONSOLE_NAME_PREFIX

// These allow you to compile-out everything below a certain severity level if necessary
#define ROSCONSOLE_SEVERITY_DEBUG 0
#define ROSCONSOLE_SEVERITY_INFO 1
#define ROSCONSOLE_SEVERITY_WARN 2
#define ROSCONSOLE_SEVERITY_ERROR 3
#define ROSCONSOLE_SEVERITY_FATAL 4
#define ROSCONSOLE_SEVERITY_NONE 5

/**
 * \def ROSCONSOLE_MIN_SEVERITY
 *
 * Define ROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL] in your build options to compile out anything below that severity
 */
#ifndef ROSCONSOLE_MIN_SEVERITY
#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG
#endif

/**
 * \def ROSCONSOLE_AUTOINIT
 * \brief Initializes the rosconsole library.  Usually unnecessary to call directly.
 */
#define ROSCONSOLE_AUTOINIT \
  do \
  { \
    if (ROS_UNLIKELY(!ros::console::g_initialized)) \
    { \
      ros::console::initialize(); \
    } \
  } while(0)

#define ROSCONSOLE_DEFINE_LOCATION(cond, level, name) \
  static ros::console::LogLocation loc; \
  if (ROS_UNLIKELY(loc.logger_ == NULL)) \
  { \
    loc.initialize(name); \
  } \
  if (ROS_UNLIKELY(loc.level_ != level)) \
  { \
    loc.setLevel(level); \
    loc.checkEnabled(); \
  } \
  bool enabled = loc.logger_enabled_ && (cond);

/**
 * \brief Log to a given named logger at a given verbosity level, only if a given condition has been met, with printf-style formatting
 *
 * \note The condition will only be evaluated if this logging statement is enabled
 *
 * \param cond Boolean condition to be evaluated
 * \param level One of the levels specified in ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define ROS_LOG_COND(cond, level, name, ...) \
  do \
  { \
    ROSCONSOLE_AUTOINIT; \
    ROSCONSOLE_DEFINE_LOCATION(cond, level, name); \
    \
    if (ROS_UNLIKELY(enabled)) \
    { \
      ros::console::print( loc.logger_, *loc.log4cxx_level_, LOG4CXX_LOCATION, __VA_ARGS__); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, only if a given condition has been met, with stream-style formatting
 *
 * \note The condition will only be evaluated if this logging statement is enabled
 *
 * \param cond Boolean condition to be evaluated
 * \param level One of the levels specified in ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define ROS_LOG_STREAM_COND(cond, level, name, args) \
  do \
  { \
    ROSCONSOLE_AUTOINIT; \
    ROSCONSOLE_DEFINE_LOCATION(cond, level, name); \
    if (ROS_UNLIKELY(enabled)) \
    { \
      log4cxx::helpers::MessageBuffer oss_; \
      loc.logger_->forcedLog(*loc.log4cxx_level_, oss_.str(oss_ << args), LOG4CXX_LOCATION); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, with printf-style formatting
 *
 * \param level One of the levels specified in ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define ROS_LOG(level, name, ...) ROS_LOG_COND(true, level, name, __VA_ARGS__)
/**
 * \brief Log to a given named logger at a given verbosity level, with stream-style formatting
 *
 * \param level One of the levels specified in ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define ROS_LOG_STREAM(level, name, args) ROS_LOG_STREAM_COND(true, level, name, args)

#include "rosconsole/macros_generated.h"

#endif // ROSCONSOLE_ROSCONSOLE_H
