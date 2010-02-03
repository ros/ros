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

#include <cstdio>
#include <sstream>
#include <ros/time.h>

// TODO: this header is no longer needed to be included here, but removing it will break various code that incorrectly does not itself include log4cxx/logger.h
// We should vet all the code using log4cxx directly and make sure the includes/link flags are used in those packages, and then we can remove this include
#include <log4cxx/logger.h>

#ifdef __GNUC__
#if __GNUC__ >= 3
#define ROSCONSOLE_PRINTF_ATTRIBUTE(a, b) __attribute__ ((__format__ (__printf__, a, b)));
#endif
#endif

#ifndef ROSCONSOLE_PRINTF_ATTRIBUTE
#define ROSCONSOLE_PRINTF_ATTRIBUTE(a, b)
#endif

// log4cxx forward declarations
namespace log4cxx
{
namespace helpers
{
template<typename T> class ObjectPtrT;
} // namespace helpers

class Level;
typedef helpers::ObjectPtrT<Level> LevelPtr;

class Logger;
typedef helpers::ObjectPtrT<Logger> LoggerPtr;
} // namespace log4cxx

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
void print(log4cxx::Logger* logger, Level level, const char* file, int line, const char* function, const char* fmt, ...) ROSCONSOLE_PRINTF_ATTRIBUTE(6, 7);

void print(log4cxx::Logger* logger, Level level, const std::stringstream& str, const char* file, int line, const char* function);

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

struct LogLocation;
void initializeLogLocation(LogLocation* loc, const std::string& name, Level level);
void setLogLocationLevel(LogLocation* loc, Level level);
void checkLogLocationEnabled(LogLocation* loc);

struct LogLocation
{
  bool initialized_;
  bool logger_enabled_;
  ros::console::Level level_;
  log4cxx::Logger* logger_;
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

#if defined(MSVC)
  #define __ROSCONSOLE_FUNCTION__ __FUNCSIG__
#elif defined(__GNUC__)
  #define __ROSCONSOLE_FUNCTION__ __PRETTY_FUNCTION__
#else
  #define __ROSCONSOLE_FUNCTION__ ""
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
  ROSCONSOLE_AUTOINIT; \
  static ros::console::LogLocation loc = {false, false, ros::console::levels::Count, 0}; /* Initialized at compile-time */ \
  if (ROS_UNLIKELY(!loc.initialized_)) \
  { \
    initializeLogLocation(&loc, name, level); \
  } \
  if (ROS_UNLIKELY(loc.level_ != level)) \
  { \
    setLogLocationLevel(&loc, level); \
    checkLogLocationEnabled(&loc); \
  } \
  bool enabled = loc.logger_enabled_ && (cond);

#define ROSCONSOLE_PRINT_AT_LOCATION(...) \
    ros::console::print( loc.logger_, loc.level_, __FILE__, __LINE__, __ROSCONSOLE_FUNCTION__, __VA_ARGS__)

#define ROSCONSOLE_PRINT_STREAM_AT_LOCATION(args) \
  do \
  { \
    std::stringstream ss; \
    ss << args; \
    ros::console::print(loc.logger_, loc.level_, ss, __FILE__, __LINE__, __ROSCONSOLE_FUNCTION__); \
  } while (0)

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
    ROSCONSOLE_DEFINE_LOCATION(cond, level, name); \
    \
    if (ROS_UNLIKELY(enabled)) \
    { \
      ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
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
    ROSCONSOLE_DEFINE_LOCATION(cond, level, name); \
    if (ROS_UNLIKELY(enabled)) \
    { \
      ROSCONSOLE_PRINT_STREAM_AT_LOCATION(args); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, only the first time it is hit when enabled, with printf-style formatting
 *
 * \param level One of the levels specified in ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define ROS_LOG_ONCE(level, name, ...) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static bool hit = false; \
    if (ROS_UNLIKELY(enabled) && ROS_UNLIKELY(!hit)) \
    { \
      hit = true; \
      ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, only the first time it is hit when enabled, with printf-style formatting
 *
 * \param level One of the levels specified in ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define ROS_LOG_STREAM_ONCE(level, name, args) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static bool hit = false; \
    if (ROS_UNLIKELY(enabled) && ROS_UNLIKELY(!hit)) \
    { \
      hit = true; \
      ROSCONSOLE_PRINT_STREAM_AT_LOCATION(args); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with printf-style formatting
 *
 * \param level One of the levels specified in ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 * \param rate The rate it should actually trigger at
 */
#define ROS_LOG_LIMIT(rate, level, name, ...) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static double last_hit = 0.0; \
    ros::Time now = ros::Time::now(); \
    if (ROS_UNLIKELY(enabled) && ROS_UNLIKELY(last_hit + rate <= now.toSec())) \
    { \
      last_hit = now.toSec(); \
      ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with printf-style formatting
 *
 * \param level One of the levels specified in ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 * \param rate The rate it should actually trigger at
 */
#define ROS_LOG_STREAM_LIMIT(rate, level, name, args) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static double last_hit = 0.0; \
    ros::Time now = ros::Time::now(); \
    if (ROS_UNLIKELY(enabled) && ROS_UNLIKELY(last_hit + rate <= now.toSec())) \
    { \
      last_hit = now.toSec(); \
      ROSCONSOLE_PRINT_STREAM_AT_LOCATION(args); \
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
