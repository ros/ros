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
#include <cstdarg>
#include <ros/macros.h>

// Import/export for windows dll's and visibility for gcc shared libraries.

#ifdef ROS_BUILD_SHARED_LIBS // ros is being built around shared libraries
  #ifdef rosconsole_EXPORTS // we are building a shared lib/dll
    #define ROSCONSOLE_DECL ROS_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define ROSCONSOLE_DECL ROS_HELPER_IMPORT
  #endif
#else // ros is being built around static libraries
  #define ROSCONSOLE_DECL
#endif

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

namespace boost
{
template<typename T> class shared_array;
}

namespace ros
{
namespace console
{

ROSCONSOLE_DECL void shutdown();

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

extern ROSCONSOLE_DECL log4cxx::LevelPtr g_level_lookup[];

/**
 * \brief Only exported because the macros need it.  Do not use directly.
 */
extern ROSCONSOLE_DECL bool g_initialized;

/**
 * \brief Don't call this directly.  Performs any required initialization/configuration.  Happens automatically when using the macro API.
 *
 * If you're going to be using log4cxx or any of the ::ros::console functions, and need the system to be initialized, use the
 * ROSCONSOLE_AUTOINIT macro.
 */
ROSCONSOLE_DECL void initialize();

class FilterBase;
/**
 * \brief Don't call this directly.  Use the ROS_LOG() macro instead.
 * @param level Logging level
 * @param file File this logging statement is from (usually generated with __FILE__)
 * @param line Line of code this logging statement is from (usually generated with __LINE__)
 * @param fmt Format string
 */
ROSCONSOLE_DECL void print(FilterBase* filter, log4cxx::Logger* logger, Level level,
	   const char* file, int line, 
	   const char* function, const char* fmt, ...) ROSCONSOLE_PRINTF_ATTRIBUTE(7, 8);

ROSCONSOLE_DECL void print(FilterBase* filter, log4cxx::Logger* logger, Level level,
	   const std::stringstream& str, const char* file, int line, const char* function);

struct ROSCONSOLE_DECL LogLocation;

/**
 * \brief Registers a logging location with the system.
 *
 * This is used for the case where a logger's verbosity level changes, and we need to reset the enabled status of
 * all the logging statements.
 * @param loc The location to add
 */
ROSCONSOLE_DECL void registerLogLocation(LogLocation* loc);

/**
 * \brief Tells the system that a logger's level has changed
 *
 * This must be called if a log4cxx::Logger's level has been changed in the middle of an application run.
 * Because of the way the static guard for enablement works, if a logger's level is changed and this
 * function is not called, only logging statements which are first hit *after* the change will be correct wrt
 * that logger.
 */
ROSCONSOLE_DECL void notifyLoggerLevelsChanged();

ROSCONSOLE_DECL void setFixedFilterToken(const std::string& key, const std::string& val);

/**
 * \brief Parameter structure passed to FilterBase::isEnabled(...);.  Includes both input and output parameters
 */
struct FilterParams
{
  // input parameters
  const char* file;                         ///< [input] File the message came from
  int line;                                 ///< [input] Line the message came from
  const char* function;                     ///< [input] Function the message came from
  const char* message;                      ///< [input] The formatted message that will be output

  // input/output parameters
  log4cxx::LoggerPtr logger;                ///< [input/output] Logger that this message will be output to.  If changed, uses the new logger
  Level level;                              ///< [input/output] Severity level.  If changed, uses the new level

  // output parameters
  std::string out_message;                  ///< [output] If set, writes this message instead of the original
};

/**
 * \brief Base-class for filters.  Filters allow full user-defined control over whether or not a message should print.
 * The ROS_X_FILTER... macros provide the filtering functionality.
 *
 * Filters get a chance to veto the message from printing at two times: first before the message arguments are
 * evaluated and the message is formatted, and then once the message is formatted before it is printed.  It is also possible
 * to change the message, logger and severity level at this stage (see the FilterParams struct for more details).
 *
 * When a ROS_X_FILTER... macro is called, here is the high-level view of how it uses the filter passed in:
\verbatim
if (<logging level is enabled> && filter->isEnabled())
{
  <format message>
  <fill out FilterParams>
  if (filter->isEnabled(params))
  {
    <print message>
  }
}
\endverbatim
 */
class FilterBase
{
public:
  virtual ~FilterBase() {}
  /**
   * \brief Returns whether or not the log statement should be printed.  Called before the log arguments are evaluated
   * and the message is formatted.
   */
  inline virtual bool isEnabled() { return true; }
  /**
   * \brief Returns whether or not the log statement should be printed.  Called once the message has been formatted,
   * and allows you to change the message, logger and severity level if necessary.
   */
  inline virtual bool isEnabled(FilterParams&) { return true; }
};

struct ROSCONSOLE_DECL LogLocation;
/**
 * \brief Internal
 */
ROSCONSOLE_DECL void initializeLogLocation(LogLocation* loc, const std::string& name, Level level);
/**
 * \brief Internal
 */
ROSCONSOLE_DECL void setLogLocationLevel(LogLocation* loc, Level level);
/**
 * \brief Internal
 */
ROSCONSOLE_DECL void checkLogLocationEnabled(LogLocation* loc);

/**
 * \brief Internal
 */
struct LogLocation
{
  bool initialized_;
  bool logger_enabled_;
  ::ros::console::Level level_;
  log4cxx::Logger* logger_;
};

ROSCONSOLE_DECL void vformatToBuffer(boost::shared_array<char>& buffer, size_t& buffer_size, const char* fmt, va_list args);
ROSCONSOLE_DECL void formatToBuffer(boost::shared_array<char>& buffer, size_t& buffer_size, const char* fmt, ...);
ROSCONSOLE_DECL std::string formatToString(const char* fmt, ...);

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
    if (ROS_UNLIKELY(!::ros::console::g_initialized)) \
    { \
      ::ros::console::initialize(); \
    } \
  } while(0)

#define ROSCONSOLE_DEFINE_LOCATION(cond, level, name) \
  ROSCONSOLE_AUTOINIT; \
  static ::ros::console::LogLocation loc = {false, false, ::ros::console::levels::Count, 0}; /* Initialized at compile-time */ \
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

#define ROSCONSOLE_PRINT_AT_LOCATION_WITH_FILTER(filter, ...) \
    ::ros::console::print(filter, loc.logger_, loc.level_, __FILE__, __LINE__, __ROSCONSOLE_FUNCTION__, __VA_ARGS__)

#define ROSCONSOLE_PRINT_AT_LOCATION(...) \
    ROSCONSOLE_PRINT_AT_LOCATION_WITH_FILTER(0, __VA_ARGS__)

// inside a macro which uses args use only well namespaced variable names in order to not overlay variables coming in via args
#define ROSCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER(filter, args) \
  do \
  { \
    std::stringstream __rosconsole_print_stream_at_location_with_filter__ss__; \
    __rosconsole_print_stream_at_location_with_filter__ss__ << args; \
    ::ros::console::print(filter, loc.logger_, loc.level_, __rosconsole_print_stream_at_location_with_filter__ss__, __FILE__, __LINE__, __ROSCONSOLE_FUNCTION__); \
  } while (0)

#define ROSCONSOLE_PRINT_STREAM_AT_LOCATION(args) \
    ROSCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER(0, args)

/**
 * \brief Log to a given named logger at a given verbosity level, only if a given condition has been met, with printf-style formatting
 *
 * \note The condition will only be evaluated if this logging statement is enabled
 *
 * \param cond Boolean condition to be evaluated
 * \param level One of the levels specified in ::ros::console::levels::Level
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
 * \param level One of the levels specified in ::ros::console::levels::Level
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
 * \param level One of the levels specified in ::ros::console::levels::Level
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

// inside a macro which uses args use only well namespaced variable names in order to not overlay variables coming in via args
/**
 * \brief Log to a given named logger at a given verbosity level, only the first time it is hit when enabled, with printf-style formatting
 *
 * \param level One of the levels specified in ::ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define ROS_LOG_STREAM_ONCE(level, name, args) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static bool __ros_log_stream_once__hit__ = false; \
    if (ROS_UNLIKELY(enabled) && ROS_UNLIKELY(!__ros_log_stream_once__hit__)) \
    { \
      __ros_log_stream_once__hit__ = true; \
      ROSCONSOLE_PRINT_STREAM_AT_LOCATION(args); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with printf-style formatting
 *
 * \param level One of the levels specified in ::ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 * \param rate The rate it should actually trigger at
 */
#define ROS_LOG_THROTTLE(rate, level, name, ...) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static double last_hit = 0.0; \
    ::ros::Time now = ::ros::Time::now(); \
    if (ROS_UNLIKELY(enabled) && ROS_UNLIKELY(last_hit + rate <= now.toSec())) \
    { \
      last_hit = now.toSec(); \
      ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
    } \
  } while(0)

// inside a macro which uses args use only well namespaced variable names in order to not overlay variables coming in via args
/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with printf-style formatting
 *
 * \param level One of the levels specified in ::ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 * \param rate The rate it should actually trigger at
 */
#define ROS_LOG_STREAM_THROTTLE(rate, level, name, args) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static double __ros_log_stream_throttle__last_hit__ = 0.0; \
    ::ros::Time __ros_log_stream_throttle__now__ = ::ros::Time::now(); \
    if (ROS_UNLIKELY(enabled) && ROS_UNLIKELY(__ros_log_stream_throttle__last_hit__ + rate <= __ros_log_stream_throttle__now__.toSec())) \
    { \
      __ros_log_stream_throttle__last_hit__ = __ros_log_stream_throttle__now__.toSec(); \
      ROSCONSOLE_PRINT_STREAM_AT_LOCATION(args); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, with user-defined filtering, with printf-style formatting
 *
 * \param filter pointer to the filter to be used
 * \param level One of the levels specified in ::ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define ROS_LOG_FILTER(filter, level, name, ...) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    if (ROS_UNLIKELY(enabled) && (filter)->isEnabled()) \
    { \
      ROSCONSOLE_PRINT_AT_LOCATION_WITH_FILTER(filter, __VA_ARGS__); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, with user-defined filtering, with stream-style formatting
 *
 * \param cond Boolean condition to be evaluated
 * \param level One of the levels specified in ::ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define ROS_LOG_STREAM_FILTER(filter, level, name, args) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    if (ROS_UNLIKELY(enabled) && (filter)->isEnabled()) \
    { \
      ROSCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER(filter, args); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, with printf-style formatting
 *
 * \param level One of the levels specified in ::ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define ROS_LOG(level, name, ...) ROS_LOG_COND(true, level, name, __VA_ARGS__)
/**
 * \brief Log to a given named logger at a given verbosity level, with stream-style formatting
 *
 * \param level One of the levels specified in ::ros::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use ROSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define ROS_LOG_STREAM(level, name, args) ROS_LOG_STREAM_COND(true, level, name, args)

#include "rosconsole/macros_generated.h"

#endif // ROSCONSOLE_ROSCONSOLE_H
