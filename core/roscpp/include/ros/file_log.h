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

#ifndef ROSCPP_FILE_LOG_H
#define ROSCPP_FILE_LOG_H

#include "forwards.h"
#include <ros/console.h>

namespace log4cxx
{
namespace helpers
{
template <class T> class ObjectPtrT;
}

class Logger;
typedef helpers::ObjectPtrT<Logger> LoggerPtr;
class Level;
typedef helpers::ObjectPtrT<Level> LevelPtr;
}

#define ROSCPP_LOG_DEBUG(...) \
    do \
    { \
      ROSCONSOLE_AUTOINIT; \
      ROSCONSOLE_DEFINE_LOCATION(true, ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME); \
      \
      if (ROS_UNLIKELY(enabled)) \
      { \
        ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
      } \
      else if (log4cxx::LoggerPtr logger = ros::file_log::getFileOnlyLogger()) \
      { \
        ros::console::print(logger, ros::console::levels::Debug, __FILE__, __LINE__, __ROSCONSOLE_FUNCTION__, __VA_ARGS__); \
      } \
    } while(0)

namespace ros
{

/**
 * \brief internal
 */
namespace file_log
{

const std::string& getLogFilename();
const std::string& getLogDirectory();

log4cxx::LoggerPtr& getFileOnlyLogger();
log4cxx::LevelPtr getDebugLevel();

}

}

#endif // ROSCPP_FILE_LOG_H
