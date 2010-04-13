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

#ifndef ROSCONSOLE_ROSASSERT_H
#define ROSCONSOLE_ROSASSERT_H

#include "ros/console.h"

/** \file */

/** \def ROS_ASSERT(cond)
 * \brief Asserts that the provided condition evaluates to true.
 *
 * If it is false, program execution will abort, with an informative
 * statement about which assertion failed, in what file.  Use ROS_ASSERT
 * instead of assert() itself.
 *
 * If running inside a debugger, ROS_ASSERT will allow you to step past the assertion.
 */

/** \def ROS_ASSERT_MSG(cond, ...)
 * \brief Asserts that the provided condition evaluates to true.
 *
 * If it is false, program execution will abort, with an informative
 * statement about which assertion failed, in what file, and it will print out
 * a printf-style message you define.  Example usage:
 @verbatim
   ROS_ASSERT_MSG(x > 0, "Uh oh, x went negative.  Value = %d", x);
 @endverbatim
 *
 * If running inside a debugger, ROS_ASSERT will allow you to step past the assertion.
 */

/**
 * \def ROS_ASSERT_CMD()
 * \brief Runs a command if the provided condition is false
 *
 * For example:
\verbatim
  ROS_ASSERT_CMD(x > 0, handleError(...));
\endverbatim
 */

/** \def ROS_BREAK()
 * \brief Aborts program execution.
 *
 * Aborts program execution with an informative message stating what file and
 * line it was called from. Use ROS_BREAK instead of calling assert(0) or
 * ROS_ASSERT(0).
 *
 * If running inside a debugger, ROS_BREAK will allow you to step past the breakpoint.
 */

/** \def ROS_ISSUE_BREAK()
 * \brief Always issues a breakpoint instruction.
 *
 * This define is mostly for internal use, but is useful if you want to simply issue a break
 * instruction in a cross-platform way.
 *
 * Currently implemented for Windows (any platform), powerpc64, and x86
 */

#ifdef WIN32
# define ROS_ISSUE_BREAK() __debugbreak();
#elif defined(__powerpc64__)
# define ROS_ISSUE_BREAK() asm volatile ("tw 31,1,1");
#elif defined(__i386__) || defined(__ia64__) || defined(__x86_64__)
# define ROS_ISSUE_BREAK() asm("int $3");
#else
# include <cassert>
# define ROS_ISSUE_BREAK() assert(false);
#endif

/**
 * \def ROS_COMPILE_ASSERT(cond)
 * \brief Compile-time assert.
 *
 * Only works with compile time statements, ie:
 @verbatim
   struct A
   {
     uint32_t a;
   };
   ROS_COMPILE_ASSERT(sizeof(A) == 4);
 @endverbatim
 */
#define ROS_COMPILE_ASSERT(cond) typedef char __ROS_COMPILE_ASSERT__[(cond)?1:-1]

#ifndef NDEBUG
#define ROS_ASSERT_ENABLED
#endif

#ifdef ROS_ASSERT_ENABLED
#define ROS_BREAK() \
  do { \
    ROS_FATAL("BREAKPOINT HIT\n\tfile = %s\n\tline=%d\n", __FILE__, __LINE__); \
    ROS_ISSUE_BREAK() \
  } while (0)

#define ROS_ASSERT(cond) \
  do { \
    if (!(cond)) { \
      ROS_FATAL("ASSERTION FAILED\n\tfile = %s\n\tline = %d\n\tcond = %s\n", __FILE__, __LINE__, #cond); \
      ROS_ISSUE_BREAK() \
    } \
  } while (0)

#define ROS_ASSERT_MSG(cond, ...) \
  do { \
    if (!(cond)) { \
      ROS_FATAL("ASSERTION FAILED\n\tfile = %s\n\tline = %d\n\tcond = %s\n\tmessage = ", __FILE__, __LINE__, #cond); \
      ROS_FATAL(__VA_ARGS__); \
      ROS_FATAL("\n"); \
      ROS_ISSUE_BREAK(); \
    } \
  } while (0)

#define ROS_ASSERT_CMD(cond, cmd) \
  do { \
    if (!cond) { \
      cmd; \
    } \
  } while (0)


#else
#define ROS_BREAK()
#define ROS_ASSERT(cond)
#define ROS_ASSERT_MSG(cond, ...)
#define ROS_ASSERT_CMD(cond, cmd)
#endif

#endif // ROSCONSOLE_ROSASSERT_H
