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

#ifndef ROSCONSOLE_STATIC_ASSERT_H
#define ROSCONSOLE_STATIC_ASSERT_H

// boost's static assert provides better errors messages in the failure case when using
// in templated situations
#include <boost/static_assert.hpp>

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
#define ROS_COMPILE_ASSERT(cond) BOOST_STATIC_ASSERT(cond)

/**
 * \def ROS_STATIC_ASSERT(cond)
 * \brief Compile-time assert.
 *
 * Only works with compile time statements, ie:
 @verbatim
   struct A
   {
     uint32_t a;
   };
   ROS_STATIC_ASSERT(sizeof(A) == 4);
 @endverbatim
 */
#define ROS_STATIC_ASSERT(cond) BOOST_STATIC_ASSERT(cond)


#endif // ROSCONSOLE_STATIC_ASSERT_H
