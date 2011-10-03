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

#ifndef ROSCPP_WALL_TIMER_OPTIONS_H
#define ROSCPP_WALL_TIMER_OPTIONS_H

#include "common.h"
#include "ros/forwards.h"

namespace ros
{

/**
 * \brief Encapsulates all options available for starting a timer
 */
struct ROSCPP_DECL WallTimerOptions
{
  WallTimerOptions()
  : period(0.1)
  , callback_queue(0)
  , oneshot(false)
  , autostart(true)
  {
  }

  /*
   * \brief Constructor
   * \param
   */
  WallTimerOptions(WallDuration _period, const WallTimerCallback& _callback, CallbackQueueInterface* _queue, 
                   bool oneshot = false, bool autostart = true)
  : period(_period)
  , callback(_callback)
  , callback_queue(_queue)
  , oneshot(oneshot)
  , autostart(autostart)
  {}

  WallDuration period;                                              ///< The period to call the callback at
  WallTimerCallback callback;                                       ///< The callback to call

  CallbackQueueInterface* callback_queue;                           ///< Queue to add callbacks to.  If NULL, the global callback queue will be used

  /**
   * A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
   * and if the reference count goes to 0 the subscriber callbacks will not get called.
   *
   * \note Note that setting this will cause a new reference to be added to the object before the
   * callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
   * thread) that the callback is invoked from.
   */
  VoidConstPtr tracked_object;

  bool oneshot;
  bool autostart;
};


}

#endif

