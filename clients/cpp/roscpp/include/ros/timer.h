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
 *   * Neither the names of Willow Garage, Inc. nor the names of its
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

#ifndef ROSCPP_TIMER_H
#define ROSCPP_TIMER_H

#include "common.h"
#include "forwards.h"
#include "timer_options.h"

namespace ros
{

/**
 * \brief Manages a timer callback
 *
 * A Timer should always be created through a call to NodeHandle::createTimer(), or copied from one
 * that was. Once all copies of a specific
 * Timer go out of scope, the callback associated with that handle will stop
 * being called.
 */
class ROSCPP_DECL Timer
{
public:
  Timer() {}
  Timer(const Timer& rhs);
  ~Timer();

  /**
   * \brief Start the timer.  Does nothing if the timer is already started.
   */
  void start();
  /**
   * \brief Stop the timer.  Once this call returns, no more callbacks will be called.  Does
   * nothing if the timer is already stopped.
   */
  void stop();

  /**
   * \brief Returns whether or not the timer has any pending events to call.
   */
  bool hasPending();

  /**
   * \brief Set the period of this timer
   */
  void setPeriod(const Duration& period);

  bool isValid() { return impl_ && impl_->isValid(); }
  operator void*() { return isValid() ? (void*)1 : (void*)0; }

  bool operator<(const Timer& rhs)
  {
    return impl_ < rhs.impl_;
  }

  bool operator==(const Timer& rhs)
  {
    return impl_ == rhs.impl_;
  }

  bool operator!=(const Timer& rhs)
  {
    return impl_ != rhs.impl_;
  }

private:
  Timer(const TimerOptions& ops);

  class Impl
  {
  public:
    Impl();
    ~Impl();

    bool isValid();
    bool hasPending();
    void setPeriod(const Duration& period);

    void start();
    void stop();

    bool started_;
    int32_t timer_handle_;

    Duration period_;
    TimerCallback callback_;
    CallbackQueueInterface* callback_queue_;
    VoidConstWPtr tracked_object_;
    bool has_tracked_object_;
    bool oneshot_;
  };
  typedef boost::shared_ptr<Impl> ImplPtr;
  typedef boost::weak_ptr<Impl> ImplWPtr;

  ImplPtr impl_;

  friend class NodeHandle;
};

}

#endif
