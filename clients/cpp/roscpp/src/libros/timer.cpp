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

#include "ros/timer.h"
#include "ros/timer_manager.h"

namespace ros
{

Timer::Impl::Impl()
  : started_(false)
  , timer_handle_(-1)
{ }

Timer::Impl::~Impl()
{
  ROS_DEBUG("Timer deregistering callbacks.");
  stop();
}

bool Timer::Impl::isValid()
{
  return !period_.isZero();
}

void Timer::Impl::start()
{
  if (!started_)
  {
    VoidConstPtr tracked_object;
    if (has_tracked_object_)
    {
      tracked_object = tracked_object_.lock();
    }

    timer_handle_ = TimerManager<Time, Duration, TimerEvent>::global().add(period_, callback_, callback_queue_, tracked_object, oneshot_);
    started_ = true;
  }
}

void Timer::Impl::stop()
{
  if (started_)
  {
    started_ = false;
    TimerManager<Time, Duration, TimerEvent>::global().remove(timer_handle_);
    timer_handle_ = -1;
  }
}

bool Timer::Impl::hasPending()
{
  if (!isValid() || timer_handle_ == -1)
  {
    return false;
  }

  return TimerManager<Time, Duration, TimerEvent>::global().hasPending(timer_handle_);
}

void Timer::Impl::setPeriod(const Duration& period)
{
  period_ = period;
  TimerManager<Time, Duration, TimerEvent>::global().setPeriod(timer_handle_, period);
}

Timer::Timer(const TimerOptions& ops)
: impl_(new Impl)
{
  impl_->period_ = ops.period;
  impl_->callback_ = ops.callback;
  impl_->callback_queue_ = ops.callback_queue;
  impl_->tracked_object_ = ops.tracked_object;
  impl_->has_tracked_object_ = ops.tracked_object;
  impl_->oneshot_ = ops.oneshot;
}

Timer::Timer(const Timer& rhs)
{
  impl_ = rhs.impl_;
}

Timer::~Timer()
{
}

void Timer::start()
{
  if (impl_)
  {
    impl_->start();
  }
}

void Timer::stop()
{
  if (impl_)
  {
    impl_->stop();
  }
}

bool Timer::hasPending()
{
  if (impl_)
  {
    return impl_->hasPending();
  }

  return false;
}

void Timer::setPeriod(const Duration& period)
{
  if (impl_)
  {
    impl_->setPeriod(period);
  }
}

}
