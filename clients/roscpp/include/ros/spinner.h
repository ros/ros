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

#ifndef ROSCPP_SPINNER_H
#define ROSCPP_SPINNER_H

#include "ros/types.h"
#include "common.h"

#include <boost/shared_ptr.hpp>

namespace ros
{
class NodeHandle;
class CallbackQueue;

/**
 * \brief Abstract interface for classes which spin on a callback queue.
 */
class ROSCPP_DECL Spinner
{
public:
  virtual ~Spinner() {}

  /**
   * \brief Spin on a callback queue (defaults to the global one).  Blocks until roscpp has been shutdown.
   */
  virtual void spin(CallbackQueue* queue = 0) = 0;
};

/**
 * \brief Spinner which runs in a single thread.
 */
class SingleThreadedSpinner : public Spinner
{
public:
  virtual void spin(CallbackQueue* queue = 0);
};

/**
 * \brief Spinner which spins in multiple threads.
 */
class ROSCPP_DECL MultiThreadedSpinner : public Spinner
{
public:
  /**
   * \param thread_count Number of threads to use for calling callbacks.   0 will
   * automatically use however many hardware threads exist on your system.
   */
  MultiThreadedSpinner(uint32_t thread_count = 0);

  virtual void spin(CallbackQueue* queue = 0);

private:
  uint32_t thread_count_;
};

class AsyncSpinnerImpl;
typedef boost::shared_ptr<AsyncSpinnerImpl> AsyncSpinnerImplPtr;

/**
 * \brief AsyncSpinner is a spinner that does not conform to the abstract Spinner interface.  Instead,
 * it spins asynchronously when you call start(), and stops when either you call stop(), ros::shutdown()
 * is called, or its destructor is called
 *
 * AsyncSpinner is reference counted internally, so if you copy one it will continue spinning until all
 * copies have destructed (or stop() has been called on one of them)
 */
class ROSCPP_DECL AsyncSpinner
{
public:
  /**
   * \brief Simple constructor.  Uses the global callback queue
   * \param thread_count The number of threads to use.  A value of 0 means to use the number of processor cores
   */
  AsyncSpinner(uint32_t thread_count);

  /**
   * \brief Constructor with custom callback queue
   * \param thread_count The number of threads to use.  A value of 0 means to use the number of processor cores
   * \param queue The callback queue to operate on.  A null value means to use the global queue
   */
  AsyncSpinner(uint32_t thread_count, CallbackQueue* queue);



  /**
   * \brief Start this spinner spinning asynchronously
   */
  void start();
  /**
   * \brief Stop this spinner from running
   */
  void stop();

private:
  AsyncSpinnerImplPtr impl_;
};

}

#endif // ROSCPP_SPIN_POLICY_H
