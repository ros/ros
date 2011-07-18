/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSCPP_CALLBACK_QUEUE_INTERFACE_H
#define ROSCPP_CALLBACK_QUEUE_INTERFACE_H

#include <boost/shared_ptr.hpp>
#include "common.h"
#include "ros/types.h"

namespace ros
{

/**
 * \brief Abstract interface for items which can be added to a CallbackQueueInterface
 */
class ROSCPP_DECL CallbackInterface
{
public:
  /**
   * \brief Possible results for the call() method
   */
  enum CallResult
  {
    Success,   ///< Call succeeded
    TryAgain,  ///< Call not ready, try again later
    Invalid,   ///< Call no longer valid
  };

  virtual ~CallbackInterface() {}

  /**
   * \brief Call this callback
   * \return The result of the call
   */
  virtual CallResult call() = 0;
  /**
   * \brief Provides the opportunity for specifying that a callback is not ready to be called
   * before call() actually takes place.
   */
  virtual bool ready() { return true; }
};
typedef boost::shared_ptr<CallbackInterface> CallbackInterfacePtr;

/**
 * \brief Abstract interface for a queue used to handle all callbacks within roscpp.
 *
 * Allows you to inherit and provide your own implementation that can be used instead of our
 * default CallbackQueue
 */
class CallbackQueueInterface
{
public:
  virtual ~CallbackQueueInterface() {}

  /**
   * \brief Add a callback, with an optional owner id.  The owner id can be used to
   * remove a set of callbacks from this queue.
   */
  virtual void addCallback(const CallbackInterfacePtr& callback, uint64_t owner_id = 0) = 0;

  /**
   * \brief Remove all callbacks associated with an owner id
   */
  virtual void removeByID(uint64_t owner_id) = 0;
};

}

#endif
