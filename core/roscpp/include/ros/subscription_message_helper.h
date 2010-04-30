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

#ifndef ROSCPP_SUBSCRIPTION_MESSAGE_HELPER_H
#define ROSCPP_SUBSCRIPTION_MESSAGE_HELPER_H

#include "ros/forwards.h"
#include "ros/message.h"

#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/remove_const.hpp>

namespace ros
{

/**
 * \brief Abstract base class used by subscriptions to deal with concrete message types through a common
 * interface.  This is one part of the roscpp API that is \b not fully stable, so overloading this class
 * is not recommended unless you have an explicit need (like to implement a scripting interface).
 */
class SubscriptionMessageHelper
{
public:
  virtual ~SubscriptionMessageHelper() {}
  /**
   * \brief Create a message
   */
  virtual MessagePtr create() = 0;

  /**
   * \brief Returns the md5sum of this message
   */
  virtual std::string getMD5Sum() = 0;
  /**
   * \brief Returns the datatype of this message
   */
  virtual std::string getDataType() = 0;

  /**
   * \brief Call the callback associated with this helper on the specified message
   */
  virtual void call(const MessagePtr& msg) = 0;
};
typedef boost::shared_ptr<SubscriptionMessageHelper> SubscriptionMessageHelperPtr;

/**
 * \brief Concrete generic implementation of SubscriptionMessageHelper for any normal message type
 */
template<class M>
class SubscriptionMessageHelperT : public SubscriptionMessageHelper
{
public:
  typedef boost::shared_ptr<M> MPtr;
  typedef boost::function<void (const MPtr&)> Callback;
  SubscriptionMessageHelperT(const Callback& callback)
  : callback_(callback)
  {}

  virtual MessagePtr create()
  {
    typedef typename boost::remove_const<M>::type NonConstType;
    NonConstType* msg = new NonConstType;
    return MessagePtr(msg);
  }
  virtual void call(const MessagePtr& msg)
  {
    MPtr casted_msg = boost::static_pointer_cast<M>(msg);
    callback_(casted_msg);
  }

  virtual std::string getMD5Sum() { return M::__s_getMD5Sum(); }
  virtual std::string getDataType() { return M::__s_getDataType(); }

private:
  Callback callback_;
};

}

#endif // ROSCPP_SUBSCRIPTION_MESSAGE_HELPER_H
