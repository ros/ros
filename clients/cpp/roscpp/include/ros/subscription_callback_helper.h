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

#ifndef ROSCPP_SUBSCRIPTION_CALLBACK_HELPER_H
#define ROSCPP_SUBSCRIPTION_CALLBACK_HELPER_H

#include <typeinfo>

#include "common.h"
#include "ros/forwards.h"
#include "ros/parameter_adapter.h"
#include "ros/message_traits.h"
#include "ros/builtin_message_traits.h"
#include "ros/serialization.h"
#include "ros/message_event.h"
#include <ros/static_assert.h>

#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/make_shared.hpp>

namespace ros
{

namespace serialization
{
// Additional serialization traits

template<typename M>
struct PreDeserializeParams
{
  boost::shared_ptr<M> message;
  boost::shared_ptr<std::map<std::string, std::string> > connection_header;
};

/**
 * \brief called by the SubscriptionCallbackHelper after a message is
 * instantiated but before that message is deserialized
 */
template<typename M>
struct PreDeserialize
{
  static void notify(const PreDeserializeParams<M>&) { }
};

}

template<typename T>
void
assignSubscriptionConnectionHeader(T* t, const boost::shared_ptr<M_string>& connection_header,
                                   typename boost::enable_if<ros::message_traits::IsMessage<T> >::type*_=0)
{
  (void)_; // warning stopper
  t->__connection_header = connection_header;
}

template<typename T>
void
assignSubscriptionConnectionHeader(T* t, const boost::shared_ptr<M_string>& connection_header,
                                   typename boost::disable_if<ros::message_traits::IsMessage<T> >::type*_=0)
{ 
  (void)_; // warning stopper
}

struct SubscriptionCallbackHelperDeserializeParams
{
  uint8_t* buffer;
  uint32_t length;
  boost::shared_ptr<M_string> connection_header;
};

struct ROSCPP_DECL SubscriptionCallbackHelperCallParams
{
  MessageEvent<void const> event;
};

/**
 * \brief Abstract base class used by subscriptions to deal with concrete message types through a common
 * interface.  This is one part of the roscpp API that is \b not fully stable, so overloading this class
 * is not recommended.
 */
class ROSCPP_DECL SubscriptionCallbackHelper
{
public:
  virtual ~SubscriptionCallbackHelper() {}
  virtual VoidConstPtr deserialize(const SubscriptionCallbackHelperDeserializeParams&) = 0;
  virtual void call(SubscriptionCallbackHelperCallParams& params) = 0;
  virtual const std::type_info& getTypeInfo() = 0;
  virtual bool isConst() = 0;
};
typedef boost::shared_ptr<SubscriptionCallbackHelper> SubscriptionCallbackHelperPtr;

/**
 * \brief Concrete generic implementation of
 * SubscriptionCallbackHelper for any normal message type.  Use
 * directly with care, this is mostly for internal use.
 */
template<typename P, typename Enabled = void>
class SubscriptionCallbackHelperT : public SubscriptionCallbackHelper
{
public:
  typedef ParameterAdapter<P> Adapter;
  typedef typename ParameterAdapter<P>::Message NonConstType;
  typedef typename ParameterAdapter<P>::Event Event;
  typedef typename boost::add_const<NonConstType>::type ConstType;
  typedef boost::shared_ptr<NonConstType> NonConstTypePtr;
  typedef boost::shared_ptr<ConstType> ConstTypePtr;

  static const bool is_const = ParameterAdapter<P>::is_const;

  typedef boost::function<void(typename Adapter::Parameter)> Callback;
  typedef boost::function<NonConstTypePtr()> CreateFunction;

  SubscriptionCallbackHelperT(const Callback& callback, 
			      const CreateFunction& create = DefaultMessageCreator<NonConstType>())
    : callback_(callback)
    , create_(create)
  { }

  void setCreateFunction(const CreateFunction& create)
  {
    create_ = create;
  }

  virtual VoidConstPtr deserialize(const SubscriptionCallbackHelperDeserializeParams& params)
  {
    namespace ser = serialization;

    NonConstTypePtr msg = create_();

    if (!msg)
    {
      ROS_DEBUG("Allocation failed for message of type [%s]", getTypeInfo().name());
      return VoidConstPtr();
    }

    assignSubscriptionConnectionHeader(msg.get(), params.connection_header);

    ser::PreDeserializeParams<NonConstType> predes_params;
    predes_params.message = msg;
    predes_params.connection_header = params.connection_header;
    ser::PreDeserialize<NonConstType>::notify(predes_params);

    ser::IStream stream(params.buffer, params.length);
    ser::deserialize(stream, *msg);

    return VoidConstPtr(msg);
  }

  virtual void call(SubscriptionCallbackHelperCallParams& params)
  {
    Event event(params.event, create_);
    callback_(ParameterAdapter<P>::getParameter(event));
  }

  virtual const std::type_info& getTypeInfo()
  {
    return typeid(NonConstType);
  }

  virtual bool isConst()
  {
    return ParameterAdapter<P>::is_const;
  }

private:
  Callback callback_;
  CreateFunction create_;
};

}

#endif // ROSCPP_SUBSCRIPTION_CALLBACK_HELPER_H
