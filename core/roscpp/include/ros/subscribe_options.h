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

#ifndef ROSCPP_SUBSCRIBE_OPTIONS_H
#define ROSCPP_SUBSCRIBE_OPTIONS_H

#include "ros/forwards.h"
#include "ros/transport_hints.h"
#include "subscription_message_helper.h"

namespace ros
{

/**
 * \brief Encapsulates all options available for creating a Subscriber
 */
struct SubscribeOptions
{
  /**
   *
   */
  SubscribeOptions()
  : queue_size(1)
  , callback_queue(0)
  {
  }

  /**
   * \brief Constructor
   * \param _topic Topic to subscribe on
   * \param _queue_size Number of incoming messages to queue up for
   *        processing (messages in excess of this queue capacity will be
   *        discarded).
   * \param _helper Helper object used to get create messages and call callbacks
   */
  SubscribeOptions(const std::string& _topic, uint32_t _queue_size, const SubscriptionMessageHelperPtr& _helper)
  : topic(_topic)
  , queue_size(_queue_size)
  , md5sum(_helper->getMD5Sum())
  , datatype(_helper->getDataType())
  , helper(_helper)
  , callback_queue(0)
  {}

  /**
   * \brief Constructor
   * \param _topic Topic to subscribe on
   * \param _queue_size Number of incoming messages to queue up for
   *        processing (messages in excess of this queue capacity will be
   *        discarded).
   * \param _md5sum
   * \param _datatype
   */
  SubscribeOptions(const std::string& _topic, uint32_t _queue_size, const std::string& _md5sum, const std::string& _datatype)
  : topic(_topic)
  , queue_size(_queue_size)
  , md5sum(_md5sum)
  , datatype(_datatype)
  , callback_queue(0)
  {}

  /**
   * \brief Templated initialization
   * \param _topic Topic to subscribe on
   * \param _queue_size Number of incoming messages to queue up for
   *        processing (messages in excess of this queue capacity will be
   *        discarded).
   * \param _callback Callback to call when a message arrives on this topic
   */
  template<class M>
  void init(const std::string& _topic, uint32_t _queue_size,
       const boost::function<void (const boost::shared_ptr<M>&)>& _callback)
  {
    topic = _topic;
    queue_size = _queue_size;
    md5sum = M::__s_getMD5Sum();
    datatype = M::__s_getDataType();
    helper = SubscriptionMessageHelperPtr(new SubscriptionMessageHelperT<M>(_callback));
  }

  std::string topic;                                                ///< Topic to subscribe to
  uint32_t queue_size;                                              ///< Number of incoming messages to queue up for processing (messages in excess of this queue capacity will be discarded).

  std::string md5sum;                                               ///< MD5 of the message datatype
  std::string datatype;                                             ///< Datatype of the message we'd like to subscribe as

  SubscriptionMessageHelperPtr helper;                              ///< Helper object used to get create messages and call callbacks

  CallbackQueueInterface* callback_queue;                           ///< Queue to add callbacks to.  If NULL, the global callback queue will be used

  /**
   * \brief An object whose destruction will prevent the callback associated with this subscription
   *
   * A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
   * and if the reference count goes to 0 the subscriber callbacks will not get called.
   *
   * \note Note that setting this will cause a new reference to be added to the object before the
   * callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
   * thread) that the callback is invoked from.
   */
  VoidPtr tracked_object;

  TransportHints transport_hints;                                   ///< Hints for transport type and options

  /**
   * \brief Templated helper function for creating an AdvertiseServiceOptions with most of its options
   * \param topic Topic name to subscribe to
   * \param queue_size Number of incoming messages to queue up for
   *        processing (messages in excess of this queue capacity will be
   *        discarded).
   * \param callback The callback to invoke when a message is received on this topic
   * \param tracked_object The tracked object to use (see SubscribeOptions::tracked_object)
   * \param queue The callback queue to use (see SubscribeOptions::callback_queue)
   */
  template<class M>
  static SubscribeOptions create(const std::string& topic, uint32_t queue_size,
                                 const boost::function<void (const boost::shared_ptr<M>&)>& callback,
                                 const VoidPtr& tracked_object, CallbackQueueInterface* queue)
  {
    SubscribeOptions ops;
    ops.init<M>(topic, queue_size, callback);
    ops.tracked_object = tracked_object;
    ops.callback_queue = queue;
    return ops;
  }
};

}

#endif


