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

#ifndef ROSCPP_ADVERTISE_OPTIONS_H
#define ROSCPP_ADVERTISE_OPTIONS_H

#include "ros/forwards.h"
#include "ros/message_traits.h"
#include "common.h"

namespace ros
{

/**
 * \brief Encapsulates all options available for creating a Publisher
 */
struct ROSCPP_DECL AdvertiseOptions
{
  AdvertiseOptions()
  : callback_queue(0)
  , latch(false)
  {
  }

  /*
   * \brief Constructor
   * \param _topic Topic to publish on
   * \param _queue_size Maximum number of outgoing messages to be queued for delivery to subscribers
   * \param _md5sum The md5sum of the message datatype published on this topic
   * \param _datatype Datatype of the message published on this topic (eg. "std_msgs/String")
   * \param _connect_cb Function to call when a subscriber connects to this topic
   * \param _disconnect_cb Function to call when a subscriber disconnects from this topic
   */
  AdvertiseOptions(const std::string& _topic, uint32_t _queue_size, const std::string& _md5sum,
                   const std::string& _datatype, const std::string& _message_definition,
                   const SubscriberStatusCallback& _connect_cb = SubscriberStatusCallback(),
                   const SubscriberStatusCallback& _disconnect_cb = SubscriberStatusCallback())
  : topic(_topic)
  , queue_size(_queue_size)
  , md5sum(_md5sum)
  , datatype(_datatype)
  , message_definition(_message_definition)
  , connect_cb(_connect_cb)
  , disconnect_cb(_disconnect_cb)
  , callback_queue(0)
  , latch(false)
  , has_header(false)
  {}

  /**
   * \brief templated helper function for automatically filling out md5sum, datatype and message definition
   *
   * \param M [template] Message type
   * \param _topic Topic to publish on
   * \param _queue_size Maximum number of outgoing messages to be queued for delivery to subscribers
   * \param _connect_cb Function to call when a subscriber connects to this topic
   * \param _disconnect_cb Function to call when a subscriber disconnects from this topic
   */
  template <class M>
  void init(const std::string& _topic, uint32_t _queue_size,
            const SubscriberStatusCallback& _connect_cb = SubscriberStatusCallback(),
            const SubscriberStatusCallback& _disconnect_cb = SubscriberStatusCallback())
  {
    topic = _topic;
    queue_size = _queue_size;
    connect_cb = _connect_cb;
    disconnect_cb = _disconnect_cb;
    md5sum = message_traits::md5sum<M>();
    datatype = message_traits::datatype<M>();
    message_definition = message_traits::definition<M>();
    has_header = message_traits::hasHeader<M>();
  }

  std::string topic;                                                ///< The topic to publish on
  uint32_t queue_size;                                              ///< The maximum number of outgoing messages to be queued for delivery to subscribers

  std::string md5sum;                                               ///< The md5sum of the message datatype published on this topic
  std::string datatype;                                             ///< The datatype of the message published on this topic (eg. "std_msgs/String")
  std::string message_definition;                                   ///< The full definition of the message published on this topic

  SubscriberStatusCallback connect_cb;                              ///< The function to call when a subscriber connects to this topic
  SubscriberStatusCallback disconnect_cb;                           ///< The function to call when a subscriber disconnects from this topic

  CallbackQueueInterface* callback_queue;                           ///< Queue to add callbacks to.  If NULL, the global callback queue will be used

  /**
   * \brief An object whose destruction will prevent the callbacks associated with this advertisement from being called
   *
   * A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
   * and if the reference count goes to 0 the subscriber callbacks will not get called.
   *
   * \note Note that setting this will cause a new reference to be added to the object before the
   * callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
   * thread) that the callback is invoked from.
   */
  VoidConstPtr tracked_object;

  /**
   * \brief Whether or not this publication should "latch".  A latching publication will automatically send out the last published message
   * to any new subscribers.
   */
  bool latch;

  /** \brief Tells whether or not the message has a header.  If it does, the sequence number will be written directly into the
   *         serialized bytes after the message has been serialized.
   */
  bool has_header;


  /**
   * \brief Templated helper function for creating an AdvertiseOptions for a message type with most options.
   *
   * \param M [template] Message type
   * \param topic Topic to publish on
   * \param queue_size Maximum number of outgoing messages to be queued for delivery to subscribers
   * \param connect_cb Function to call when a subscriber connects to this topic
   * \param disconnect_cb Function to call when a subscriber disconnects from this topic
   * \param tracked_object tracked object to use (see AdvertiseOptions::tracked_object)
   * \param queue The callback queue to use (see AdvertiseOptions::callback_queue)
   *
   * \return an AdvertiseOptions which embodies the parameters
   */
  template<class M>
  static AdvertiseOptions create(const std::string& topic, uint32_t queue_size,
                          const SubscriberStatusCallback& connect_cb,
                          const SubscriberStatusCallback& disconnect_cb,
                          const VoidConstPtr& tracked_object,
                          CallbackQueueInterface* queue)
  {
    AdvertiseOptions ops;
    ops.init<M>(topic, queue_size, connect_cb, disconnect_cb);
    ops.tracked_object = tracked_object;
    ops.callback_queue = queue;
    return ops;
  }
};


}

#endif
