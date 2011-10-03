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

#ifndef ROSCPP_NODE_HANDLE_H
#define ROSCPP_NODE_HANDLE_H

#include "ros/forwards.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/service_server.h"
#include "ros/service_client.h"
#include "ros/timer.h"
#include "ros/rate.h"
#include "ros/wall_timer.h"
#include "ros/advertise_options.h"
#include "ros/advertise_service_options.h"
#include "ros/subscribe_options.h"
#include "ros/service_client_options.h"
#include "ros/timer_options.h"
#include "ros/wall_timer_options.h"
#include "ros/spinner.h"
#include "ros/init.h"
#include "common.h"

#include <boost/bind.hpp>

#include <XmlRpcValue.h>

namespace ros
{

  class NodeHandleBackingCollection;

  /**
   * \brief roscpp's interface for creating subscribers, publishers, etc.
   *
   * This class is used for writing nodes.  It provides a RAII interface
   * to this process' node, in that when the first NodeHandle is
   * created, it instantiates everything necessary for this node, and
   * when the last NodeHandle goes out of scope it shuts down the node.
   *
   * NodeHandle uses reference counting internally, and copying a
   * NodeHandle is very lightweight.
   *
   * You must call one of the ros::init functions prior to instantiating
   * this class.
   *
   * The most widely used methods are:
   *   - Setup:
   *    - ros::init()
   *   - Publish / subscribe messaging:
   *    - advertise()
   *    - subscribe()
   *   - RPC services:
   *    - advertiseService()
   *    - serviceClient()
   *    - ros::service::call()
   *   - Parameters:
   *    - getParam()
   *    - setParam()
   */
  class ROSCPP_DECL NodeHandle
  {
  public:
    /**
     * \brief Constructor
     *
     * When a NodeHandle is constructed, it checks to see if the global
     * node state has already been started.  If so, it increments a
     * global reference count.  If not, it starts the node with
     * ros::start() and sets the reference count to 1.
     *
     * \param ns Namespace for this NodeHandle.  This acts in addition to any namespace assigned to this ROS node.
     *           eg. If the node's namespace is "/a" and the namespace passed in here is "b", all 
     *           topics/services/parameters will be prefixed with "/a/b/"
     * \param remappings Remappings for this NodeHandle.
     * \throws InvalidNameException if the namespace is not a valid graph resource name
     */
    NodeHandle(const std::string& ns = std::string(), const M_string& remappings = M_string());
    /**
     * \brief Copy constructor
     *
     * When a NodeHandle is copied, it inherits the namespace of the
     * NodeHandle being copied, and increments the reference count of
     * the global node state by 1.
     */
    NodeHandle(const NodeHandle& rhs);
    /**
     * \brief Parent constructor
     *
     * This version of the constructor takes a "parent" NodeHandle, and is equivalent to:
     \verbatim
     NodeHandle child(parent.getNamespace() + "/" + ns);
     \endverbatim
     *
     * When a NodeHandle is copied, it inherits the namespace of the
     * NodeHandle being copied, and increments the reference count of
     * the global node state by 1.
     *
     * \throws InvalidNameException if the namespace is not a valid
     * graph resource name
     */
    NodeHandle(const NodeHandle& parent, const std::string& ns);
    /**
     * \brief Parent constructor
     *
     * This version of the constructor takes a "parent" NodeHandle, and is equivalent to:
     \verbatim
     NodeHandle child(parent.getNamespace() + "/" + ns, remappings);
     \endverbatim
     *
     * This version also lets you pass in name remappings that are specific to this NodeHandle
     *
     * When a NodeHandle is copied, it inherits the namespace of the NodeHandle being copied, 
     * and increments the reference count of the global node state
     * by 1.
     * \throws InvalidNameException if the namespace is not a valid graph resource name
     */
    NodeHandle(const NodeHandle& parent, const std::string& ns, const M_string& remappings);
    /**
     * \brief Destructor
     *
     * When a NodeHandle is destroyed, it decrements a global reference
     * count by 1, and if the reference count is now 0, shuts down the
     * node.
     */
    ~NodeHandle();

    NodeHandle& operator=(const NodeHandle& rhs);

    /**
     * \brief Set the default callback queue to be used by this NodeHandle.
     *
     * Setting this will cause any callbacks from
     * advertisements/subscriptions/services/etc. to happen through the
     * use of the specified queue.  NULL (the default) causes the global
     * queue (serviced by ros::spin() and ros::spinOnce()) to be used.
     */
    void setCallbackQueue(CallbackQueueInterface* queue);

    /**
     * \brief Returns the callback queue associated with this
     * NodeHandle.  If none has been explicitly set, returns the global
     * queue.
     */
    CallbackQueueInterface* getCallbackQueue() const 
    { 
      return callback_queue_ ? callback_queue_ : (CallbackQueueInterface*)getGlobalCallbackQueue(); 
    }

    /**
     * \brief Returns the namespace associated with this NodeHandle
     */
    const std::string& getNamespace() const { return namespace_; }

    /**
     * \brief Returns the namespace associated with this NodeHandle as
     * it was passed in (before it was resolved)
     */
    const std::string& getUnresolvedNamespace() const { return unresolved_namespace_; }

    /** \brief Resolves a name into a fully-qualified name
     *
     * Resolves a name into a fully qualified name, eg. "blah" =>
     * "/namespace/blah". By default also applies any matching
     * name-remapping rules (which were usually supplied on the command
     * line at startup) to the given name, returning the resulting
     * remapped name.
     *
     * \param name Name to remap
     *
     * \param remap Whether to apply name-remapping rules
     *
     * \return Resolved name.
     *
     * \throws InvalidNameException If the name begins with a tilde, or is an otherwise invalid graph resource name
     */
    std::string resolveName(const std::string& name, bool remap = true) const;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Versions of advertise()
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * \brief Advertise a topic, simple version
     *
     * This call connects to the master to publicize that the node will be
     * publishing messages on the given topic.  This method returns a Publisher that allows you to
     * publish a message on this topic.
     *
     * This version of advertise is a templated convenience function, and can be used like so
     *
     *   ros::Publisher pub = handle.advertise<std_msgs::Empty>("my_topic", 1);
     *
     * \param topic Topic to advertise on
     *
     * \param queue_size Maximum number of outgoing messages to be
     * queued for delivery to subscribers
     *
     * \param latch (optional) If true, the last message published on
     * this topic will be saved and sent to new subscribers when they
     * connect
     *
     * \return On success, a Publisher that, when it goes out of scope,
     * will automatically release a reference on this advertisement.  On
     * failure, an empty Publisher.
     *
     * \throws InvalidNameException If the topic name begins with a
     * tilde, or is an otherwise invalid graph resource name, or is an
     * otherwise invalid graph resource name
     */
    template <class M>
    Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)
    {
      AdvertiseOptions ops;
      ops.template init<M>(topic, queue_size);
      ops.latch = latch;
      return advertise(ops);
    }

    /**
     * \brief Advertise a topic, with most of the available options, including subscriber status callbacks
     *
     * This call connects to the master to publicize that the node will be
     * publishing messages on the given topic.  This method returns a Publisher that allows you to
     * publish a message on this topic.
     *
     * This version of advertise allows you to pass functions to be called when new subscribers connect and
     * disconnect.  With bare functions it can be used like so:
     \verbatim
     void connectCallback(const ros::SingleSubscriberPublisher& pub)
     {
     // Do something
     }

     handle.advertise<std_msgs::Empty>("my_topic", 1, connectCallback);
     \endverbatim
     *
     * With class member functions it can be used with boost::bind:
     \verbatim
     void MyClass::connectCallback(const ros::SingleSubscriberPublisher& pub)
     {
     // Do something
     }

     MyClass my_class;
     ros::Publisher pub = handle.advertise<std_msgs::Empty>("my_topic", 1, 
                                                            boost::bind(&MyClass::connectCallback, my_class, _1));
     \endverbatim
     *
   *
   * \param topic Topic to advertise on
   *
   * \param queue_size Maximum number of outgoing messages to be queued for delivery to subscribers
   *
   * \param connect_cb Function to call when a subscriber connects
   *
   * \param disconnect_cb (optional) Function to call when a subscriber disconnects
     *
   * \param tracked_object (optional) A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
   * and if the reference count goes to 0 the subscriber callbacks will not get called.
   * Note that setting this will cause a new reference to be added to the object before the
   * callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
   * thread) that the callback is invoked from.
   * \param latch (optional) If true, the last message published on this topic will be saved and sent to new subscribers when they connect
   * \return On success, a Publisher that, when it goes out of scope, will automatically release a reference
   * on this advertisement.  On failure, an empty Publisher which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   * \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   */
  template <class M>
  Publisher advertise(const std::string& topic, uint32_t queue_size,
                            const SubscriberStatusCallback& connect_cb,
                            const SubscriberStatusCallback& disconnect_cb = SubscriberStatusCallback(),
                            const VoidConstPtr& tracked_object = VoidConstPtr(),
                            bool latch = false)
  {
    AdvertiseOptions ops;
    ops.template init<M>(topic, queue_size, connect_cb, disconnect_cb);
    ops.tracked_object = tracked_object;
    ops.latch = latch;
    return advertise(ops);
  }

  /**
   * \brief Advertise a topic, with full range of AdvertiseOptions
   *
   * This call connects to the master to publicize that the node will be
   * publishing messages on the given topic.  This method returns a Publisher that allows you to
   * publish a message on this topic.
   *
   * This is an advanced version advertise() that exposes all options (through the AdvertiseOptions structure)
   *
   * \param ops Advertise options to use
   * \return On success, a Publisher that, when it goes out of scope, will automatically release a reference
   * on this advertisement.  On failure, an empty Publisher which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *
   * \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   */
  Publisher advertise(AdvertiseOptions& ops);


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Versions of subscribe()
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Subscribe to a topic, version for class member function with bare pointer
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, fp is invoked and passed a shared pointer
   * to the message received.  This message should \b not be changed in place, as it
   * is shared with any other subscriptions to this topic.
   *
   * This version of subscribe is a convenience function for using member functions, and can be used like so:
\verbatim
void Foo::callback(const std_msgs::Empty::ConstPtr& message)
{
}

Foo foo_object;
ros::Subscriber sub = handle.subscribe("my_topic", 1, &Foo::callback, &foo_object);
\endverbatim
   *
   * \param M [template] M here is the callback parameter type (e.g. const boost::shared_ptr<M const>& or const M&), \b not the message type, and should almost always be deduced
   * \param topic Topic to subscribe to
   * \param queue_size Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   * \param fp Member function pointer to call when a message has arrived
   * \param obj Object to call fp on
   * \param transport_hints a TransportHints structure which defines various transport-related options
   * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
   * On failure, an empty Subscriber which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
   */
  template<class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj, 
                       const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }

  /// and the const version
  template<class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(M) const, T* obj, 
                       const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }

  /**
   * \brief Subscribe to a topic, version for class member function with bare pointer
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, fp is invoked and passed a shared pointer
   * to the message received.  This message should \b not be changed in place, as it
   * is shared with any other subscriptions to this topic.
   *
   * This version of subscribe is a convenience function for using member functions, and can be used like so:
\verbatim
void Foo::callback(const std_msgs::Empty::ConstPtr& message)
{
}

Foo foo_object;
ros::Subscriber sub = handle.subscribe("my_topic", 1, &Foo::callback, &foo_object);
\endverbatim
   *
   * \param M [template] M here is the message type
   * \param topic Topic to subscribe to
   * \param queue_size Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   * \param fp Member function pointer to call when a message has arrived
   * \param obj Object to call fp on
   * \param transport_hints a TransportHints structure which defines various transport-related options
   * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
   * On failure, an empty Subscriber which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
   */
  template<class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, 
                       void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, 
                       const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }
  template<class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, 
                       void(T::*fp)(const boost::shared_ptr<M const>&) const, T* obj, 
                       const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }

  /**
   * \brief Subscribe to a topic, version for class member function with shared_ptr
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, fp is invoked and passed a shared pointer
   * to the message received.  This message should \b not be changed in place, as it
   * is shared with any other subscriptions to this topic.
   *
   * This version of subscribe is a convenience function for using member functions on a shared_ptr:
\verbatim
void Foo::callback(const std_msgs::Empty::ConstPtr& message)
{
}

boost::shared_ptr<Foo> foo_object(new Foo);
ros::Subscriber sub = handle.subscribe("my_topic", 1, &Foo::callback, foo_object);
\endverbatim
   *
   * \param M [template] M here is the callback parameter type (e.g. const boost::shared_ptr<M const>& or const M&), \b not the message type, and should almost always be deduced
   * \param topic Topic to subscribe to
   * \param queue_size Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   * \param fp Member function pointer to call when a message has arrived
   * \param obj Object to call fp on.  Since this is a shared pointer, the object will automatically be tracked with a weak_ptr
   * so that if it is deleted before the Subscriber goes out of scope the callback will no longer be called (and therefore will not crash).
   * \param transport_hints a TransportHints structure which defines various transport-related options
   * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
   * On failure, an empty Subscriber which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
   */
  template<class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(M), 
                       const boost::shared_ptr<T>& obj, const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
    ops.tracked_object = obj;
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }

  template<class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(M) const, 
                       const boost::shared_ptr<T>& obj, const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
    ops.tracked_object = obj;
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }

  /**
   * \brief Subscribe to a topic, version for class member function with shared_ptr
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, fp is invoked and passed a shared pointer
   * to the message received.  This message should \b not be changed in place, as it
   * is shared with any other subscriptions to this topic.
   *
   * This version of subscribe is a convenience function for using member functions on a shared_ptr:
\verbatim
void Foo::callback(const std_msgs::Empty::ConstPtr& message)
{
}

boost::shared_ptr<Foo> foo_object(new Foo);
ros::Subscriber sub = handle.subscribe("my_topic", 1, &Foo::callback, foo_object);
\endverbatim
   *
   * \param M [template] M here is the message type
   * \param topic Topic to subscribe to
   * \param queue_size Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   * \param fp Member function pointer to call when a message has arrived
   * \param obj Object to call fp on.  Since this is a shared pointer, the object will automatically be tracked with a weak_ptr
   * so that if it is deleted before the Subscriber goes out of scope the callback will no longer be called (and therefore will not crash).
   * \param transport_hints a TransportHints structure which defines various transport-related options
   * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
   * On failure, an empty Subscriber which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
   */
  template<class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, 
                       void(T::*fp)(const boost::shared_ptr<M const>&), 
                       const boost::shared_ptr<T>& obj, const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
    ops.tracked_object = obj;
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }
  template<class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, 
                       void(T::*fp)(const boost::shared_ptr<M const>&) const, 
                       const boost::shared_ptr<T>& obj, const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
    ops.tracked_object = obj;
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }

  /**
   * \brief Subscribe to a topic, version for bare function
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, fp is invoked and passed a shared pointer
   * to the message received.  This message should \b not be changed in place, as it
   * is shared with any other subscriptions to this topic.
   *
   * This version of subscribe is a convenience function for using bare functions, and can be used like so:
\verbatim
void callback(const std_msgs::Empty::ConstPtr& message)
{
}

ros::Subscriber sub = handle.subscribe("my_topic", 1, callback);
\endverbatim
   *
   * \param M [template] M here is the callback parameter type (e.g. const boost::shared_ptr<M const>& or const M&), \b not the message type, and should almost always be deduced
   * \param topic Topic to subscribe to
   * \param queue_size Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   * \param fp Function pointer to call when a message has arrived
   * \param transport_hints a TransportHints structure which defines various transport-related options
   * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
   * On failure, an empty Subscriber which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
   */
  template<class M>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(*fp)(M), const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, fp);
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }

  /**
   * \brief Subscribe to a topic, version for bare function
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, fp is invoked and passed a shared pointer
   * to the message received.  This message should \b not be changed in place, as it
   * is shared with any other subscriptions to this topic.
   *
   * This version of subscribe is a convenience function for using bare functions, and can be used like so:
\verbatim
void callback(const std_msgs::Empty::ConstPtr& message)
{
}

ros::Subscriber sub = handle.subscribe("my_topic", 1, callback);
\endverbatim
   *
   * \param M [template] M here is the message type
   * \param topic Topic to subscribe to
   * \param queue_size Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   * \param fp Function pointer to call when a message has arrived
   * \param transport_hints a TransportHints structure which defines various transport-related options
   * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
   * On failure, an empty Subscriber which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
   */
  template<class M>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr<M const>&), const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, fp);
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }

  /**
   * \brief Subscribe to a topic, version for arbitrary boost::function object
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, callback is invoked and passed a shared pointer
   * to the message received.  This message should \b not be changed in place, as it
   * is shared with any other subscriptions to this topic.
   *
   * This version of subscribe allows anything bindable to a boost::function object
   *
   * \param M [template] M here is the message type
   * \param topic Topic to subscribe to
   * \param queue_size Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   * \param callback Callback to call when a message has arrived
   * \param tracked_object A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
   * and if the reference count goes to 0 the subscriber callbacks will not get called.
   * Note that setting this will cause a new reference to be added to the object before the
   * callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
   * thread) that the callback is invoked from.
   * \param transport_hints a TransportHints structure which defines various transport-related options
   * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
   * On failure, an empty Subscriber which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
   */
  template<class M>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, const boost::function<void (const boost::shared_ptr<M const>&)>& callback,
                             const VoidConstPtr& tracked_object = VoidConstPtr(), const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, callback);
    ops.tracked_object = tracked_object;
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }

  /**
   * \brief Subscribe to a topic, version for arbitrary boost::function object
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, callback is invoked and passed a shared pointer
   * to the message received.  This message should \b not be changed in place, as it
   * is shared with any other subscriptions to this topic.
   *
   * This version of subscribe allows anything bindable to a boost::function object
   *
   * \param M [template] the message type
   * \param C [template] the callback parameter type (e.g. const boost::shared_ptr<M const>& or const M&)
   * \param topic Topic to subscribe to
   * \param queue_size Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   * \param callback Callback to call when a message has arrived
   * \param tracked_object A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
   * and if the reference count goes to 0 the subscriber callbacks will not get called.
   * Note that setting this will cause a new reference to be added to the object before the
   * callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
   * thread) that the callback is invoked from.
   * \param transport_hints a TransportHints structure which defines various transport-related options
   * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
   * On failure, an empty Subscriber which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
   */
  template<class M, class C>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, const boost::function<void (C)>& callback,
                             const VoidConstPtr& tracked_object = VoidConstPtr(), const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<C>(topic, queue_size, callback);
    ops.tracked_object = tracked_object;
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }

  /**
   * \brief Subscribe to a topic, version with full range of SubscribeOptions
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, fp is invoked and passed a shared pointer
   * to the message received.  This message should \b not be changed in place, as it
   * is shared with any other subscriptions to this topic.
   *
   * This version of subscribe allows the full range of options, exposed through the SubscribeOptions class
   *
   * \param ops Subscribe options
   * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
   * On failure, an empty Subscriber which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
   */
  Subscriber subscribe(SubscribeOptions& ops);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Versions of advertiseService()
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Advertise a service, version for class member function with bare pointer
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name.
   *
   * This is a convenience function for using member functions, and can be used like so:
\verbatim
bool Foo::callback(std_srvs::Empty& request, std_srvs::Empty& response)
{
  return true;
}

Foo foo_object;
ros::ServiceServer service = handle.advertiseService("my_service", &Foo::callback, &foo_object);
\endverbatim
   *
   * \param service Service name to advertise on
   * \param srv_func Member function pointer to call when a message has arrived
   * \param obj Object to call srv_func on
   * \return On success, a ServiceServer that, when all copies of it go out of scope, will unadvertise this service.
   * On failure, an empty ServiceServer which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the service name begins with a tilde, or is an otherwise invalid graph resource name, or is an otherwise invalid graph resource name
   */
  template<class T, class MReq, class MRes>
  ServiceServer advertiseService(const std::string& service, bool(T::*srv_func)(MReq &, MRes &), T *obj)
  {
    AdvertiseServiceOptions ops;
    ops.template init<MReq, MRes>(service, boost::bind(srv_func, obj, _1, _2));
    return advertiseService(ops);
  }

  /**
   * \brief Advertise a service, version for class member function with bare pointer using ros::ServiceEvent as the callback parameter type
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name.
   *
   * This is a convenience function for using member functions, and can be used like so:
\verbatim
bool Foo::callback(ros::ServiceEvent<std_srvs::Empty::Request, std_srvs::Empty::Response>& event)
{
  return true;
}

Foo foo_object;
ros::ServiceServer service = handle.advertiseService("my_service", &Foo::callback, &foo_object);
\endverbatim
   *
   * \param service Service name to advertise on
   * \param srv_func Member function pointer to call when a message has arrived
   * \param obj Object to call srv_func on
   * \return On success, a ServiceServer that, when all copies of it go out of scope, will unadvertise this service.
   * On failure, an empty ServiceServer which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the service name begins with a tilde, or is an otherwise invalid graph resource name, or is an otherwise invalid graph resource name
   */
  template<class T, class MReq, class MRes>
  ServiceServer advertiseService(const std::string& service, bool(T::*srv_func)(ServiceEvent<MReq, MRes>&), T *obj)
  {
    AdvertiseServiceOptions ops;
    ops.template initBySpecType<ServiceEvent<MReq, MRes> >(service, boost::bind(srv_func, obj, _1));
    return advertiseService(ops);
  }

  /**
   * \brief Advertise a service, version for class member function with shared_ptr
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name.
   *
   * This is a convenience function for using member functions on shared pointers, and can be used like so:
\verbatim
bool Foo::callback(std_srvs::Empty& request, std_srvs::Empty& response)
{
  return true;
}

boost::shared_ptr<Foo> foo_object(new Foo);
ros::ServiceServer service = handle.advertiseService("my_service", &Foo::callback, foo_object);
\endverbatim
   *
   * \param service Service name to advertise on
   * \param srv_func Member function pointer to call when a message has arrived
   * \param obj Object to call srv_func on.  Since this is a shared_ptr, it will automatically be tracked with a weak_ptr,
   * and if the object is deleted the service callback will stop being called (and therefore will not crash).
   * \return On success, a ServiceServer that, when all copies of it go out of scope, will unadvertise this service.
   * On failure, an empty ServiceServer which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   * \throws InvalidNameException If the service name begins with a tilde, or is an otherwise invalid graph resource name
   */
  template<class T, class MReq, class MRes>
  ServiceServer advertiseService(const std::string& service, bool(T::*srv_func)(MReq &, MRes &), const boost::shared_ptr<T>& obj)
  {
    AdvertiseServiceOptions ops;
    ops.template init<MReq, MRes>(service, boost::bind(srv_func, obj.get(), _1, _2));
    ops.tracked_object = obj;
    return advertiseService(ops);
  }

  /**
   * \brief Advertise a service, version for class member function with shared_ptr using ros::ServiceEvent as the callback parameter type
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name.
   *
   * This is a convenience function for using member functions on shared pointers, and can be used like so:
\verbatim
bool Foo::callback(ros::ServiceEvent<std_srvs::Empty, std_srvs::Empty>& event)
{
  return true;
}

boost::shared_ptr<Foo> foo_object(new Foo);
ros::ServiceServer service = handle.advertiseService("my_service", &Foo::callback, foo_object);
\endverbatim
   *
   * \param service Service name to advertise on
   * \param srv_func Member function pointer to call when a message has arrived
   * \param obj Object to call srv_func on.  Since this is a shared_ptr, it will automatically be tracked with a weak_ptr,
   * and if the object is deleted the service callback will stop being called (and therefore will not crash).
   * \return On success, a ServiceServer that, when all copies of it go out of scope, will unadvertise this service.
   * On failure, an empty ServiceServer which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   * \throws InvalidNameException If the service name begins with a tilde, or is an otherwise invalid graph resource name
   */
  template<class T, class MReq, class MRes>
  ServiceServer advertiseService(const std::string& service, bool(T::*srv_func)(ServiceEvent<MReq, MRes>&), const boost::shared_ptr<T>& obj)
  {
    AdvertiseServiceOptions ops;
    ops.template initBySpecType<ServiceEvent<MReq, MRes> >(service, boost::bind(srv_func, obj.get(), _1));
    ops.tracked_object = obj;
    return advertiseService(ops);
  }

  /**
   * \brief Advertise a service, version for bare function
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name.
   *
   * This is a convenience function for using bare functions, and can be used like so:
\verbatim
bool callback(std_srvs::Empty& request, std_srvs::Empty& response)
{
  return true;
}

ros::ServiceServer service = handle.advertiseService("my_service", callback);
\endverbatim
   *
   * \param service Service name to advertise on
   * \param srv_func function pointer to call when a message has arrived
   * \return On success, a ServiceServer that, when all copies of it go out of scope, will unadvertise this service.
   * On failure, an empty ServiceServer which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   * \throws InvalidNameException If the service name begins with a tilde, or is an otherwise invalid graph resource name
   */
  template<class MReq, class MRes>
  ServiceServer advertiseService(const std::string& service, bool(*srv_func)(MReq&, MRes&))
  {
    AdvertiseServiceOptions ops;
    ops.template init<MReq, MRes>(service, srv_func);
    return advertiseService(ops);
  }

  /**
   * \brief Advertise a service, version for bare function using ros::ServiceEvent as the callback parameter type
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name.
   *
   * This is a convenience function for using bare functions, and can be used like so:
\verbatim
bool callback(ros::ServiceEvent<std_srvs::Empty, std_srvs::Empty>& event)
{
  return true;
}

ros::ServiceServer service = handle.advertiseService("my_service", callback);
\endverbatim
   *
   * \param service Service name to advertise on
   * \param srv_func function pointer to call when a message has arrived
   * \return On success, a ServiceServer that, when all copies of it go out of scope, will unadvertise this service.
   * On failure, an empty ServiceServer which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   * \throws InvalidNameException If the service name begins with a tilde, or is an otherwise invalid graph resource name
   */
  template<class MReq, class MRes>
  ServiceServer advertiseService(const std::string& service, bool(*srv_func)(ServiceEvent<MReq, MRes>&))
  {
    AdvertiseServiceOptions ops;
    ops.template initBySpecType<ServiceEvent<MReq, MRes> >(service, srv_func);
    return advertiseService(ops);
  }

  /**
   * \brief Advertise a service, version for arbitrary boost::function object
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name.
   *
   * This version of advertiseService allows non-class functions, as well as functor objects and boost::bind (along with anything
   * else boost::function supports).
   *
   * \param service Service name to advertise on
   * \param callback Callback to call when the service is called
   * \param tracked_object A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
   * and if the reference count goes to 0 the subscriber callbacks will not get called.
   * Note that setting this will cause a new reference to be added to the object before the
   * callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
   * thread) that the callback is invoked from.
   * \return On success, a ServiceServer that, when all copies of it go out of scope, will unadvertise this service.
   * On failure, an empty ServiceServer which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   * \throws InvalidNameException If the service name begins with a tilde, or is an otherwise invalid graph resource name
   */
  template<class MReq, class MRes>
  ServiceServer advertiseService(const std::string& service, const boost::function<bool(MReq&, MRes&)>& callback, 
                                 const VoidConstPtr& tracked_object = VoidConstPtr())
  {
    AdvertiseServiceOptions ops;
    ops.template init<MReq, MRes>(service, callback);
    ops.tracked_object = tracked_object;
    return advertiseService(ops);
  }

  /**
   * \brief Advertise a service, version for arbitrary boost::function object using ros::ServiceEvent as the callback parameter type
   *
   * Note that the template parameter S is the full event type, e.g. ros::ServiceEvent<Req, Res>
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name.
   *
   * This version of advertiseService allows non-class functions, as well as functor objects and boost::bind (along with anything
   * else boost::function supports).
   *
   * \param service Service name to advertise on
   * \param callback Callback to call when the service is called
   * \param tracked_object A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
   * and if the reference count goes to 0 the subscriber callbacks will not get called.
   * Note that setting this will cause a new reference to be added to the object before the
   * callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
   * thread) that the callback is invoked from.
   * \return On success, a ServiceServer that, when all copies of it go out of scope, will unadvertise this service.
   * On failure, an empty ServiceServer which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   * \throws InvalidNameException If the service name begins with a tilde, or is an otherwise invalid graph resource name
   */
  template<class S>
  ServiceServer advertiseService(const std::string& service, const boost::function<bool(S&)>& callback, 
                                 const VoidConstPtr& tracked_object = VoidConstPtr())
  {
    AdvertiseServiceOptions ops;
    ops.template initBySpecType<S>(service, callback);
    ops.tracked_object = tracked_object;
    return advertiseService(ops);
  }

  /**
   * \brief Advertise a service, with full range of AdvertiseServiceOptions
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name.
   *
   * This version of advertiseService allows the full set of options, exposed through the AdvertiseServiceOptions class
   *
   * \param ops Advertise options
   * \return On success, a ServiceServer that, when all copies of it go out of scope, will unadvertise this service.
   * On failure, an empty ServiceServer which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   * \throws InvalidNameException If the service name begins with a tilde, or is an otherwise invalid graph resource name
   */
  ServiceServer advertiseService(AdvertiseServiceOptions& ops);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Versions of serviceClient()
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** @brief Create a client for a service, version templated on two message types
   *
   * When the last handle reference of a persistent connection is cleared, the connection will automatically close.
   *
   * @param service_name The name of the service to connect to
   * @param persistent Whether this connection should persist.  Persistent services keep the connection to the remote host active
   *        so that subsequent calls will happen faster.  In general persistent services are discouraged, as they are not as
   *        robust to node failure as non-persistent services.
   * @param header_values Key/value pairs you'd like to send along in the connection handshake
   * \throws InvalidNameException If the service name begins with a tilde, or is an otherwise invalid graph resource name
   */
  template<class MReq, class MRes>
  ServiceClient serviceClient(const std::string& service_name, bool persistent = false, 
                              const M_string& header_values = M_string())
  {
    ServiceClientOptions ops;
    ops.template init<MReq, MRes>(service_name, persistent, header_values);
    return serviceClient(ops);
  }

  /** @brief Create a client for a service, version templated on service type
   *
   * When the last handle reference of a persistent connection is cleared, the connection will automatically close.
   *
   * @param service_name The name of the service to connect to
   * @param persistent Whether this connection should persist.  Persistent services keep the connection to the remote host active
   *        so that subsequent calls will happen faster.  In general persistent services are discouraged, as they are not as
   *        robust to node failure as non-persistent services.
   * @param header_values Key/value pairs you'd like to send along in the connection handshake
   * \throws InvalidNameException If the service name begins with a tilde, or is an otherwise invalid graph resource name
   */
  template<class Service>
  ServiceClient serviceClient(const std::string& service_name, bool persistent = false, 
                              const M_string& header_values = M_string())
  {
    ServiceClientOptions ops;
    ops.template init<Service>(service_name, persistent, header_values);
    return serviceClient(ops);
  }

  /** @brief Create a client for a service, version with full range of ServiceClientOptions
   *
   * When the last handle reference of a persistent connection is cleared, the connection will automatically close.
   *
   * @param ops The options for this service client
   * \throws InvalidNameException If the service name begins with a tilde, or is an otherwise invalid graph resource name
   */
  ServiceClient serviceClient(ServiceClientOptions& ops);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Versions of createTimer()
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Create a timer which will call a callback at the specified rate.  This variant takes
   * a class member function, and a bare pointer to the object to call the method on.
   *
   * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
   * be stopped, and the callback will no longer be called.
   *
   * \param r The rate at which to call the callback
   * \param callback The method to call
   * \param obj The object to call the method on
   * \param oneshot If true, this timer will only fire once
   * \param autostart If true (default), return timer that is already started
   */
  template<class Handler, class Obj>
  Timer createTimer(Rate r, Handler h, Obj o, bool oneshot = false, bool autostart = true) const
  {
    return createTimer(r.expectedCycleTime(), h, o, oneshot, autostart);
  }

  /**
   * \brief Create a timer which will call a callback at the specified rate.  This variant takes
   * a class member function, and a bare pointer to the object to call the method on.
   *
   * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
   * be stopped, and the callback will no longer be called.
   *
   * \param period The period at which to call the callback
   * \param callback The method to call
   * \param obj The object to call the method on
   * \param oneshot If true, this timer will only fire once
   * \param autostart If true (default), return timer that is already started
   */
  template<class T>
  Timer createTimer(Duration period, void(T::*callback)(const TimerEvent&) const, T* obj, 
                    bool oneshot = false, bool autostart = true) const
  {
    return createTimer(period, boost::bind(callback, obj, _1), oneshot, autostart);
  }

  /**
   * \brief Create a timer which will call a callback at the specified rate.  This variant takes
   * a class member function, and a bare pointer to the object to call the method on.
   *
   * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
   * be stopped, and the callback will no longer be called.
   *
   * \param period The period at which to call the callback
   * \param callback The method to call
   * \param obj The object to call the method on
   * \param oneshot If true, this timer will only fire once
   * \param autostart If true (default), return timer that is already started
   */
  template<class T>
  Timer createTimer(Duration period, void(T::*callback)(const TimerEvent&), T* obj, 
                    bool oneshot = false, bool autostart = true) const
  {
    return createTimer(period, boost::bind(callback, obj, _1), oneshot, autostart);
  }

  /**
   * \brief Create a timer which will call a callback at the specified rate.  This variant takes
   * a class member function, and a shared pointer to the object to call the method on.
   *
   * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
   * be stopped, and the callback will no longer be called.
   *
   * \param period The period at which to call the callback
   * \param callback The method to call
   * \param obj The object to call the method on.  Since this is a shared pointer, the object will
   * automatically be tracked with a weak_ptr so that if it is deleted before the Timer goes out of
   * scope the callback will no longer be called (and therefore will not crash).
   * \param oneshot If true, this timer will only fire once
   * \param autostart If true (default), return timer that is already started
   */
  template<class T>
  Timer createTimer(Duration period, void(T::*callback)(const TimerEvent&), const boost::shared_ptr<T>& obj, 
                    bool oneshot = false, bool autostart = true) const
  {
    TimerOptions ops(period, boost::bind(callback, obj.get(), _1), 0);
    ops.tracked_object = obj;
    ops.oneshot = oneshot;
    ops.autostart = autostart;
    return createTimer(ops);
  }

  /**
   * \brief Create a timer which will call a callback at the specified rate.  This variant takes
   * anything that can be bound to a Boost.Function, including a bare function
   *
   * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
   * be stopped, and the callback will no longer be called.
   *
   * \param period The period at which to call the callback
   * \param callback The function to call
   * \param oneshot If true, this timer will only fire once
   * \param autostart If true (default), return timer that is already started
   */
  Timer createTimer(Duration period, const TimerCallback& callback, bool oneshot = false,
                    bool autostart = true) const;

  /**
   * \brief Create a timer which will call a callback at the specified rate.  This variant allows
   * the full range of TimerOptions.
   *
   * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
   * be stopped, and the callback will no longer be called.
   *
   * \param ops The options to use when creating the timer
   */
  Timer createTimer(TimerOptions& ops) const;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Versions of createWallTimer()
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * \brief Create a timer which will call a callback at the specified rate, using wall time to determine
   * when to call the callback instead of ROS time.
   * This variant takes a class member function, and a bare pointer to the object to call the method on.
   *
   * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
   * be stopped, and the callback will no longer be called.
   *
   * \param period The period at which to call the callback
   * \param callback The method to call
   * \param obj The object to call the method on
   * \param oneshot If true, this timer will only fire once
   * \param autostart If true (default), return timer that is already started
   */
  template<class T>
  WallTimer createWallTimer(WallDuration period, void(T::*callback)(const WallTimerEvent&), T* obj, 
                            bool oneshot = false, bool autostart = true) const
  {
    return createWallTimer(period, boost::bind(callback, obj, _1), oneshot, autostart);
  }

  /**
   * \brief Create a timer which will call a callback at the specified rate, using wall time to determine
   * when to call the callback instead of ROS time.  This variant takes
   * a class member function, and a shared pointer to the object to call the method on.
   *
   * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
   * be stopped, and the callback will no longer be called.
   *
   * \param period The period at which to call the callback
   * \param callback The method to call
   * \param obj The object to call the method on.  Since this is a shared pointer, the object will
   * automatically be tracked with a weak_ptr so that if it is deleted before the Timer goes out of
   * scope the callback will no longer be called (and therefore will not crash).
   * \param oneshot If true, this timer will only fire once
   */
  template<class T>
  WallTimer createWallTimer(WallDuration period, void(T::*callback)(const WallTimerEvent&), 
                            const boost::shared_ptr<T>& obj, 
                            bool oneshot = false, bool autostart = true) const
  {
    WallTimerOptions ops(period, boost::bind(callback, obj.get(), _1), 0);
    ops.tracked_object = obj;
    ops.oneshot = oneshot;
    ops.autostart = autostart;
    return createWallTimer(ops);
  }

  /**
   * \brief Create a timer which will call a callback at the specified rate, using wall time to determine
   * when to call the callback instead of ROS time.  This variant takes
   * anything that can be bound to a Boost.Function, including a bare function
   *
   * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
   * be stopped, and the callback will no longer be called.
   *
   * \param period The period at which to call the callback
   * \param callback The function to call
   * \param oneshot If true, this timer will only fire once
   */
  WallTimer createWallTimer(WallDuration period, const WallTimerCallback& callback, 
                            bool oneshot = false, bool autostart = true) const;

  /**
   * \brief Create a timer which will call a callback at the specified rate, using wall time to determine
   * when to call the callback instead of ROS time.  This variant allows
   * the full range of TimerOptions.
   *
   * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
   * be stopped, and the callback will no longer be called.
   *
   * \param ops The options to use when creating the timer
   */
  WallTimer createWallTimer(WallTimerOptions& ops) const;

  /** \brief Set an arbitrary XML/RPC value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param v The value to be inserted.
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  void setParam(const std::string& key, const XmlRpc::XmlRpcValue& v) const;
  /** \brief Set a string value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param s The value to be inserted.
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  void setParam(const std::string& key, const std::string& s) const;
  /** \brief Set a string value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param s The value to be inserted.
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  void setParam(const std::string& key, const char* s) const;
  /** \brief Set a double value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param d The value to be inserted.
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  void setParam(const std::string& key, double d) const;
  /** \brief Set a integer value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param i The value to be inserted.
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  void setParam(const std::string& key, int i) const;
  /** \brief Set a integer value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param b The value to be inserted.
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  void setParam(const std::string& key, bool b) const;

  /** \brief Get a string value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] s Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  bool getParam(const std::string& key, std::string& s) const;
  /** \brief Get a double value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] d Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  bool getParam(const std::string& key, double& d) const;
  /** \brief Get a integer value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] i Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  bool getParam(const std::string& key, int& i) const;
  /** \brief Get a boolean value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] b Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  bool getParam(const std::string& key, bool& b) const;
  /** \brief Get an arbitrary XML/RPC value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] v Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  bool getParam(const std::string& key, XmlRpc::XmlRpcValue& v) const;

  /** \brief Get a string value from the parameter server, with local caching
   *
   * This method will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] s Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  bool getParamCached(const std::string& key, std::string& s) const;
  /** \brief Get a double value from the parameter server, with local caching
   *
   * This method will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] d Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  bool getParamCached(const std::string& key, double& d) const;
  /** \brief Get a integer value from the parameter server, with local caching
   *
   * This method will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] i Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  bool getParamCached(const std::string& key, int& i) const;
  /** \brief Get a boolean value from the parameter server, with local caching
   *
   * This method will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] b Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  bool getParamCached(const std::string& key, bool& b) const;
  /** \brief Get an arbitrary XML/RPC value from the parameter server, with local caching
   *
   * This method will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] v Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  bool getParamCached(const std::string& key, XmlRpc::XmlRpcValue& v) const;

  /** \brief Check whether a parameter exists on the parameter server.
   *
   * \param key The key to check.
   *
   * \return true if the parameter exists, false otherwise
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  bool hasParam(const std::string& key) const;
  /** \brief Search up the tree for a parameter with a given key
   *
   * This function parameter server's searchParam feature to search up the tree for
   * a parameter.  For example, if the parameter server has a parameter [/a/b]
   * and you're in the namespace [/a/c/d], searching for the parameter "b" will
   * yield [/a/b].  If [/a/c/d/b] existed, that parameter would be returned instead.
   *
   * \param key the parameter to search for
   * \param [out] result the found value (if any)
   *
   * \return true if the parameter was found, false otherwise.
   */
  bool searchParam(const std::string& key, std::string& result) const;
  /** \brief Delete a parameter from the parameter server.
   *
   * \param key The key to delete.
   *
   * \return true if the deletion succeeded, false otherwise.
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  bool deleteParam(const std::string& key) const;

  /** \brief Assign value from parameter server, with default.
   *
   * This method tries to retrieve the indicated parameter value from the
   * parameter server, storing the result in param_val.  If the value
   * cannot be retrieved from the server, default_val is used instead.
   *
   * \param param_name The key to be searched on the parameter server.
   * \param[out] param_val Storage for the retrieved value.
   * \param default_val Value to use if the server doesn't contain this
   * parameter.
   * \throws InvalidNameException If the parameter key begins with a tilde, or is an otherwise invalid graph resource name
   */
  template<typename T>
  void param(const std::string& param_name, T& param_val, const T& default_val) const
  {
    if (hasParam(param_name))
    {
      if (getParam(param_name, param_val))
      {
        return;
      }
    }

    param_val = default_val;
  }

  /**
   * \brief Shutdown every handle created through this NodeHandle.
   *
   * This method will unadvertise every topic and service advertisement,
   * and unsubscribe every subscription created through this NodeHandle.
   */
  void shutdown();

  /** \brief Check whether it's time to exit.
   *
   * This method checks to see if both ros::ok() is true and shutdown() has not been called on this NodeHandle, to see whether it's yet time
   * to exit.  ok() is false once either ros::shutdown() or NodeHandle::shutdown() have been called
   *
   * \return true if we're still OK, false if it's time to exit
   */
  bool ok() const;

private:
  struct no_validate { };
  // this is pretty awful, but required to preserve public interface (and make minimum possible changes)
  std::string resolveName(const std::string& name, bool remap, no_validate) const;

  void construct(const std::string& ns, bool validate_name);
  void destruct();

  void initRemappings(const M_string& remappings);

  std::string remapName(const std::string& name) const;

  std::string namespace_;
  std::string unresolved_namespace_;
  M_string remappings_;
  M_string unresolved_remappings_;

  CallbackQueueInterface* callback_queue_;

  NodeHandleBackingCollection* collection_;

  bool ok_;
};

}

#endif // ROSCPP_NODE_HANDLE_H
