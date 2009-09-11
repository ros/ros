/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
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

#ifndef ROSCPP_NODE_H
#define ROSCPP_NODE_H

#include <csignal>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <stdarg.h>
#include "ros/common.h"
#include "ros/forwards.h"
#include "ros/time.h"
#include "roslib/Time.h"
#include "ros/message.h"
#include "ros/service.h"
#include "ros/poll_set.h"
#include "ros/subscription_message_helper.h"
#include "ros/service_message_helper.h"
#include "ros/init.h"
#include "ros/advertise_options.h"
#include "ros/advertise_service_options.h"
#include "ros/subscribe_options.h"
#include "ros/single_subscriber_publisher.h"
#include "ros/callback_queue.h"

#include "XmlRpc.h"

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>

namespace ros
{

/** \deprecated Use NodeHandle instead
 */
class Node
{
public:
  /** @brief Flag for node() constructor
   *
   * Use this option in the ros::Node() constructor to tell ROS not to install a
   * SIGINT handler.  You should install your own SIGINT handler in this
   * case, to ensure that the node is deleted application exits.
   *
   * \deprecated
   */
  static const uint32_t DONT_HANDLE_SIGINT = ros::init_options::NoSigintHandler;
  /** @brief Flag for node() constructor
   *
   * Use this option in the ros::Node() constructor to tell ROS to add a
   * random number to the end of your node's name, to make it unique.
   *
   * \deprecated
   */
  static const uint32_t ANONYMOUS_NAME = ros::init_options::AnonymousName;
  /** \deprecated
   */
  static const uint32_t DONT_START_SERVER_THREAD = 0;

  /**
   * \brief Flag for node() constructor
   *
   * Use this option in the ros::Node() constructor to tell ROS to not
   * add the rosout appender
   *
   * \deprecated
   */
  static const uint32_t DONT_ADD_ROSOUT_APPENDER = ros::init_options::NoRosout;

  /** @brief Node constructor
   *
   * @param name The node's name.
   * @param options A set of flags, ORed together.  The valid flags
   *                  are ros::Node::DONT_HANDLE_SIGINT, ros::Node::ANONYMOUS_NAME
   * @param master_retry_timeout The maximum time this node should spend looping
   *                              trying to connect to the master, in milliseconds
   *
   * \deprecated
   */
  ROSCPP_DEPRECATED Node(std::string name, uint32_t options = 0, int32_t master_retry_timeout = -1);
  Node();
  /** @brief Node destructor */
  virtual ~Node();
  /** @brief Get pointer to global node object.
   *
   * @return Pointer to global node object.
   *
   * \deprecated
   */
  ROSCPP_DEPRECATED static Node *instance();
  /** @brief Check whether it's time to exit.
   *
   * This method checks the value of \ref ok_, to see whether it's yet time
   * to exit.
   *
   * @return true if we're still OK, false if it's time to exit
   *
   * \deprecated Use ros::ok() or NodeHandle::ok()
   */
  ROSCPP_DEPRECATED bool ok() const;
  /** @brief Check whether we're in the middle of shutting everything down
   *
   * @return true if we're shutting down, false otherwise
   *
   * \deprecated
   */
  ROSCPP_DEPRECATED inline bool shuttingDown() const
  {
    return shutting_down_;
  }
  /** @brief Get the node's name.
   *
   * @return The node's name
   * \deprecated use this_node::getName()
   */
  ROSCPP_DEPRECATED const std::string &getName() const;
  /** \deprecated use this_node::getNamespace() */
  ROSCPP_DEPRECATED const std::string& getNamespace() const;

  /**
   * @brief Set the max time this node should spend looping trying to connect to the master
   * @param milliseconds the timeout, in milliseconds.  A value of -1 means infinite
   *
   * \deprecated use master::setRetryTimeout()
   */
  ROSCPP_DEPRECATED void setMasterRetryTimeout(int32_t milliseconds);

  /** @brief Get the list of topics advertised by this node
   *
   * @param[out] topics The advertised topics
   *
   * \deprecated use this_node::getAdvertisedTopics()
   */
  ROSCPP_DEPRECATED void getAdvertisedTopics(V_string& topics);

  /** @brief Get the list of topics subscribed to by this node
   *
   * @param[out] The subscribed topics
   *
   * \deprecated use this_node::getSubscribedTopics()
   */
  ROSCPP_DEPRECATED void getSubscribedTopics(V_string& topics);

  /** @brief Advertise a topic.
   *
   * This call connects to the master to publicize that the node will be
   * publishing messages on the given topic.  This method must be called
   * prior to publishing messages, for each topic.
   *
   * @param "class T" The type of the message to be published.
   * @param topic The topic to be published on.
   * @param max_queue Maximum number of outgoing messages to be queued
   * for delivery to subscribers
   *
   * @return true on successful advertisement, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class T>
  ROSCPP_DEPRECATED bool advertise(const std::string &topic, size_t max_queue)
  {
    AdvertiseOptions ops;
    ops.init<T>(resolveName(topic), max_queue);
    return advertise(ops);
  }

  /** @brief Advertise a topic.
   *
   * @note The templated version of this call is preferred.  This version
   * is provided for special situations where the templated version doesn't
   * work (e.g., inside log playback).
   *
   * This call connects to the master to publicize that the node will be
   * publishing messages on the given topic.  This method must be called
   * prior to publishing messages, for each topic.
   *
   * @param topic  The topic to be published on.
   * @param msgref  Message instance of the type that will be published.
   * @param max_queue  Maximum number of outgoing messages to be queued
   * for delivery to subscribers
   *
   * @return true on successful advertisement, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool advertise(const std::string &topic, const Message& msgref, size_t max_queue);

  /** @brief Advertise a topic, with subscription callbacks.
   *
   * This call connects to the master to publicize that the node will be
   * publishing messages on the given topic.  This method must be called
   * prior to publishing messages, for each topic.
   *
   * This version of advertise accepts callback functions that will be
   * invoked when subscribers connect and disconnect on the given topic.
   *
   * @param "class T" The type of the message to be published.
   * @param topic The topic to be published on.
   * @param msgref  Message instance of the type that will be published.
   * @param sub_connect Callback to be invoked on subscriber connection.
   * @param sub_disconnect Callback to be invoked on subscriber disconnection
   * @param max_queue Maximum number of outgoing messages to be queued
   * for delivery to subscribers
   *
   * @return true on successful advertisement, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class T>
  ROSCPP_DEPRECATED bool advertise(const std::string &_topic, const SubscriberStatusCallback& sub_connect,
                 const SubscriberStatusCallback& sub_disconnect, size_t max_queue)
  {
    AdvertiseOptions ops;
    ops.init<T>(resolveName(_topic), max_queue, sub_connect, sub_disconnect);
    return advertise(ops);
  }

  /** @brief Advertise a topic, with subscription callbacks.
   *
   * This call connects to the master to publicize that the node will be
   * publishing messages on the given topic.  This method must be called
   * prior to publishing messages, for each topic.
   *
   * This version of advertise accepts callback functions that will be
   * invoked when subscribers connect on the given topic.
   *
   * @param "class T" The type of the message to be published.
   * @param topic The topic to be published on.
   * @param msgref  Message instance of the type that will be published.
   * @param sub_connect Callback to be invoked on subscriber connection.
   * @param max_queue Maximum number of outgoing messages to be queued
   * for delivery to subscribers
   *
   * @return true on successful advertisement, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class T>
  ROSCPP_DEPRECATED bool advertise(const std::string &_topic, const SubscriberStatusCallback& sub_connect,
                 size_t max_queue)
  {
    AdvertiseOptions ops;
    ops.init<T>(resolveName(_topic), max_queue, sub_connect);
    return advertise(ops);
  }

  /** @brief Advertise a topic, with subscription callbacks.
   *
   * This call connects to the master to publicize that the node will be
   * publishing messages on the given topic.  This method must be called
   * prior to publishing messages, for each topic.
   *
   * This version of advertise accepts callback functions that will be
   * invoked when subscribers connect and disconnect on the given topic.
   *
   * @param "class T" The node-derived class on which the callback will be called
   * @param topic The topic to be published on.
   * @param msgref  Message instance of the type that will be published.
   * @param sub_connect Callback to be invoked on subscriber connection.
   * @param sub_disconnect Callback to be invoked on subscriber disconnection
   * @param max_queue Maximum number of outgoing messages to be queued
   * for delivery to subscribers
   *
   * @return true on successful advertisement, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class T>
  ROSCPP_DEPRECATED bool advertise(const std::string &_topic, const Message& msgref,
      void(T::*sub_connect)(const SingleSubscriberPublisher&), void(T::*sub_disconnect)(
          const SingleSubscriberPublisher&), size_t max_queue)
  {
    AdvertiseOptions ops(resolveName(_topic),
                     max_queue,
                     msgref.__getMD5Sum(),
                     msgref.__getDataType(),
                     msgref.__getMessageDefinition(),
                     boost::bind(sub_connect, static_cast<T*>(this), _1),
                     boost::bind(sub_disconnect, static_cast<T*>(this), _1));
    return advertise(ops);
  }

  // because the template instantiation doesn't work for a NULL disconnect callback pointer,
  // I have to provide this version as well... :(   is anybody using this feature anyway??
  /** @brief Advertise a topic, with a subscription connection callback.
   *
   * This call connects to the master to publicize that the node will be
   * publishing messages on the given topic.  This method must be called
   * prior to publishing messages, for each topic.
   *
   * This version of advertise accepts a callback function that will be
   * invoked when subscribers connect on the given topic.
   *
   * @param "class T" The node-derived class on which the callback will be called
   * @param topic The topic to be published on.
   * @param msgref  Message instance of the type that will be published.
   * @param sub_connect Callback to be invoked on subscriber connection.
   * @param max_queue Maximum number of outgoing messages to be queued
   * for delivery to subscribers
   *
   * @return true on successful advertisement, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class T>
  ROSCPP_DEPRECATED bool advertise(const std::string &_topic, const Message& msgref,
      void(T::*sub_connect)(const SingleSubscriberPublisher&), size_t max_queue)
  {
    AdvertiseOptions ops(resolveName(_topic),
                     max_queue,
                     msgref.__getMD5Sum(),
                     msgref.__getDataType(),
                     msgref.__getMessageDefinition(),
                     boost::bind(sub_connect, static_cast<T*>(this), _1));
    return advertise(ops);
  }

  /** @brief Unadvertise a topic.
   *
   * This call unadvertises a topic.  You must have previously called
   * advertise() with the given topic.
   *
   * @param topic The topic to be unadvertised
   *
   * @return true on successful unadvertisement, false otherwise
   */
  ROSCPP_DEPRECATED bool unadvertise(const std::string &topic, const SubscriberCallbacksPtr& callbacks = SubscriberCallbacksPtr());

  /** @brief Advertise a service.
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name.
   *
   * @param topic The name of the service.
   * @param srv_func A callback to be invoked when a client uses this
   * service.
   * @param thread_pool_size Size of thread pool that will be used to invoke
   * callbacks on this service.  Set to 0 for no threads; in this case the
   * callback will be invoked in the ROS thread.  Set to n > 0 to allocate
   * n threads in which callbacks for concurrent clients will be invoked.
   * Set n to -1 to guarantee at least as many worker threads as there are
   * concurrent clients.
   *
   * @return true on successful advertisement, false otherwise
   *
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class T, class MReq, class MRes>
  ROSCPP_DEPRECATED bool advertiseService(const std::string &topic, bool(T::*srv_func)(MReq &,
      MRes &), int thread_pool_size = 1)
  {
    std::string mapped_name = resolveName(topic);
    if (MReq::__s_getServerMD5Sum() != MRes::__s_getServerMD5Sum())
    {
      ROS_FATAL("woah! the request and response parameters to the server "
        "callback function must be autogenerated from the same "
        "server definition file (.srv). your advertise_servce "
        "call for %s appeared to use request/response types "
        "from different .srv files.", topic.c_str());
      ROS_BREAK();
      return false;
    }

    AdvertiseServiceOptions ops;
    ops.init<MReq, MRes>(mapped_name, boost::bind(srv_func, static_cast<T*>(this), _1, _2));
    return advertiseService(ops, thread_pool_size);
  }

  /** @brief Advertise a service with an arbitrary member function pointer
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name. This form of the function
   * allows you to use member functions of objects that are not derivatives
   * of ros::Node.
   *
   * @param topic The name of the service.
   * @param srv_func A callback to be invoked when a client uses this
   * service.
   * @param obj The object in which srv_func is defined.
   * @param thread_pool_size Size of thread pool that will be used to invoke
   * callbacks on this service.  Set to 0 for no threads; in this case the
   * callback will be invoked in the ROS thread.  Set to n > 0 to allocate
   * n threads in which callbacks for concurrent clients will be invoked.
   * Set n to -1 to guarantee at least as many worker threads as there are
   * concurrent clients.
   *
   * @return true on successful advertisement, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class T, class MReq, class MRes>
  ROSCPP_DEPRECATED bool advertiseService(const std::string &topic, bool(T::*srv_func)(MReq &,
      MRes &), T *obj, int thread_pool_size = 1)
  {
    std::string mapped_name = resolveName(topic);

    if (MReq::__s_getServerMD5Sum() != MRes::__s_getServerMD5Sum())
    {
      ROS_FATAL("woah! the request and response parameters to the server "
        "callback function must be autogenerated from the same "
        "server definition file (.srv). your advertise_servce "
        "call for %s appeared to use request/response types "
        "from different .srv files.", topic.c_str());
      ROS_BREAK();
    }

    AdvertiseServiceOptions ops;
    ops.init<MReq, MRes>(mapped_name, boost::bind(srv_func, obj, _1, _2));
    return advertiseService(ops, thread_pool_size);
  }

  /** @brief Advertise a service with an arbitrary member function pointer
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name. This form of the function
   * allows you to use member functions of objects that are not derivatives
   * of ros::Node.
   *
   * @param topic The name of the service.
   * @param srv_func A callback to be invoked when a client uses this
   * service.
   * @param obj The object in which srv_func is defined.
   * @param thread_pool_size Size of thread pool that will be used to invoke
   * callbacks on this service.  Set to 0 for no threads; in this case the
   * callback will be invoked in the ROS thread.  Set to n > 0 to allocate
   * n threads in which callbacks for concurrent clients will be invoked.
   * Set n to -1 to guarantee at least as many worker threads as there are
   * concurrent clients.
   *
   * @return true on successful advertisement, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class MReq, class MRes>
  ROSCPP_DEPRECATED bool advertiseService(const std::string &topic, bool (*srv_func)(MReq&, MRes&), int thread_pool_size = 1)
  {
    std::string mapped_name = resolveName(topic);

    if (MReq::__s_getServerMD5Sum() != MRes::__s_getServerMD5Sum())
    {
      ROS_FATAL("woah! the request and response parameters to the server "
        "callback function must be autogenerated from the same "
        "server definition file (.srv). your advertise_servce "
        "call for %s appeared to use request/response types "
        "from different .srv files.", topic.c_str());
      ROS_BREAK();
    }

    AdvertiseServiceOptions ops;
    ops.init<MReq, MRes>(mapped_name, srv_func);
    return advertiseService(ops, thread_pool_size);
  }

  /** @brief Advertise a service.
   *
   * This version takes a request/response object, so that, for example,
   * scripting interfaces can call this function with a generic object that
   * only has the non-static functions available.
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name.
   *
   * @param topic The name of the service.
   * @param srv_func A callback to be invoked when a client uses this
   * service.
   * @param thread_pool_size Size of thread pool that will be used to invoke
   * callbacks on this service.  Set to 0 for no threads; in this case the
   * callback will be invoked in the ROS thread.  Set to n > 0 to allocate
   * n threads in which callbacks for concurrent clients will be invoked.
   * Set n to -1 to guarantee at least as many worker threads as there are
   * concurrent clients.
   *
   * @return true on successful advertisement, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class T, class MReq, class MRes>
  ROSCPP_DEPRECATED bool advertiseService(const std::string &topic, bool(T::*srv_func)(MReq &,
      MRes &), const MReq& req, const MRes& res, int thread_pool_size = 1)
  {
    std::string mapped_name = resolveName(topic);
    if (req.__getServerMD5Sum() != res.__getServerMD5Sum())
    {
      ROS_FATAL("woah! the request and response parameters to the server "
        "callback function must be autogenerated from the same "
        "server definition file (.srv). your advertise_servce "
        "call for %s appeared to use request/response types "
        "from different .srv files.", topic.c_str());
      ROS_BREAK();
      return false;
    }

    AdvertiseServiceOptions ops(mapped_name,
                                ServiceMessageHelperPtr(new ServiceMessageHelperT<MReq, MRes> (
                                                        boost::bind(srv_func, static_cast<T*>(this), _1, _2),
                                                        req.__getServerMD5Sum(),
                                                        req.__getServiceDataType(),
                                                        req.__getDataType(),
                                                        res.__getDataType())));
    return advertiseService(ops, thread_pool_size);
  }

  /** @brief Advertise a service with an arbitrary member function pointer
   *
   * This call connects to the master to publicize that the node will be
   * offering an RPC service with the given name. This form of the function
   * allows you to use member functions of objects that are not derivatives
   * of ros::Node.
   *
   * @param topic The name of the service.
   * @param srv_func A callback to be invoked when a client uses this
   * service.
   * @param obj The object in which srv_func is defined.
   * @param thread_pool_size Size of thread pool that will be used to invoke
   * callbacks on this service.  Set to 0 for no threads; in this case the
   * callback will be invoked in the ROS thread.  Set to n > 0 to allocate
   * n threads in which callbacks for concurrent clients will be invoked.
   * Set n to -1 to guarantee at least as many worker threads as there are
   * concurrent clients.
   *
   * @return true on successful advertisement, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class T, class MReq, class MRes>
  ROSCPP_DEPRECATED bool advertiseService(const std::string &topic, bool(T::*srv_func)(MReq &,
      MRes &), T *obj, const MReq& req, const MRes& res, int thread_pool_size =
      1)
  {
    std::string mapped_name = resolveName(topic);

    if (req.__getServerMD5Sum() != res.__getServerMD5Sum())
    {
      ROS_FATAL("woah! the request and response parameters to the server "
        "callback function must be autogenerated from the same "
        "server definition file (.srv). your advertise_servce "
        "call for %s appeared to use request/response types "
        "from different .srv files.", topic.c_str());
      ROS_BREAK();
    }

    AdvertiseServiceOptions ops(mapped_name,
                                ServiceMessageHelperPtr(new ServiceMessageHelperT<MReq, MRes> (
                                                        boost::bind(srv_func, obj, _1, _2),
                                                        req.__getServerMD5Sum(),
                                                        req.__getServiceDataType(),
                                                        req.__getDataType(),
                                                        res.__getDataType())));
    return advertiseService(ops, thread_pool_size);
  }

  /** @brief Unadvertise a service.
   *
   * This call unadvertises a service, which must have been previously
   * advertised, using advertiseService().
   *
   * After this call completes, it is guaranteed that no further
   * callbacks will be invoked for this service.
   *
   * This method can be safely called from within a service callback.
   *
   * @param serv_name The service to be unadvertised.
   *
   * @return true on successful unadvertisement, false otherwise.
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool unadvertiseService(const std::string &serv_name);

  /** @brief Publish a message.
   *
   * This method publishes a message on a topic, delivering it to any
   * currently connected subscribers.  If no subscribers are connected,
   * this call does nothing.
   *
   * You must have already called \ref advertise()
   * on the topic you are trying to publish to, and the type supplied in
   * the advertise() call must match the type of the message you are trying
   * to publish.
   *
   * This method can be safely called from within a subscriber connection
   * callback.
   *
   * @param _topic The topic to publish to.
   * @param msg Message to be published.
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED void publish(const std::string &_topic, const Message& m);

  ROSCPP_DEPRECATED void publish(const PublicationPtr& p, const Message& m);

  /** @brief Return the number of subscriptions the node has:
   *
   * @return number of subscriptions
   *
   * \deprecated Was essentially useless, file a ticket if you'd like to be able to get the number of topics this node is subscribed to without getting any information about them.  If you want information about them, use this_node::getSubscribedTopics()
   */
  ROSCPP_DEPRECATED size_t numSubscriptions();

  /** @brief Return the number of subscribers a node has for a particular topic:
   *
   * @param _topic The topic name to check
   *
   * @return number of subscribers
   *
   * \deprecated Use the NodeHandle API's Publisher::getNumSubscribers() instead
   */
  ROSCPP_DEPRECATED size_t numSubscribers(const std::string &_topic);

  /** @brief Subscribe to a topic, possibly from outside the node object.
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, _msg is filled with
   * the new data and fp is invoked.
   *
   * This method can be called from outside the node object.
   *
   * @param _topic The topic to be subscribed
   * @param _msg A message object into which incoming messages will be
   * written, prior to invoking the callback
   * @param fp A callback to be invoked when a new message is received.
   * @param obj A pointer to the object within which the callback is defined.
   * @param max_queue Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   *
   * @return true on success, false otherwise (subscribe failed)
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class M, class T>
  ROSCPP_DEPRECATED bool subscribe(const std::string &_topic, M &_msg, void(T::*fp)(), T* obj,
      int max_queue)
  {
    SubscribeOptions ops(resolveName(_topic), max_queue, _msg.__getMD5Sum(), _msg.__getDataType());
    AbstractFunctor *afp = new MethodFunctor<T> (obj, fp);
    return subscribe(ops, &_msg, afp);
  }

  /** @brief Subscribe to a topic.
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, _msg is filled with
   * the new data and fp is invoked.
   *
   * @param _topic The topic to be subscribed
   * @param _msg A message object into which incoming messages will be
   * written, prior to invoking the callback. The message is automatically locked
   * for the duration of the callback.
   * @param fp A callback to be invoked when a new message is received.
   * @param max_queue Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   *
   * @return true on success, false otherwise (subscribe failed)
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class M, class T>
  ROSCPP_DEPRECATED bool subscribe(const std::string &_topic, M &_msg, void(T::*fp)(),
      int max_queue)
  {
    SubscribeOptions ops(resolveName(_topic), max_queue, _msg.__getMD5Sum(), _msg.__getDataType());
    AbstractFunctor *afp = new MethodFunctor<T> (static_cast<T*> (this), fp);
    return subscribe(ops, &_msg, afp);
  }

  /** @brief Subscribe to a topic.
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, _msg is filled with
   * the new data and fp is invoked.
   *
   * @param _topic The topic to be subscribed
   * @param _msg A message object into which incoming messages will be
   * written, prior to invoking the callback. The message is automatically locked
   * for the duration of the callback.
   * @param fp A callback to be invoked when a new message is received.
   * @param max_queue Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   *
   * @return true on success, false otherwise (subscribe failed)
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class M>
  ROSCPP_DEPRECATED bool subscribe(const std::string &_topic, M &_msg, void(*fp)(),
      int max_queue)
  {
    SubscribeOptions ops(resolveName(_topic), max_queue, _msg.__getMD5Sum(), _msg.__getDataType());
    AbstractFunctor *afp = new FunctionFunctor(fp);
    return subscribe(ops, &_msg, afp);
  }

  /** @brief Subscribe to a topic, with user data for callback.
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, _msg is filled with
   * the new data and fp is invoked.
   *
   * This form accepts a void* that will be passed into the user's
   * callback.
   *
   * @param _topic The topic to be subscribed
   * @param _msg A message object into which incoming messages will be
   * written, prior to invoking the callback. The message is automatically locked
   * for the duration of the callback.
   * @param fp A callback to be invoked when a new message is received.
   * @param user_data Opaque data to be provided to callback.
   * @param max_queue Number of incoming messages to queue up for
   *
   * @return true on success, false otherwise (subscribe failed)
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class M>
  ROSCPP_DEPRECATED bool subscribe(const std::string &_topic, M &_msg, void (*fp)(void *),
      void *user_data, int max_queue)
  {
    SubscribeOptions ops(resolveName(_topic), max_queue, _msg.__getMD5Sum(), _msg.__getDataType());
    AbstractFunctor *afp = new FunctionFunctor(fp, user_data);
    return subscribe(ops, &_msg, afp);
  }

  /** @brief Subscribe to a topic, with user data for callback.
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, _msg is filled with
   * the new data and fp is invoked.
   *
   * This form accepts a void* that will be passed into the user's
   * callback.
   *
   * @param _topic The topic to be subscribed
   * @param _msg A message object into which incoming messages will be
   * written, prior to invoking the callback. The message is automatically locked
   * for the duration of the callback.
   * @param fp A callback to be invoked when a new message is received.
   * @param user_data Opaque data to be provided to callback.
   * @param max_queue Number of incoming messages to queue up for
   *
   * @return true on success, false otherwise (subscribe failed)
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class M, class T>
  ROSCPP_DEPRECATED bool subscribe(const std::string &_topic, M &_msg, void(T::*fp)(void *),
      void *user_data, int max_queue)
  {
    SubscribeOptions ops(resolveName(_topic), max_queue, _msg.__getMD5Sum(), _msg.__getDataType());
    AbstractFunctor *afp = new MethodFunctor<T> (static_cast<T*> (this), fp,
        user_data);
    return subscribe(ops, &_msg, afp);
  }

  /** @brief Subscribe to a topic, with user data for callback, possibly
   * from outside the node object.
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, _msg is filled with
   * the new data and fp is invoked.
   *
   * This form accepts a void* that will be passed into the user's
   * callback.
   *
   * This method can be called from outside the node object.
   *
   * @param _topic The topic to be subscribed
   * @param _msg A message object into which incoming messages will be
   * written, prior to invoking the callback. The message is automatically locked
   * for the duration of the callback.
   * @param fp A callback to be invoked when a new message is received.
   * @param obj A pointer to the object within which the callback is defined.
   * @param user_data Opaque data to be provided to callback.
   * @param max_queue Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   *
   * @return true on success, false otherwise (subscribe failed)
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class M, class T>
  ROSCPP_DEPRECATED bool subscribe(const std::string &_topic, M &_msg, void(T::*fp)(void *),
      T *obj, void *user_data, int max_queue)
  {
    SubscribeOptions ops(resolveName(_topic), max_queue, _msg.__getMD5Sum(), _msg.__getDataType());
    AbstractFunctor *afp = new MethodFunctor<T> (obj, fp, user_data);
    return subscribe(ops, &_msg, afp);
  }

  //Assumes only one subscription for a given topic
  /** @brief Unsubscribe from a topic.
   *
   * This method unsubscribes from a topic to which the node has previously
   * subscribed.
   *
   * After this call completes, it is guaranteed that no further
   * callbacks will be invoked for this topic.
   *
   * This method can be safely called from within a message callback.
   *
   * @param _topic The topic from which to unsubscribe.
   *
   * @return true on success, false otherwise (unsubscribe failed)
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool unsubscribe(const std::string &_topic);
  /** @brief Unsubscribe from a topic, using a message object.
   *
   * This method unsubscribes from a topic to which the node has previously
   * subscribed.
   *
   * After this call completes, it is guaranteed that no further
   * callbacks will be invoked for this topic.
   *
   * This method can be safely called from within a message callback.
   *
   * @param _msg The message object that was passed into the subscribe()
   * call.
   *
   * @return true on success, false otherwise (unsubscribe failed)
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool unsubscribe(const Message& _msg);

  /** @brief Unsubscribe from a topic, using a topic, member function pointer,
   *         object (for the function pointer), and userdata
   *
   * This method unsubscribes from a topic to which the node has previously
   * subscribed using a function pointer and userdata. This form is required
   * if you have subscribed multiple times to a topic and want to unsubscribe
   * just one of them, rather than unsubscribing everything at once (which
   * is what happens if you just use the unsubscribe flavor that has a topic
   * name)
   *
   * After this call completes, it is guaranteed that the supplied callback
   * will not be fired again.
   *
   * This method can be safely called from within a message callback.
   *
   * @param _topic the topic from which to unsubscribe
   * @param fp the member function pointer
   * @param obj the object used by the member function pointer
   * @param userdata the (opaque) userdata, which will be tested by the
   *        native (void *) equality operator. In other words, deep comparison
   *        will NOT be attempted.
   *
   * @return true on success, false otherwise (unsubscribe failed)
   *
   * \deprecated Use the NodeHandle API instead
   */

  template<class T>
  ROSCPP_DEPRECATED bool unsubscribe(const std::string &_topic, void(T::*fp)(void *), T *obj,
      void *user_data)
  {
    AbstractFunctor* afp = new MethodFunctor<T> (obj, fp, user_data);
    bool ret = unsubscribe(_topic, afp);
    delete afp;

    return ret;
  }

  /** @brief Unsubscribe from a topic, using a topic, member function pointer,
   *         object (for the function pointer), and userdata
   *
   * This method unsubscribes from a topic to which the node has previously
   * subscribed using a function pointer and userdata. This form is required
   * if you have subscribed multiple times to a topic and want to unsubscribe
   * just one of them, rather than unsubscribing everything at once (which
   * is what happens if you just use the unsubscribe flavor that has a topic
   * name)
   *
   * After this call completes, it is guaranteed that the supplied callback
   * will not be fired again.
   *
   * This method can be safely called from within a message callback.
   *
   * @param _topic the topic from which to unsubscribe
   * @param fp the function pointer
   * @param userdata the (opaque) userdata, which will be tested by the
   *        native (void *) equality operator. In other words, deep comparison
   *        will NOT be attempted.
   *
   * @return true on success, false otherwise (unsubscribe failed)
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool unsubscribe(const std::string &_topic, void(*fp)(void *), void *user_data)
  {
    AbstractFunctor* afp = new FunctionFunctor(fp, user_data);
    bool ret = unsubscribe(_topic, afp);
    delete afp;

    return ret;
  }

  /** @brief Unsubscribe from a topic, using a topic, member function pointer,
   *         and object (for the function pointer)
   *
   * This method unsubscribes from a topic to which the node has previously
   * subscribed using a function pointer and userdata. This form is required
   * if you have subscribed multiple times to a topic and want to unsubscribe
   * just one of them, rather than unsubscribing everything at once (which
   * is what happens if you just use the unsubscribe flavor that has a topic
   * name)
   *
   * After this call completes, it is guaranteed that the supplied callback
   * will not be fired again.
   *
   * This method can be safely called from within a message callback.
   *
   * @param _topic the topic from which to unsubscribe
   * @param fp the member function pointer
   * @param obj the object used by the member function pointer
   *
   * @return true on success, false otherwise (unsubscribe failed)
   *
   * \deprecated Use the NodeHandle API instead
   */

  template<class T>
  ROSCPP_DEPRECATED bool unsubscribe(const std::string &_topic, void(T::*fp)(), T *obj)
  {
    AbstractFunctor* f = new MethodFunctor<T> (obj, fp);
    bool ret = unsubscribe(_topic, f);
    delete f;
    return ret;
  }

  /** @brief Unsubscribe from a topic, using a topic, member function pointer,
   *         and object (for the function pointer)
   *
   * This method unsubscribes from a topic to which the node has previously
   * subscribed using a function pointer and userdata. This form is required
   * if you have subscribed multiple times to a topic and want to unsubscribe
   * just one of them, rather than unsubscribing everything at once (which
   * is what happens if you just use the unsubscribe flavor that has a topic
   * name)
   *
   * After this call completes, it is guaranteed that the supplied callback
   * will not be fired again.
   *
   * This method can be safely called from within a message callback.
   *
   * @param _topic the topic from which to unsubscribe
   * @param fp the member function pointer
   * @param obj the object used by the member function pointer
   *
   * @return true on success, false otherwise (unsubscribe failed)
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool unsubscribe(const std::string &_topic, void(*fp)())
  {
    AbstractFunctor* f = new FunctionFunctor(fp);
    bool ret = unsubscribe(_topic, f);
    delete f;
    return ret;
  }

  /** @brief Unsubscribe from a topic
   *
   * This method unsubscribes from a topic to which the node has previously
   * subscribed using a function pointer and userdata. This form is required
   * if you have subscribed multiple times to a topic and want to unsubscribe
   * just one of them, rather than unsubscribing everything at once (which
   * is what happens if you just use the unsubscribe flavor that has a topic
   * name)
   *
   * After this call completes, it is guaranteed that the supplied callback
   * will not be fired again.
   *
   * This method can be safely called from within a message callback.
   *
   * @param _topic the topic from which to unsubscribe
   * @param helper The helper object used to determine which callback to remove
   *
   * @return true on success, false otherwise (unsubscribe failed)
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool unsubscribe(const std::string &_topic, const SubscriptionMessageHelperPtr& helper);

  /** @brief Get the list of topics that are being published by all nodes.
   *
   * This method communicates with the master to retrieve the list of all
   * currently advertised topics.
   *
   * @param topics A place to store the resulting list.  Each item in the
   * list is a pair <string topic, string type>.  The type is represented
   * in the format "package_name/MessageName", and is also retrievable
   * through message.__getDataType() or MessageName::__s_getDataType().
   *
   * @return true on success, false otherwise (topics not filled in)
   *
   * \deprecated Use master::getTopics()
   */
  ROSCPP_DEPRECATED bool getPublishedTopics(VP_string* topics);

  /**
   * \brief Retreives the currently-known list of nodes from the master
   *
   * \deprecated Use master::getNodes()
   */
  ROSCPP_DEPRECATED bool getNodes(V_string& nodes);

  /** @brief Enter simple event loop
   *
   * This method enters an infinite loop, sleeping for a short time on
   * each iteration.  The loop exits when the value of \ref _ok is false.
   *
   * This method is useful when your node does all of its work in
   * subscription callbacks.
   *
   * \deprecated Use ros::spin() or NodeHandle::spin()
   */
  ROSCPP_DEPRECATED void spin();

  /** @brief Set an arbitrary XML/RPC value on the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param v The value to be inserted.
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED void setParam(const std::string &key, const XmlRpc::XmlRpcValue &v);
  /** @brief Set a string value on the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param s The value to be inserted.
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED void setParam(const std::string &key, const std::string &s);
  /** @brief Set a string value on the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param s The value to be inserted.
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED void setParam(const std::string &key, const char* s);
  /** @brief Set a double value on the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param d The value to be inserted.
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED void setParam(const std::string &key, double d);
  /** @brief Set a integer value on the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param i The value to be inserted.
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED void setParam(const std::string &key, int i);
  /** @brief Set a integer value on the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param b The value to be inserted.
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED void setParam(const std::string& key, bool b);

  /** @brief Get a string value from the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param[out] s Storage for the retrieved value.
   * @param use_cache Determines whether or not we will cache and subscribe
   * to updates for this parameter.  If use_cache is true and we don't have
   * this parameter cached, we will subscribe to updates for this parameter
   * from the parameter server and cache the value for fast access.
   * If use_cache is false, we always hit the parameter server to request
   * the value.
   *
   * @return true if the parameter value was retrieved, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool getParam(const std::string &key, std::string &s, bool use_cache = false);
  /** @brief Get a double value from the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param[out] d Storage for the retrieved value.
   * @param use_cache Determines whether or not we will cache and subscribe
   * to updates for this parameter.  If use_cache is true and we don't have
   * this parameter cached, we will subscribe to updates for this parameter
   * from the parameter server and cache the value for fast access.
   * If use_cache is false, we always hit the parameter server to request
   * the value.
   *
   * @return true if the parameter value was retrieved, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool getParam(const std::string &key, double &d, bool use_cache = false);
  /** @brief Get a integer value from the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param[out] i Storage for the retrieved value.
   * @param use_cache Determines whether or not we will cache and subscribe
   * to updates for this parameter.  If use_cache is true and we don't have
   * this parameter cached, we will subscribe to updates for this parameter
   * from the parameter server and cache the value for fast access.
   * If use_cache is false, we always hit the parameter server to request
   * the value.
   *
   * @return true if the parameter value was retrieved, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool getParam(const std::string &key, int &i, bool use_cache = false);
  /** @brief Get a boolean value from the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param[out] b Storage for the retrieved value.
   * @param use_cache Determines whether or not we will cache and subscribe
   * to updates for this parameter.  If use_cache is true and we don't have
   * this parameter cached, we will subscribe to updates for this parameter
   * from the parameter server and cache the value for fast access.
   * If use_cache is false, we always hit the parameter server to request
   * the value.
   *
   * @return true if the parameter value was retrieved, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool getParam(const std::string &key, bool &b, bool use_cache = false);
  /** @brief Get an arbitrary XML/RPC value from the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param[out] v Storage for the retrieved value.
   * @param use_cache Determines whether or not we will cache and subscribe
   * to updates for this parameter.  If use_cache is true and we don't have
   * this parameter cached, we will subscribe to updates for this parameter
   * from the parameter server and cache the value for fast access.
   * If use_cache is false, we always hit the parameter server to request
   * the value.
   *
   * @return true if the parameter value was retrieved, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool getParam(const std::string &key, XmlRpc::XmlRpcValue &v, bool use_cache = false);

  /** @brief Check whether a parameter exists on the parameter server.
   *
   * @param key The key to check.
   *
   * @return true if the parameter exists, false otherwise
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool hasParam(const std::string &key);
  /** @brief Delete a parameter from the parameter server.
   *
   * @param key The key to delete.
   *
   * @return true if the deletion succeeded, false otherwise.
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED bool deleteParam(const std::string &key);

  /** @brief Assign value from parameter server, with default.
   *
   * This method tries to retrieve the indicated parameter value from the
   * parameter server, storing the result in param_val.  If the value
   * cannot be retrieved from the server, default_val is used instead.
   *
   * @param param_name The key to be searched on the parameter server.
   * @param[out] param_val Storage for the retrieved value.
   * @param default_val Value to use if the server doesn't contain this
   * parameter.
   *
   * \deprecated Use the NodeHandle API instead
   */
  template<class T>
  ROSCPP_DEPRECATED void param(const std::string& param_name, T &param_val, const T &default_val)
  {
    if (hasParam(param_name))
      if (getParam(param_name, param_val))
        return;
    param_val = default_val;
  }

  /** @brief Split a URI into host and port
   *
   * @param uri The URI to split
   * @param[out] host Storage for the host component of the URI.
   * @param[out] port Storage for the port component of the URI.
   *
   * @return true on success, false otherwise
   *
   * @todo Consider making this private.
   *
   * \deprecated
   */
  ROSCPP_DEPRECATED static bool splitURI(const std::string &uri, std::string &host, int &port);

  /** @brief Resolves a name into a fully-qualified name
   *
   * Resolves a name into a fully qualified name, eg. "blah" => "/namespace/blah". By default
   * also applies any matching name-remapping rules (which were usually supplied
   * on the command line at startup) to the given name, returning the
   * resulting remapped name.
   *
   * @param name Name to remap
   * @param remap Whether to apply name-remapping rules
   *
   * @return Resolved name.
   *
   * \deprecated Use the NodeHandle API instead
   */
  std::string resolveName(const std::string& name, bool remap = true);

  /**
   * \deprecated in favor of resolveName()
   */
  ROSCPP_DEPRECATED std::string mapName(const std::string& name) { return resolveName(name); }

  /**
   * @brief Cleans a name, removing excess /s
   *
   * @param name Name to clean
   * @return Cleaned name
   *
   * \deprecated
   */
  ROSCPP_DEPRECATED std::string cleanName(const std::string& name);

  /**
   * @brief Get the args we parsed out of argv in ros::init()
   *
   * \deprecated Use the NodeHandle API instead
   */
  ROSCPP_DEPRECATED static const V_string& getParsedArgs()
  {
    return s_args_;
  }

  /** \deprecated in favor of ros::shutdown()
   */
  ROSCPP_DEPRECATED void shutdown();
  /**
   * @brief Request that the node shut itself down from within the ROS thread
   *
   * This method signals the ROS thread to call shutdown().
   *
   * \deprecated in favor of ros::requestShutdown()
   */
  ROSCPP_DEPRECATED void requestShutdown();

  ROSCPP_DEPRECATED std::string getIP();

  /** @brief Get the IP address for unreliable connections
   *
   * @return Returns IP address where node listens for unreliable messages
   *
   * \deprecated
   */
  ROSCPP_DEPRECATED std::string getUnreliableServerIp() const;

  /** @brief Check whether the master is up
   *
   * This method tries to contact the master.  You can call it any time
   * after the node constructor has run, including within the constructor
   * of a class that inherits from node.  The intended usage is to check
   * whether the master is up before trying to make other requests
   * (subscriptions, advertisements, etc.).
   *
   * @returns true if the master is available, false otherwise.
   *
   * \deprecated use ros::master::check()
   */
  ROSCPP_DEPRECATED bool checkMaster();

  ROSCPP_DEPRECATED const std::string& getLogFilePath();

private:
  /** ros::Node is not copyable */
  Node(const Node &);
  /** ros::Node is not assignable either */
  Node &operator =(const Node&);

  /** @brief Args we care about
   */
  static V_string s_args_;
  static VP_string s_remappings_;
  static std::string s_name_;
  static uint32_t s_flags_;
  static bool s_initialized_;

  static bool s_created_by_handle_;
  static uint32_t s_refcount_;
  static boost::mutex s_refcount_mutex_;

  bool shutting_down_;
  int32_t master_retry_timeout_;

  void init(uint32_t options, int32_t master_retry_timeout);

  void internalCallbackQueueThreadFunc();

  bool isServiceAdvertised(const std::string& serv_name);
  bool unregisterService(const std::string& service);
  bool advertiseService(const AdvertiseServiceOptions& ops, int32_t thread_pool_size);

  bool subscribe(const SubscribeOptions& ops, Message* m, AbstractFunctor *cb);
  bool unsubscribe(const std::string &topic, AbstractFunctor *afp);
  bool advertise(const AdvertiseOptions& ops, bool allow_multiple = false);


  friend void init(int& argc, char** argv);
  friend void init(const VP_string& remapping_args);

  friend void init(int& argc, char** argv, const std::string& name, uint32_t options);
  friend void init(const VP_string& remappings, const std::string& name, uint32_t options);
  friend void init(const M_string& remappings, const std::string& name, uint32_t options);

  friend class NodeHandle;
};

void basicSigintHandler(int sig);

}

#endif

