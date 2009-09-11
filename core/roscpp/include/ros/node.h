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
#include "ros/time.h"
#include "roslib/Time.h"
#include "ros/message.h"
#include "ros/service.h"
#include "ros/poll_set.h"
#include "XmlRpc.h"

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

namespace ros_rpc
{
class Shutdown;
class GetPid;
class PublisherUpdate;
class RequestTopic;
class GetBusStats;
class GetBusInfo;
class ParamUpdate;
}

namespace ros
{

/** @brief ROS initialization function.
 *
 * You must call one of the init functions exactly once, prior to any other ROS calls.
 * Pass in argc and argv as they were provided to your main() function.
 * This function will parse any ROS arguments (e.g., topic name
 * remappings), and will consume them (i.e., argc and argv may be modified
 * as a result of this call).
 */
void init(int &argc, char **argv);

typedef std::vector<std::pair<std::string, std::string> > VP_string;
/**
 * \brief alternate ROS initialization function.
 *
 * This version of init takes a vector of string pairs, where each one constitutes
 * a name remapping, or one of the special remappings like __name, __master, __ns, etc.
 */
void init(const VP_string& remapping_args);

class AbstractServiceFunctor
{
public:
  AbstractServiceFunctor()
  {
  }
  virtual bool call(Message*, Message*) = 0;
  virtual ~AbstractServiceFunctor()
  {
  }

  virtual Message* createRequest() = 0;
  virtual Message* createResponse() = 0;
};

template<class MReq, class MRes>
class ServiceFunctor: public AbstractServiceFunctor
{
public:
  typedef boost::function<bool(MReq&, MRes&)> Callback;

  ServiceFunctor(const Callback& callback) :
    AbstractServiceFunctor(), callback_(callback)
  {
  }
  virtual ~ServiceFunctor()
  {
  }
  virtual bool call(Message* req, Message* res)
  {
    return callback_((MReq &) *req, (MRes &) *res);
  } // hack

  Message* createRequest()
  {
    return new MReq;
  }
  Message* createResponse()
  {
    return new MRes;
  }
private:
  Callback callback_;
};

class Header;
class Transport;
typedef boost::shared_ptr<Transport> TransportPtr;
class TransportTCP;
typedef boost::shared_ptr<TransportTCP> TransportTCPPtr;
class Connection;
typedef boost::shared_ptr<Connection> ConnectionPtr;
typedef std::set<ConnectionPtr> S_Connection;
typedef std::vector<ConnectionPtr> V_Connection;
class Publication;
typedef boost::shared_ptr<Publication> PublicationPtr;
typedef std::vector<PublicationPtr> V_Publication;
class Publisher;
typedef boost::shared_ptr<Publisher> PublisherPtr;
class Subscription;
typedef boost::shared_ptr<Subscription> SubscriptionPtr;
typedef std::list<SubscriptionPtr> L_Subscription;
typedef std::vector<SubscriptionPtr> V_Subscription;
class ServiceServer;
typedef boost::shared_ptr<ServiceServer> ServiceServerPtr;
typedef std::list<ServiceServerPtr> L_ServiceServer;
typedef std::vector<ServiceServerPtr> V_ServiceServer;
class ServiceClient;
typedef boost::shared_ptr<ServiceClient> ServiceClientPtr;
typedef std::list<ServiceClientPtr> L_ServiceClient;
class Transport;
typedef boost::shared_ptr<Transport> TransportPtr;

typedef boost::function<void(const PublisherPtr&)>
    SubscriptionConnectionCallback;

class MutexedXmlRpcClient
{
public:
  bool in_use;
  double last_use_time; // for reaping
  static const double zombie_time; // how long before it is toasted
  XmlRpc::XmlRpcClient *c;
  MutexedXmlRpcClient(XmlRpc::XmlRpcClient *_c) :
    in_use(false), last_use_time(0), c(_c)
  {
  }
};

/** @brief The ROS node class.
 *
 * This class is used in writing nodes.  Do not inherit from this class.
 *
 * The most widely used methods are:
 *   - Setup:
 *    - ros::init()
 *   - Publish / subscribe messaging:
 *    - advertise()
 *    - unadvertise()
 *    - publish()
 *    - subscribe()
 *    - unsubscribe()
 *   - RPC services:
 *    - advertiseService()
 *    - unadvertiseService()
 *    - ros::service::call()
 *   - Parameters:
 *    - getParam()
 *    - setParam()
 */
class Node
{
public:
  /** @brief Flag for node() constructor
   *
   * Use this option in the ros::Node() constructor to tell ROS not to install a
   * SIGINT handler.  You should install your own SIGINT handler in this
   * case, to ensure that the node is deleted application exits.
   */
  static const uint32_t DONT_HANDLE_SIGINT = 1 << 0;
  /** @brief Flag for node() constructor
   *
   * Use this option in the ros::Node() constructor to tell ROS to add a
   * random number to the end of your node's name, to make it unique.
   */
  static const uint32_t ANONYMOUS_NAME = 1 << 1;
  /** @brief Flag for node() constructor
   *
   * EXPERIMENTAL; do not use.
   *
   * Use this option in the ros::Node() constructor to tell ROS not
   * to start a thread for comms handling.  If you do this, you must
   * call tcprosServerUpdate() often.
   */
  static const uint32_t DONT_START_SERVER_THREAD = 1 << 2;

  /**
   * \brief Flag for node() constructor
   *
   * Use this option in the ros::Node() constructor to tell ROS to not
   * add the rosout appender
   */
  static const uint32_t DONT_ADD_ROSOUT_APPENDER = 1 << 3;

  /** @brief Node constructor
   *
   * @param name The node's name.
   * @param options A set of flags, ORed together.  The valid flags
   *                  are ros::Node::DONT_HANDLE_SIGINT, ros::Node::ANONYMOUS_NAME
   * @param master_retry_timeout The maximum time this node should spend looping
   *                              trying to connect to the master, in milliseconds
   */
  Node(std::string name, uint32_t options = 0, int32_t master_retry_timeout = -1);
  /** @brief Node destructor */
  virtual ~Node();
  /** @brief Get pointer to global node object.
   *
   * @return Pointer to global node object.
   */
  static Node *instance();
  /** @brief Check whether it's time to exit.
   *
   * This method checks the value of \ref _ok, to see whether it's yet time
   * to exit.
   *
   * @return true if we're still OK, false if it's time to exit
   */
  inline bool ok()
  {
    return ok_;
  }
  /** @brief Check whether we're in the middle of shutting everything down
   *
   * @return true if we're shutting down, false otherwise
   */
  inline bool shuttingDown()
  {
    return shutting_down_;
  }
  /** @brief Get the node's name.
   *
   * @return The node's name
   */
  inline const std::string &getName()
  {
    return name_;
  }

  /**
   * @brief Set the max time this node should spend looping trying to connect to the master
   * @param milliseconds the timeout, in milliseconds.  A value of -1 means infinite
   */
  void setMasterRetryTimeout(int32_t milliseconds);

  /** @brief Get the list of advertised topics
   *
   * @return The advertised topics
   */
  void getAdvertisedTopics(std::vector<std::string>& topics);

  PublicationPtr getTopic(std::string topic);

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
   */
  template<class T>
  bool advertise(const std::string &topic, size_t max_queue)
  {
    return advertise(topic, topic, T::__s_getDataType(), T::__s_getMD5Sum(),
        NULL, NULL, max_queue);
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
   */
  bool advertise(const std::string &topic, const Message& msgref, size_t max_queue);

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
   */
  template<class T>
  bool advertise(const std::string &_topic, const SubscriptionConnectionCallback& sub_connect,
                 const SubscriptionConnectionCallback& sub_disconnect, size_t max_queue)
  {
    return advertise(_topic, _topic, T::__s_getDataType(),
        T::__s_getMD5Sum(), sub_connect, sub_disconnect, max_queue);
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
   */
  template<class T>
  bool advertise(const std::string &_topic, const SubscriptionConnectionCallback& sub_connect,
                 size_t max_queue)
  {
    return advertise(_topic, _topic, T::__s_getDataType(),
        T::__s_getMD5Sum(), sub_connect, SubscriptionConnectionCallback(), max_queue);
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
   */
  template<class T>
  bool advertise(const std::string &_topic, const Message& msgref,
      void(T::*sub_connect)(const PublisherPtr&), void(T::*sub_disconnect)(
          const PublisherPtr&), size_t max_queue)
  {
    return advertise(_topic, _topic, msgref.__getDataType(),
        msgref.__getMD5Sum(), boost::bind(sub_connect, static_cast<T*>(this), _1),
        boost::bind(sub_disconnect, static_cast<T*>(this), _1), max_queue);
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
   */
  template<class T>
  bool advertise(const std::string &_topic, const Message& msgref,
      void(T::*sub_connect)(const PublisherPtr&), size_t max_queue)
  {
    return advertise(_topic, _topic, msgref.__getDataType(),
        msgref.__getMD5Sum(), boost::bind(sub_connect, static_cast<T*>(this), _1),
        SubscriptionConnectionCallback(), max_queue);
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
  bool unadvertise(const std::string &topic);

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
   */
  template<class T, class MReq, class MRes>
  bool advertiseService(const std::string &topic, bool(T::*srv_func)(MReq &,
      MRes &), int thread_pool_size = 1)
  {
    std::string mapped_name = mapName(topic);
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
    return advertiseService(mapped_name, MReq::__s_getServerMD5Sum(),
        MReq::__s_getDataType(), MRes::__s_getDataType(), thread_pool_size,
        new ServiceFunctor<MReq, MRes> (
            boost::bind(srv_func, static_cast<T*>(this), _1, _2)));
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
   */
  template<class T, class MReq, class MRes>
  bool advertiseService(const std::string &topic, bool(T::*srv_func)(MReq &,
      MRes &), T *obj, int thread_pool_size = 1)
  {
    std::string mapped_name = mapName(topic);

    if (MReq::__s_getServerMD5Sum() != MRes::__s_getServerMD5Sum())
    {
      ROS_FATAL("woah! the request and response parameters to the server "
        "callback function must be autogenerated from the same "
        "server definition file (.srv). your advertise_servce "
        "call for %s appeared to use request/response types "
        "from different .srv files.", topic.c_str());
      ROS_BREAK();
    }
    return advertiseService(mapped_name, MReq::__s_getServerMD5Sum(),
        MReq::__s_getDataType(), MRes::__s_getDataType(), thread_pool_size,
        new ServiceFunctor<MReq, MRes> (boost::bind(srv_func, obj, _1, _2)));
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
   */
  template<class MReq, class MRes>
  bool advertiseService(const std::string &topic, bool (*srv_func)(MReq&, MRes&), int thread_pool_size = 1)
  {
    std::string mapped_name = mapName(topic);

    if (MReq::__s_getServerMD5Sum() != MRes::__s_getServerMD5Sum())
    {
      ROS_FATAL("woah! the request and response parameters to the server "
        "callback function must be autogenerated from the same "
        "server definition file (.srv). your advertise_servce "
        "call for %s appeared to use request/response types "
        "from different .srv files.", topic.c_str());
      ROS_BREAK();
    }
    return advertiseService(mapped_name, MReq::__s_getServerMD5Sum(),
        MReq::__s_getDataType(), MRes::__s_getDataType(), thread_pool_size,
        new ServiceFunctor<MReq, MRes> (srv_func));
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
   */
  template<class T, class MReq, class MRes>
  bool advertiseService(const std::string &topic, bool(T::*srv_func)(MReq &,
      MRes &), const MReq& req, const MRes& res, int thread_pool_size = 1)
  {
    std::string mapped_name = mapName(topic);
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
    return advertiseService(mapped_name, req.__getServerMD5Sum(),
        req.__getDataType(), res.__getDataType(), thread_pool_size,
        new ServiceFunctor<MReq, MRes> (
            boost::bind(srv_func, static_cast<T*>(this), _1, _2)));
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
   */
  template<class T, class MReq, class MRes>
  bool advertiseService(const std::string &topic, bool(T::*srv_func)(MReq &,
      MRes &), T *obj, const MReq& req, const MRes& res, int thread_pool_size =
      1)
  {
    std::string mapped_name = mapName(topic);

    if (req.__getServerMD5Sum() != res.__getServerMD5Sum())
    {
      ROS_FATAL("woah! the request and response parameters to the server "
        "callback function must be autogenerated from the same "
        "server definition file (.srv). your advertise_servce "
        "call for %s appeared to use request/response types "
        "from different .srv files.", topic.c_str());
      ROS_BREAK();
    }
    return advertiseService(mapped_name, req.__getServerMD5Sum(),
        req.__getDataType(), res.__getDataType(), thread_pool_size,
        new ServiceFunctor<MReq, MRes> (boost::bind(srv_func, obj, _1, _2)));
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
   */
  bool unadvertiseService(const std::string &serv_name);

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
   */
  void publish(const std::string &_topic, const Message& m);

  void publish(const PublicationPtr& p, const Message& m);

  /** @brief Return the number of subscriptions the node has:
   *
   * @return number of subscriptions
   */
  size_t numSubscriptions()
  {
    return subscriptions_.size();
  }

  /** @brief Return the number of subscribers a node has for a particular topic:
   *
   * @param _topic The topic name to check
   *
   * @return number of subscribers
   */
  size_t numSubscribers(const std::string &_topic);

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
   */
  template<class M, class T>
  bool subscribe(const std::string &_topic, M &_msg, void(T::*fp)(), T* obj,
      int max_queue)
  {
    AbstractFunctor *afp = new MethodFunctor<T> (obj, fp);
    return subscribe(_topic, &_msg, afp, true, max_queue, _msg.__getDataType());
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
   */
  template<class M, class T>
  bool subscribe(const std::string &_topic, M &_msg, void(T::*fp)(),
      int max_queue)
  {
    AbstractFunctor *afp = new MethodFunctor<T> (static_cast<T*> (this), fp);
    return subscribe(_topic, &_msg, afp, true, max_queue, _msg.__getDataType());
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
   */
  template<class M>
  bool subscribe(const std::string &_topic, M &_msg, void(*fp)(),
      int max_queue)
  {
    AbstractFunctor *afp = new FunctionFunctor(fp);
    return subscribe(_topic, &_msg, afp, true, max_queue, _msg.__getDataType());
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
   */
  template<class M>
  bool subscribe(const std::string &_topic, M &_msg, void (*fp)(void *),
      void *user_data, int max_queue)
  {
    AbstractFunctor *afp = new FunctionFunctor(fp, user_data);
    return subscribe(_topic, &_msg, afp, true, max_queue, _msg.__getDataType());
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
   */
  template<class M, class T>
  bool subscribe(const std::string &_topic, M &_msg, void(T::*fp)(void *),
      void *user_data, int max_queue)
  {
    AbstractFunctor *afp = new MethodFunctor<T> (static_cast<T*> (this), fp,
        user_data);
    return subscribe(_topic, &_msg, afp, true, max_queue, _msg.__getDataType());
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
   */
  template<class M, class T>
  bool subscribe(const std::string &_topic, M &_msg, void(T::*fp)(void *),
      T *obj, void *user_data, int max_queue)
  {
    AbstractFunctor *afp = new MethodFunctor<T> (obj, fp, user_data);
    return subscribe(_topic, &_msg, afp, true, max_queue, _msg.__getDataType());
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
   */
  bool unsubscribe(const std::string &_topic);
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
   */
  bool unsubscribe(const Message& _msg);

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
   */

  template<class T>
  bool unsubscribe(const std::string &_topic, void(T::*fp)(void *), T *obj,
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
   */
  bool unsubscribe(const std::string &_topic, void(*fp)(void *), void *user_data)
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
   */

  template<class T>
  bool unsubscribe(const std::string &_topic, void(T::*fp)(), T *obj)
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
   */
  bool unsubscribe(const std::string &_topic, void(*fp)())
  {
    AbstractFunctor* f = new FunctionFunctor(fp);
    bool ret = unsubscribe(_topic, f);
    delete f;
    return ret;
  }

  /** @brief Get the list of topics that are being published by all nodes.
   *
   * This method communicates with the master to retrieve the list of all
   * currently advertised topics.
   *
   * @param topics A place to store the resulting list.  Each item in the
   * list is a pair <topic, type> (how is the type represented?)
   *
   * @return true on success, false otherwise (topics not filled in)
   *
   * @todo Explain how the retrieved list is represented.
   */
  typedef std::pair<std::string, std::string> StringPair;
  typedef std::vector<std::pair<std::string, std::string> > V_StringPair;
  bool getPublishedTopics(V_StringPair* topics);

  /** @brief Update local publisher lists.
   *
   * Use this method to update address information for publishers on a
   * given topic.
   *
   * @param topic The topic of interest
   * @param pubs The list of publishers to update.
   *
   * @return true on success, false otherwise.
   */
  bool pubUpdate(const std::string &topic, const std::vector<std::string> &pubs);

  /** @brief Enter simple event loop
   *
   * This method enters an infinite loop, sleeping for a short time on
   * each iteration.  The loop exits when the value of \ref _ok is false.
   *
   * This method is useful when your node does all of its work in
   * subscription callbacks.
   */
  void spin();

  /** @brief Validate an XML/RPC response
   *
   * @param method The RPC method that was invoked.
   * @param response The resonse that was received.
   * @param payload The payload that was received.
   *
   * @return true if validation succeeds, false otherwise.
   *
   * @todo Consider making this private.
   */
  bool validateXmlrpcResponse(std::string method,
      XmlRpc::XmlRpcValue &response, XmlRpc::XmlRpcValue &payload);
  /** @brief Request a topic
   *
   * Negotiate a subscriber connection on a topic.
   *
   * @param topic The topic of interest.
   * @param protos List of transport protocols, in preference order
   * @param ret Return value
   *
   * @return true on success, false otherwise
   *
   * @todo Consider making this private
   */
  bool requestTopic(const std::string &topic, XmlRpc::XmlRpcValue &protos,
      XmlRpc::XmlRpcValue &ret);

  /** @brief Set an arbitrary XML/RPC value on the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param v The value to be inserted.
   */
  void setParam(const std::string &key, const XmlRpc::XmlRpcValue &v);
  /** @brief Set a string value on the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param s The value to be inserted.
   */
  void setParam(const std::string &key, const std::string &s);
  /** @brief Set a string value on the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param s The value to be inserted.
   */
  void setParam(const std::string &key, const char* s);
  /** @brief Set a double value on the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param d The value to be inserted.
   */
  void setParam(const std::string &key, double d);
  /** @brief Set a integer value on the parameter server.
   *
   * @param key The key to be used in the parameter server's dictionary
   * @param i The value to be inserted.
   */
  void setParam(const std::string &key, int i);

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
   */
  bool getParam(const std::string &key, std::string &s, bool use_cache = false);
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
   */
  bool getParam(const std::string &key, double &d, bool use_cache = false);
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
   */
  bool getParam(const std::string &key, int &i, bool use_cache = false);
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
   */
  bool getParam(const std::string &key, bool &b, bool use_cache = false);
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
   */
  bool getParam(const std::string &key, XmlRpc::XmlRpcValue &v, bool use_cache = false);

  /** @brief Check whether a parameter exists on the parameter server.
   *
   * @param key The key to check.
   *
   * @return true if the parameter exists, false otherwise
   */
  bool hasParam(const std::string &key);
  /** @brief Delete a parameter from the parameter server.
   *
   * @param key The key to delete.
   *
   * @return true if the deletion succeeded, false otherwise.
   */
  bool deleteParam(const std::string &key);

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
   */
  template<class T>
  void param(const std::string& param_name, T &param_val, const T &default_val)
  {
    if (hasParam(param_name))
      if (getParam(param_name, param_val))
        return;
    param_val = default_val;
  }

  void paramUpdate(const std::string& param_name, const XmlRpc::XmlRpcValue& v);

  /** @brief Split a URI into host and port
   *
   * @param uri The URI to split
   * @param[out] host Storage for the host component of the URI.
   * @param[out] port Storage for the port component of the URI.
   *
   * @return true on success, false otherwise
   *
   * @todo Consider making this private.
   */
  static bool splitURI(const std::string &uri, std::string &host, int &port);

  /** @brief Apply name-remapping rules to a name.
   *
   * Applies any matching name-remapping rules (which were usually supplied
   * on the command line at startup) to the given name, returning the
   * resulting remapped name.
   *
   * @param name Name to remap
   *
   * @return Remapped name.
   */
  std::string mapName(const std::string& name);
  /**
   * @brief Cleans a name, removing excess /s
   *
   * @param name Name to clean
   * @return Cleaned name
   */
  std::string cleanName(const std::string& name);

  typedef std::vector<std::string> V_string;
  /**
   * @brief Get the args we parsed out of argv in ros::init()
   */
  static const V_string& getParsedArgs()
  {
    return s_args_;
  }

  /** @brief Shutdown a node
   *
   * This method shuts down a node by disconnecting it cleanly from the
   * master.  This method should always be called before exiting.  In most
   * cases, the user does not need to call it explicitly, because it will
   * be called by the Node destructor
   */
  void shutdown();
  /**
   * @brief Request that the node shut itself down from within the ROS thread
   *
   * This method signals the ROS thread to call shutdown().
   */
  void requestShutdown();
  /** @brief Print a shutdown message
   *
   * @todo Consider making this private
   */
  void setShutdownMessage(const std::string& msg);

  /** @brief Get the hostname where the master runs.
   *
   * @return The master's hostname, as a string
   */
  inline const std::string &getMasterHost()
  {
    return master_host_;
  }
  /** @brief Get the port where the master runs.
   *
   * @return The master's port.
   */
  inline int getMasterPort() const
  {
    return master_port_;
  }
  /**
   * @brief Get the xmlrpc URI of this node
   */
  inline const std::string& getXMLRPCURI()
  {
    return xmlrpc_uri_;
  }

  /** @brief Get an XML/RPC client object that is connected to a given host
   * and port.
   *
   * @param host Host to connect to.
   * @param port Port to connect to.
   * @param uri URI to connect to.
   *
   * @return An (possibly newly allocated) XML/RPC client that is connected
   * as requested.
   *
   * @todo Consider making this private
   */
  XmlRpc::XmlRpcClient *getXMLRPCClient(const std::string &host, int port,
      const std::string &uri);
  /** @brief Release an XML/RPC client object.
   *
   * Marks the client as unused, and thereby ready for reuse by someone
   * else.
   *
   * @param c The XML/RPC client object to be released.
   */
  void releaseXMLRPCClient(XmlRpc::XmlRpcClient *c);

  /** @brief Compute the statistics for the node's connectivity
   *
   * This is the implementation of the xml-rpc getBusStats function;
   * it populates the XmlRpcValue object sent to it with various statistics
   * about the node's connectivity, bandwidth utilization, etc.
   */
  void getBusStats(XmlRpc::XmlRpcValue &stats);

  /** @brief Compute the info for the node's connectivity
   *
   * This is the implementation of the xml-rpc getBusInfo function;
   * it populates the XmlRpcValue object sent to it with various info
   * about the node's connectivity.
   */
  void getBusInfo(XmlRpc::XmlRpcValue &info);

  /** @brief Lookup an advertised topic.
   *
   * This method iterates over advertised_topics, looking for one with name
   * matching the given topic name.  The advertised_topics_mutex is locked
   * during this search.  This method is only used internally.
   *
   * @param topic The topic name to look for.
   *
   * @returns Pointer to the matching Publication, NULL if none is found.
   */
  PublicationPtr lookupTopic(const std::string& topic);

  /** @brief Lookup an advertised service.
   *
   * This method iterates over advertised_services, looking for one with name
   * matching the given topic name.  The advertised_services_mutex is locked
   * during this search.  This method is only used internally.
   *
   * @param service The service name to look for.
   *
   * @returns Pointer to the matching ServiceServer, NULL if none is found.
   */
  ServiceServerPtr lookupServiceServer(const std::string& service);

  /** @brief Create a new client to the specified service.  If a client to that service already exists, returns the existing one.
   *
   * @param service The service to connect to
   * @param persistent Whether to keep this connection alive for more than one service call
   * @param request_md5sum The md5sum of the request message
   * @param response_md5sum The md5sum of the response message
   *
   * @returns Shared pointer to the ServiceClient, empty shared pointer if none is found.
   */
  typedef std::map<std::string, std::string> M_string;
  ServiceClientPtr createServiceClient(const std::string& service,
      bool persistent,
      const std::string& request_md5sum, const std::string& response_md5sum,
      const M_string& header_values);

  /** @brief Remove the specified service client from our list
   *
   * @param client The client to remove
   */
  void removeServiceClient(const ServiceClientPtr& client);

  /** @brief Add a connection to be tracked by the node.  Will automatically remove them if they've been dropped, but from inside the ros thread
   *
   * @param The connection to add
   */
  void addConnection(const ConnectionPtr& connection);

  /** @brief Lookup the host/port of a service.
   *
   * @param name The name of the service
   * @param serv_host OUT -- The host of the service
   * @param serv_port OUT -- The port of the service
   */
  bool lookupService(const std::string& name, std::string& serv_host,
      int& serv_port);

  /** @brief Run one iteration of the socket server loop
   *
   * DO NOT CALL THIS METHOD.
   *
   * @todo Rename and parameterize this method.
   */
  void tcprosServerUpdate();

  PollSet& getPollSet() { return poll_set_; }

  /** @brief Get a new connection ID
   *
   * DO NOT CALL THIS METHOD.
   *
   * @todo Hide this method (e.g., by friending Publication and Subscription)
   */
  unsigned int getNewConnectionID();

  /** @brief Check whether the master is up
   *
   * This method tries to contact the master.  You can call it any time
   * after the node constructor has run, including within the constructor
   * of a class that inherits from node.  The intended usage is to check
   * whether the master is up before trying to make other requests
   * (subscriptions, advertisements, etc.).
   *
   * @returns true if the master is available, false otherwise.
   */
  bool checkMaster();

  /** @brief Shutdown the master.
   *
   * DO NOT CALL THIS METHOD.
   *
   * This method shuts the master down.  It's used in testing.
   *
   * @returns true if the master is successfully shutdown, false otherwise.
   */
  bool _shutdownMaster();
  /** @brief Callback for time message.
   * The callback for the time message.
   */
  void timeCallback();
protected:
  std::string name_, master_host_, xmlrpc_uri_;
  int master_port_;
  std::string ip_;

private:
  /** ros::Node is not copyable */
  Node(const Node &);
  /** ros::Node is not assignable either */
  Node &operator =(const Node&);

  /** @brief Args we care about
   */
  static V_string s_args_;
  static VP_string s_remappings_;
  static bool s_initialized_;

  bool shutting_down_;
  bool needs_shutdown_;
  bool ok_;
  int32_t master_retry_timeout_;
  boost::mutex shutting_down_mutex_;
  boost::mutex log_file_mutex_, subs_mutex_;
  boost::mutex advertised_topics_mutex_;
  V_Publication advertised_topics_;
  std::list<std::string> advertised_topic_names_;
  boost::mutex advertised_topic_names_mutex_;

  L_Subscription subscriptions_;
  L_ServiceServer service_servers_;
  boost::mutex service_servers_mutex_;
  boost::thread tcpros_server_thread_;
  bool use_server_thread_;
  int xmlrpc_port_;

  L_ServiceClient service_clients_;
  boost::mutex service_clients_mutex_;

  TransportTCPPtr tcpserver_transport_;

  S_Connection connections_;
  V_Connection dropped_connections_;
  boost::mutex connections_mutex_;
  boost::mutex dropped_connections_mutex_;

  // The connection ID counter, used to assign unique ID to each inbound or
  // outbound connection.  Access via getNewConnectionID()
  unsigned int connection_id_counter_;
  boost::mutex connection_id_counter_mutex_;

  // XmlRpc server stuff
  ros_rpc::Shutdown* shutdown_xmlrpc_object_;
  ros_rpc::GetPid* getpid_xmlrpc_object_;
  ros_rpc::PublisherUpdate* publisher_update_xmlrpc_object_;
  ros_rpc::RequestTopic* request_topic_xmlrpc_object_;
  ros_rpc::GetBusStats* get_bus_stats_xmlrpc_object_;
  ros_rpc::GetBusInfo* get_bus_info_xmlrpc_object_;
  ros_rpc::ParamUpdate* param_update_xmlrpc_object_;

  typedef std::map<std::string, XmlRpc::XmlRpcValue> M_Param;
  M_Param params_;
  boost::mutex params_mutex_;
  typedef std::set<std::string> S_string;
  S_string subscribed_params_;

  M_string name_cache_;
  std::string namespace_; // namespace

  PollSet poll_set_;

#if defined(__APPLE__)
  // OSX has problems with lots of concurrent xmlrpc calls
  boost::mutex xmlrpc_call_mutex_;
#endif

  inline std::string joinPath(const std::string &p1, const std::string &p2)
  {
    std::string joined;
    // start w/leading slash if p1 is nonempty and p1 doesn't begin with a slash
    if (p1.length() && p1[0] != '/')
      joined = "/";
    // or start w/leading slash if p1 is empty and p2 doesn't begin with a slash
    else if (!p1.length() && (!p2.length() || p2[0] != '/'))
      joined = "/";
    // add p1
    joined += p1;
    // add another slash if 'joined' doesn't end with a slash and p2 is
    // nonempty and doesn't begin with a slash
    if (joined[joined.length() - 1] != '/' && p2.length() && p2[0] != '/')
      joined += "/";
    // now, add p2 and we're done.
    return joined + p2;
  }

  std::string determineIP();
  XmlRpc::XmlRpcServer xmlrpc_server_;
  std::vector<MutexedXmlRpcClient> xmlrpc_clients_;
  boost::mutex xmlrpc_clients_mutex_;

  roslib::Time time_msg_;

  bool unsubscribe(const std::string &topic, AbstractFunctor *afp);

  void tcprosServerInit();
  void tcprosServerFini();
  void tcprosServerThreadFunc();
  void tcprosAcceptConnection(const TransportTCPPtr& transport);

  // Must lock the advertised topics mutex before calling this function
  bool isTopicAdvertised(const std::string& topic);
  bool isServiceAdvertised(const std::string& serv_name);
  bool registerSubscriber(const SubscriptionPtr& s, const std::string& datatype);
  bool unregisterSubscriber(const std::string& topic);
  bool unregisterPublisher(const std::string& topic);
  bool unregisterService(const std::string& service);
  const static int MAX_TCPROS_CONN_QUEUE = 100; // magic

  PublicationPtr getPublication(const std::string &topic);

  bool advertise(const std::string &topic, const std::string &original_name,
      const std::string &datatype, const std::string &md5sum,
      const SubscriptionConnectionCallback& connect_cb,
      const SubscriptionConnectionCallback& disconnect_cb, size_t max_queue);
  bool advertiseService(const std::string &serv_name,
      const std::string& md5sum, const std::string& request_data_type,
      const std::string& response_data_type, int thread_pool_size,
      AbstractServiceFunctor *cb);

  bool subscribe(const std::string &name, Message* m, AbstractFunctor *cb,
      bool threaded, int max_queue, const std::string &datatype);

  /** if it finds a pre-existing subscription to the same topic and of the
   *  same message type, it appends the Functor to the callback vector for
   *  that subscription. otherwise, it returns false, indicating that a new
   *  subscription needs to be created.
   */
  bool addSubCallback(const std::string &_topic, Message* m, AbstractFunctor *fp,
      int max_queue);

  bool onConnectionHeaderReceived(const ConnectionPtr& conn, const Header& header);
  void onConnectionDropped(const ConnectionPtr& conn);
  // Remove any dropped connections from our list, causing them to be destroyed
  // They can't just be removed immediately when they're dropped because the ros
  // thread may still be using them (or more likely their transport)
  void removeDroppedConnections();

  void processSocketChanges();

  /** @brief Setup a basic handler for SIGINT
   */
  void setupSigintHandler();

  /** @brief Execute an XMLRPC call on the master
   *
   * @param method The RPC method to invoke
   * @param request The arguments to the RPC call
   * @param response [out] The resonse that was received.
   * @param payload [out] The payload that was received.
   * @param wait_for_master Whether or not this call should loop until it can contact the master
   *
   * @return true if call succeeds, false otherwise.
   *
   * @todo Consider making this private.
   */
  bool executeMasterXmlrpc(const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload, bool wait_for_master);

  friend void init(int& argc, char** argv);
  friend void init(const VP_string& remapping_args);
};

void basicSigintHandler(int sig);

}

#endif

