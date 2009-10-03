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
 *   * Neither the names of Willow Garage, Inc. nor the names of its
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

#ifndef ROSCPP_TOPIC_MANAGER_H
#define ROSCPP_TOPIC_MANAGER_H

#include "forwards.h"
#include "common.h"
#include "subscribe_options.h"
#include "advertise_options.h"
#include "rosout_appender.h"

#include "XmlRpcValue.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

namespace ros
{

class Message;

class TopicManager;
typedef boost::shared_ptr<TopicManager> TopicManagerPtr;

class PollManager;
typedef boost::shared_ptr<PollManager> PollManagerPtr;

class XMLRPCManager;
typedef boost::shared_ptr<XMLRPCManager> XMLRPCManagerPtr;

class ConnectionManager;
typedef boost::shared_ptr<ConnectionManager> ConnectionManagerPtr;

class TopicManager
{
public:
  static const TopicManagerPtr& instance();

  TopicManager();
  ~TopicManager();

  void start();
  void shutdown();

  bool subscribe(const SubscribeOptions& ops);
  bool unsubscribe(const std::string &_topic, const SubscriptionMessageHelperPtr& helper);

  bool advertise(const AdvertiseOptions& ops, const SubscriberCallbacksPtr& callbacks);
  bool unadvertise(const std::string &topic, const SubscriberCallbacksPtr& callbacks);

  /** @brief Get the list of topics advertised by this node
   *
   * @param[out] topics The advertised topics
   */
  void getAdvertisedTopics(V_string& topics);

  /** @brief Get the list of topics subscribed to by this node
   *
   * @param[out] The subscribed topics
   */
  void getSubscribedTopics(V_string& topics);

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
  PublicationPtr lookupPublication(const std::string& topic);

  /** @brief Return the number of subscribers a node has for a particular topic:
   *
   * @param _topic The topic name to check
   *
   * @return number of subscribers
   */
  size_t getNumSubscribers(const std::string &_topic);
  size_t getNumSubscriptions();

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

private:
  /** if it finds a pre-existing subscription to the same topic and of the
   *  same message type, it appends the Functor to the callback vector for
   *  that subscription. otherwise, it returns false, indicating that a new
   *  subscription needs to be created.
   */
  bool addSubCallback(const SubscribeOptions& ops);

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
  bool requestTopic(const std::string &topic, XmlRpc::XmlRpcValue &protos, XmlRpc::XmlRpcValue &ret);

  // Must lock the advertised topics mutex before calling this function
  bool isTopicAdvertised(const std::string& topic);

  bool registerSubscriber(const SubscriptionPtr& s, const std::string& datatype);
  bool unregisterSubscriber(const std::string& topic);
  bool unregisterPublisher(const std::string& topic);

  PublicationPtr lookupPublicationWithoutLock(const std::string &topic);

  void processPublishQueue();

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

  void pubUpdateCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void requestTopicCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void getBusStatsCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void getBusInfoCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

  bool isShuttingDown() { return shutting_down_; }

  boost::mutex subs_mutex_;
  L_Subscription subscriptions_;

  boost::recursive_mutex advertised_topics_mutex_;
  V_Publication advertised_topics_;
  std::list<std::string> advertised_topic_names_;
  boost::mutex advertised_topic_names_mutex_;

  typedef std::vector<std::pair<PublicationPtr, SerializedMessage> > V_PublicationAndSerializedMessagePair;
  V_PublicationAndSerializedMessagePair publish_queue_;
  boost::mutex publish_queue_mutex_;

  volatile bool shutting_down_;
  boost::mutex shutting_down_mutex_;

  PollManagerPtr poll_manager_;
  ConnectionManagerPtr connection_manager_;
  XMLRPCManagerPtr xmlrpc_manager_;
};

} // namespace ros

#endif // ROSCPP_TOPIC_MANAGER_H
