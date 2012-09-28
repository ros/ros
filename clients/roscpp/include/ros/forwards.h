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

#ifndef ROSCPP_FORWARDS_H
#define ROSCPP_FORWARDS_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <list>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/function.hpp>

#include <ros/time.h>
#include <ros/macros.h>
#include "exceptions.h"

namespace ros
{

typedef boost::shared_ptr<void> VoidPtr;
typedef boost::weak_ptr<void> VoidWPtr;
typedef boost::shared_ptr<void const> VoidConstPtr;
typedef boost::weak_ptr<void const> VoidConstWPtr;

class Header;
class Transport;
typedef boost::shared_ptr<Transport> TransportPtr;
class TransportTCP;
typedef boost::shared_ptr<TransportTCP> TransportTCPPtr;
class TransportUDP;
typedef boost::shared_ptr<TransportUDP> TransportUDPPtr;
class Connection;
typedef boost::shared_ptr<Connection> ConnectionPtr;
typedef std::set<ConnectionPtr> S_Connection;
typedef std::vector<ConnectionPtr> V_Connection;
class Publication;
typedef boost::shared_ptr<Publication> PublicationPtr;
typedef std::vector<PublicationPtr> V_Publication;
class SubscriberLink;
typedef boost::shared_ptr<SubscriberLink> SubscriberLinkPtr;
typedef std::vector<SubscriberLinkPtr> V_SubscriberLink;
class Subscription;
typedef boost::shared_ptr<Subscription> SubscriptionPtr;
typedef boost::weak_ptr<Subscription> SubscriptionWPtr;
typedef std::list<SubscriptionPtr> L_Subscription;
typedef std::vector<SubscriptionPtr> V_Subscription;
typedef std::set<SubscriptionPtr> S_Subscription;
class PublisherLink;
typedef boost::shared_ptr<PublisherLink> PublisherLinkPtr;
typedef std::vector<PublisherLinkPtr> V_PublisherLink;
class ServicePublication;
typedef boost::shared_ptr<ServicePublication> ServicePublicationPtr;
typedef std::list<ServicePublicationPtr> L_ServicePublication;
typedef std::vector<ServicePublicationPtr> V_ServicePublication;
class ServiceServerLink;
typedef boost::shared_ptr<ServiceServerLink> ServiceServerLinkPtr;
typedef std::list<ServiceServerLinkPtr> L_ServiceServerLink;
class Transport;
typedef boost::shared_ptr<Transport> TransportPtr;
class NodeHandle;
typedef boost::shared_ptr<NodeHandle> NodeHandlePtr;

typedef std::vector<std::pair<std::string, std::string> > VP_string;
typedef std::vector<std::string> V_string;
typedef std::set<std::string> S_string;
typedef std::map<std::string, std::string> M_string;
typedef std::pair<std::string, std::string> StringPair;

class SingleSubscriberPublisher;
typedef boost::function<void(const SingleSubscriberPublisher&)> SubscriberStatusCallback;

class CallbackQueue;
class CallbackQueueInterface;
class CallbackInterface;
typedef boost::shared_ptr<CallbackInterface> CallbackInterfacePtr;

struct SubscriberCallbacks
{
  SubscriberCallbacks(const SubscriberStatusCallback& connect = SubscriberStatusCallback(),
                      const SubscriberStatusCallback& disconnect = SubscriberStatusCallback(),
                      const VoidConstPtr& tracked_object = VoidConstPtr(),
                      CallbackQueueInterface* callback_queue = 0)
  : connect_(connect)
  , disconnect_(disconnect)
  , callback_queue_(callback_queue)
  {
    has_tracked_object_ = false;
    if (tracked_object)
    {
      has_tracked_object_ = true;
      tracked_object_ = tracked_object;
    }
  }
  SubscriberStatusCallback connect_;
  SubscriberStatusCallback disconnect_;

  bool has_tracked_object_;
  VoidConstWPtr tracked_object_;
  CallbackQueueInterface* callback_queue_;
};
typedef boost::shared_ptr<SubscriberCallbacks> SubscriberCallbacksPtr;

/**
 * \brief Structure passed as a parameter to the callback invoked by a ros::Timer
 */
struct TimerEvent
{
  Time last_expected;                     ///< In a perfect world, this is when the last callback should have happened
  Time last_real;                         ///< When the last callback actually happened

  Time current_expected;                  ///< In a perfect world, this is when the current callback should be happening
  Time current_real;                      ///< This is when the current callback was actually called (Time::now() as of the beginning of the callback)

  struct
  {
    WallDuration last_duration;           ///< How long the last callback ran for
  } profile;
};
typedef boost::function<void(const TimerEvent&)> TimerCallback;

/**
 * \brief Structure passed as a parameter to the callback invoked by a ros::WallTimer
 */
struct WallTimerEvent
{
  WallTime last_expected;                 ///< In a perfect world, this is when the last callback should have happened
  WallTime last_real;                     ///< When the last callback actually happened

  WallTime current_expected;              ///< In a perfect world, this is when the current callback should be happening
  WallTime current_real;                  ///< This is when the current callback was actually called (Time::now() as of the beginning of the callback)

  struct
  {
    WallDuration last_duration;           ///< How long the last callback ran for
  } profile;
};
typedef boost::function<void(const WallTimerEvent&)> WallTimerCallback;

class ServiceManager;
typedef boost::shared_ptr<ServiceManager> ServiceManagerPtr;
class TopicManager;
typedef boost::shared_ptr<TopicManager> TopicManagerPtr;
class ConnectionManager;
typedef boost::shared_ptr<ConnectionManager> ConnectionManagerPtr;
class XMLRPCManager;
typedef boost::shared_ptr<XMLRPCManager> XMLRPCManagerPtr;
class PollManager;
typedef boost::shared_ptr<PollManager> PollManagerPtr;

}

#endif
