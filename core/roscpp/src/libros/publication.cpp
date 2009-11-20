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

#include "ros/publication.h"
#include "ros/subscriber_link.h"
#include "ros/connection.h"
#include "ros/callback_queue_interface.h"
#include "ros/single_subscriber_publisher.h"

namespace ros
{

Publication::Publication(const std::string &name,
                         const std::string &datatype,
                         const std::string &_md5sum,
                         const std::string& message_definition,
                         size_t max_queue,
                         CallbackQueueInterface* callback_queue,
                         bool latch)
: name_(name),
  datatype_(datatype),
  md5sum_(_md5sum),
  message_definition_(message_definition),
  max_queue_(max_queue),
  seq_(0),
  dropped_(false),
  refcount_(1),
  callback_queue_(callback_queue),
  latch_(latch)
{
}

Publication::~Publication()
{
  drop();
}

void Publication::addCallbacks(const SubscriberCallbacksPtr& callbacks)
{
  boost::mutex::scoped_lock lock(callbacks_mutex_);

  callbacks_.push_back(callbacks);
}

void Publication::removeCallbacks(const SubscriberCallbacksPtr& callbacks)
{
  boost::mutex::scoped_lock lock(callbacks_mutex_);

  V_Callback::iterator it = std::find(callbacks_.begin(), callbacks_.end(), callbacks);
  if (it != callbacks_.end())
  {
    callbacks_.erase(it);
  }
}

void Publication::drop()
{
  // grab a lock here, to ensure that no subscription callback will
  // be invoked after we return
  {
    boost::mutex::scoped_lock lock(subscriber_links_mutex_);

    dropped_ = true;
  }

  dropAllConnections();
}

bool Publication::enqueueMessage(const SerializedMessage& m)
{
  // This is somewhat nasty, but prevents a deadlock in the case where the subscriber connection callback has called publish()
  // TODO this can change once the old roscpp API is removed
  V_SubscriberLink local_publishers;
  {
    boost::mutex::scoped_lock lock(subscriber_links_mutex_);

    if (dropped_)
    {
      return false;
    }

    local_publishers = subscriber_links_;
  }

  for(V_SubscriberLink::iterator i = local_publishers.begin();
      i != local_publishers.end(); ++i)
  {
    const SubscriberLinkPtr& sub_link = (*i);
    sub_link->enqueueMessage(m);
  }

  if (latch_)
  {
    last_message_ = m;
  }

  return true;
}

void Publication::addSubscriberLink(const SubscriberLinkPtr& sub_link)
{
  {
    boost::mutex::scoped_lock lock(subscriber_links_mutex_);

    subscriber_links_.push_back(sub_link);
  }

  if (dropped_)
  {
    return;
  }

  if (latch_ && last_message_.buf)
  {
    sub_link->enqueueMessage(last_message_);
  }

  // This call invokes the subscribe callback if there is one.
  // This must happen *after* the push_back above, in case the
  // callback uses publish().
  peerConnect(sub_link);
}

void Publication::removeSubscriberLink(const SubscriberLinkPtr& sub_link)
{
  SubscriberLinkPtr link;
  {
    boost::mutex::scoped_lock lock(subscriber_links_mutex_);

    V_SubscriberLink::iterator it = std::find(subscriber_links_.begin(), subscriber_links_.end(), sub_link);
    if (it != subscriber_links_.end())
    {
      link = *it;
      subscriber_links_.erase(it);
    }
  }

  if (dropped_)
  {
    return;
  }

  if (link)
  {
    peerDisconnect(link);
  }
}

XmlRpc::XmlRpcValue Publication::getStats()
{
  XmlRpc::XmlRpcValue stats;
  stats[0] = name_;
  XmlRpc::XmlRpcValue conn_data;
  conn_data.setSize(0); // force to be an array, even if it's empty

  boost::mutex::scoped_lock lock(subscriber_links_mutex_);

  uint32_t cidx = 0;
  for (V_SubscriberLink::iterator c = subscriber_links_.begin();
       c != subscriber_links_.end(); ++c, cidx++)
  {
    const SubscriberLink::Stats& s = (*c)->getStats();
    conn_data[cidx][0] = (*c)->getConnectionID();
    // todo: figure out what to do here... the bytes_sent will wrap around
    // on some flows within a reasonable amount of time. xmlrpc++ doesn't
    // seem to give me a nice way to do 64-bit ints, perhaps that's a
    // limitation of xml-rpc, not sure. alternatively we could send the number
    // of KB transmitted to gain a few order of magnitude.
    conn_data[cidx][1] = (int)s.bytes_sent_;
    conn_data[cidx][2] = (int)s.message_data_sent_;
    conn_data[cidx][3] = (int)s.messages_sent_;
    conn_data[cidx][4] = 0; // not sure what is meant by connected
  }

  stats[1] = conn_data;
  return stats;
}

// rospy returns values like this:
// [(1, "('127.0.0.1', 62334)", 'o', 'TCPROS', '/chatter')]
//
// We're outputting something like this:
// (0, (127.0.0.1, 62707), o, TCPROS, /chatter)
void Publication::getInfo(XmlRpc::XmlRpcValue& info)
{
  boost::mutex::scoped_lock lock(subscriber_links_mutex_);

  for (V_SubscriberLink::iterator c = subscriber_links_.begin();
       c != subscriber_links_.end(); ++c)
  {
    XmlRpc::XmlRpcValue curr_info;
    curr_info[0] = (int)(*c)->getConnectionID();
    curr_info[1] = (*c)->getDestinationCallerID();
    curr_info[2] = "o";
    curr_info[3] = (*c)->getTransportType();
    curr_info[4] = name_;
    info[info.size()] = curr_info;
  }
}

void Publication::dropAllConnections()
{
  // Swap our publishers list with a local one so we can only lock for a short period of time, because a
  // side effect of our calling drop() on connections can be re-locking the publishers mutex
  V_SubscriberLink local_publishers;

  {
    boost::mutex::scoped_lock lock(subscriber_links_mutex_);

    local_publishers.swap(subscriber_links_);
  }

  for (V_SubscriberLink::iterator i = local_publishers.begin();
           i != local_publishers.end(); ++i)
  {
    (*i)->getConnection()->drop();
  }
}

class PeerConnDisconnCallback : public CallbackInterface
{
public:
  PeerConnDisconnCallback(const SubscriberStatusCallback& callback, const SubscriberLinkPtr& sub_link, bool use_tracked_object, const VoidWPtr& tracked_object)
  : callback_(callback)
  , sub_link_(sub_link)
  , use_tracked_object_(use_tracked_object)
  , tracked_object_(tracked_object)
  {
  }

  virtual CallResult call()
  {
    VoidPtr tracker;
    if (use_tracked_object_)
    {
      tracker = tracked_object_.lock();

      if (!tracker)
      {
        return Invalid;
      }
    }

    SingleSubscriberPublisher pub(sub_link_);
    callback_(pub);

    return Success;
  }

private:
  SubscriberStatusCallback callback_;
  SubscriberLinkPtr sub_link_;
  bool use_tracked_object_;
  VoidWPtr tracked_object_;
};

void Publication::peerConnect(const SubscriberLinkPtr& sub_link)
{
  V_Callback::iterator it = callbacks_.begin();
  V_Callback::iterator end = callbacks_.end();
  for (; it != end; ++it)
  {
    const SubscriberCallbacksPtr& cbs = *it;
    if (cbs->connect_)
    {
      if (callback_queue_)
      {
        CallbackInterfacePtr cb(new PeerConnDisconnCallback(cbs->connect_, sub_link, cbs->has_tracked_object_, cbs->tracked_object_));
        callback_queue_->addCallback(cb);
      }
      else
      {
        cbs->connect_(sub_link);
      }
    }
  }
}

void Publication::peerDisconnect(const SubscriberLinkPtr& sub_link)
{
  V_Callback::iterator it = callbacks_.begin();
  V_Callback::iterator end = callbacks_.end();
  for (; it != end; ++it)
  {
    const SubscriberCallbacksPtr& cbs = *it;
    if (cbs->disconnect_)
    {
      if (callback_queue_)
      {
        CallbackInterfacePtr cb(new PeerConnDisconnCallback(cbs->disconnect_, sub_link, cbs->has_tracked_object_, cbs->tracked_object_));
        callback_queue_->addCallback(cb);
      }
      else
      {
        cbs->disconnect_(sub_link);
      }
    }
  }
}

void Publication::incrementRefcount()
{
  ++refcount_;
}

void Publication::decrementRefcount()
{
  --refcount_;
}

uint32_t Publication::getRefcount()
{
  return refcount_;
}

} // namespace ros
