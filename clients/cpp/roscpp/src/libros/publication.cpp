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
#include "ros/serialization.h"
#include <std_msgs/Header.h>

namespace ros
{

class PeerConnDisconnCallback : public CallbackInterface
{
public:
  PeerConnDisconnCallback(const SubscriberStatusCallback& callback, const SubscriberLinkPtr& sub_link, bool use_tracked_object, const VoidConstWPtr& tracked_object)
  : callback_(callback)
  , sub_link_(sub_link)
  , use_tracked_object_(use_tracked_object)
  , tracked_object_(tracked_object)
  {
  }

  virtual CallResult call()
  {
    VoidConstPtr tracker;
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
  VoidConstWPtr tracked_object_;
};

Publication::Publication(const std::string &name,
                         const std::string &datatype,
                         const std::string &_md5sum,
                         const std::string& message_definition,
                         size_t max_queue,
                         bool latch,
                         bool has_header)
: name_(name),
  datatype_(datatype),
  md5sum_(_md5sum),
  message_definition_(message_definition),
  max_queue_(max_queue),
  seq_(0),
  dropped_(false),
  latch_(latch),
  has_header_(has_header),
  intraprocess_subscriber_count_(0)
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

  // Add connect callbacks for all current subscriptions if this publisher wants them
  if (callbacks->connect_ && callbacks->callback_queue_)
  {
    boost::mutex::scoped_lock lock(subscriber_links_mutex_);
    V_SubscriberLink::iterator it = subscriber_links_.begin();
    V_SubscriberLink::iterator end = subscriber_links_.end();
    for (; it != end; ++it)
    {
      const SubscriberLinkPtr& sub_link = *it;
      CallbackInterfacePtr cb(new PeerConnDisconnCallback(callbacks->connect_, sub_link, callbacks->has_tracked_object_, callbacks->tracked_object_));
      callbacks->callback_queue_->addCallback(cb, (uint64_t)callbacks.get());
    }
  }
}

void Publication::removeCallbacks(const SubscriberCallbacksPtr& callbacks)
{
  boost::mutex::scoped_lock lock(callbacks_mutex_);

  V_Callback::iterator it = std::find(callbacks_.begin(), callbacks_.end(), callbacks);
  if (it != callbacks_.end())
  {
    const SubscriberCallbacksPtr& cb = *it;
    if (cb->callback_queue_)
    {
      cb->callback_queue_->removeByID((uint64_t)cb.get());
    }
    callbacks_.erase(it);
  }
}

void Publication::drop()
{
  // grab a lock here, to ensure that no subscription callback will
  // be invoked after we return
  {
    boost::mutex::scoped_lock lock(publish_queue_mutex_);
    boost::mutex::scoped_lock lock2(subscriber_links_mutex_);

    if (dropped_)
    {
      return;
    }

    dropped_ = true;
  }

  dropAllConnections();
}

bool Publication::enqueueMessage(const SerializedMessage& m)
{
  boost::mutex::scoped_lock lock(subscriber_links_mutex_);
  if (dropped_)
  {
    return false;
  }

  ROS_ASSERT(m.buf);

  uint32_t seq = incrementSequence();
  if (has_header_)
  {
    // If we have a header, we know it's immediately after the message length
    // Deserialize it, write the sequence, and then serialize it again.
    namespace ser = ros::serialization;
    std_msgs::Header header;
    ser::IStream istream(m.buf.get() + 4, m.num_bytes - 4);
    ser::deserialize(istream, header);
    header.seq = seq;
    ser::OStream ostream(m.buf.get() + 4, m.num_bytes - 4);
    ser::serialize(ostream, header);
  }

  for(V_SubscriberLink::iterator i = subscriber_links_.begin();
      i != subscriber_links_.end(); ++i)
  {
    const SubscriberLinkPtr& sub_link = (*i);
    sub_link->enqueueMessage(m, true, false);
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

    if (dropped_)
    {
      return;
    }

    subscriber_links_.push_back(sub_link);

    if (sub_link->isIntraprocess())
    {
      ++intraprocess_subscriber_count_;
    }
  }

  if (latch_ && last_message_.buf)
  {
    sub_link->enqueueMessage(last_message_, true, true);
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

    if (dropped_)
    {
      return;
    }

    if (sub_link->isIntraprocess())
    {
      --intraprocess_subscriber_count_;
    }

    V_SubscriberLink::iterator it = std::find(subscriber_links_.begin(), subscriber_links_.end(), sub_link);
    if (it != subscriber_links_.end())
    {
      link = *it;
      subscriber_links_.erase(it);
    }
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
    (*i)->drop();
  }
}

void Publication::peerConnect(const SubscriberLinkPtr& sub_link)
{
  V_Callback::iterator it = callbacks_.begin();
  V_Callback::iterator end = callbacks_.end();
  for (; it != end; ++it)
  {
    const SubscriberCallbacksPtr& cbs = *it;
    if (cbs->connect_ && cbs->callback_queue_)
    {
      CallbackInterfacePtr cb(new PeerConnDisconnCallback(cbs->connect_, sub_link, cbs->has_tracked_object_, cbs->tracked_object_));
      cbs->callback_queue_->addCallback(cb, (uint64_t)cbs.get());
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
    if (cbs->disconnect_ && cbs->callback_queue_)
    {
      CallbackInterfacePtr cb(new PeerConnDisconnCallback(cbs->disconnect_, sub_link, cbs->has_tracked_object_, cbs->tracked_object_));
      cbs->callback_queue_->addCallback(cb, (uint64_t)cbs.get());
    }
  }
}

size_t Publication::getNumCallbacks()
{
  boost::mutex::scoped_lock lock(callbacks_mutex_);
  return callbacks_.size();
}

uint32_t Publication::incrementSequence()
{
  boost::mutex::scoped_lock lock(seq_mutex_);
  uint32_t old_seq = seq_;
  ++seq_;

  return old_seq;
}

uint32_t Publication::getNumSubscribers()
{
  boost::mutex::scoped_lock lock(subscriber_links_mutex_);
  return (uint32_t)subscriber_links_.size();
}

void Publication::getPublishTypes(bool& serialize, bool& nocopy, const std::type_info& ti)
{
  boost::mutex::scoped_lock lock(subscriber_links_mutex_);
  V_SubscriberLink::const_iterator it = subscriber_links_.begin();
  V_SubscriberLink::const_iterator end = subscriber_links_.end();
  for (; it != end; ++it)
  {
    const SubscriberLinkPtr& sub = *it;
    bool s = false;
    bool n = false;
    sub->getPublishTypes(s, n, ti);
    serialize = serialize || s;
    nocopy = nocopy || n;

    if (serialize && nocopy)
    {
      break;
    }
  }
}

bool Publication::hasSubscribers()
{
  boost::mutex::scoped_lock lock(subscriber_links_mutex_);
  return !subscriber_links_.empty();
}

void Publication::publish(SerializedMessage& m)
{
  if (m.message)
  {
    boost::mutex::scoped_lock lock(subscriber_links_mutex_);
    V_SubscriberLink::const_iterator it = subscriber_links_.begin();
    V_SubscriberLink::const_iterator end = subscriber_links_.end();
    for (; it != end; ++it)
    {
      const SubscriberLinkPtr& sub = *it;
      if (sub->isIntraprocess())
      {
        sub->enqueueMessage(m, false, true);
      }
    }

    m.message.reset();
  }

  if (m.buf)
  {
    boost::mutex::scoped_lock lock(publish_queue_mutex_);
    publish_queue_.push_back(m);
  }
}

void Publication::processPublishQueue()
{
  V_SerializedMessage queue;
  {
    boost::mutex::scoped_lock lock(publish_queue_mutex_);

    if (dropped_)
    {
      return;
    }

    queue.insert(queue.end(), publish_queue_.begin(), publish_queue_.end());
    publish_queue_.clear();
  }

  if (queue.empty())
  {
    return;
  }

  V_SerializedMessage::iterator it = queue.begin();
  V_SerializedMessage::iterator end = queue.end();
  for (; it != end; ++it)
  {
    enqueueMessage(*it);
  }
}

bool Publication::validateHeader(const Header& header, std::string& error_msg)
{
  std::string md5sum, topic, client_callerid;
  if (!header.getValue("md5sum", md5sum)
   || !header.getValue("topic", topic)
   || !header.getValue("callerid", client_callerid))
  {
    std::string msg("Header from subscriber did not have the required elements: md5sum, topic, callerid");

    ROS_ERROR("%s", msg.c_str());
    error_msg = msg;

    return false;
  }

  // Check whether the topic has been deleted from
  // advertised_topics through a call to unadvertise(), which could
  // have happened while we were waiting for the subscriber to
  // provide the md5sum.
  if(isDropped())
  {
    std::string msg = std::string("received a tcpros connection for a nonexistent topic [") +
                topic + std::string("] from [" + client_callerid +"].");

    ROS_ERROR("%s", msg.c_str());
    error_msg = msg;

    return false;
  }

  if (getMD5Sum() != md5sum &&
      (md5sum != std::string("*") && getMD5Sum() != std::string("*")))
  {
    std::string datatype;
    header.getValue("type", datatype);

    std::string msg = std::string("Client [") + client_callerid + std::string("] wants topic ") + topic +
                      std::string(" to have datatype/md5sum [") + datatype + "/" + md5sum +
                      std::string("], but our version has [") + getDataType() + "/" + getMD5Sum() +
                      std::string("]. Dropping connection.");

    ROS_ERROR("%s", msg.c_str());
    error_msg = msg;

    return false;
  }

  return true;
}

} // namespace ros
