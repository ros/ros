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
#include "ros/publisher.h"
#include "ros/connection.h"
#include "ros/node.h"

using namespace XmlRpc;
using namespace std;

namespace ros
{

Publication::Publication(const std::string &name, const std::string &_original_name,
            const std::string &datatype,
            const std::string &_md5sum,
            const SubscriptionConnectionCallback& connect_cb,
            const SubscriptionConnectionCallback& disconnect_cb,
            size_t max_queue)
  : name_(name), original_name_(_original_name),
    datatype_(datatype), md5sum_(_md5sum),
    sub_connect_cb_(connect_cb),
    sub_disconnect_cb_(disconnect_cb), max_queue_(max_queue), seq_(0), dropped_(false)
{
}

Publication::~Publication()
{
  drop();
}

void Publication::drop()
{
  // grab a lock here, to ensure that no subscription callback will
  // be invoked after we return
  {
    boost::mutex::scoped_lock lock(publishers_mutex_);

    dropped_ = true;
  }

  dropAllConnections();
}

bool Publication::enqueueMessage(boost::shared_array<uint8_t> buf, size_t num_bytes)
{
  // This is somewhat nasty, but prevents a deadlock in the case where the subscriber connection callback has called publish()
  // TODO
  V_Publisher local_publishers;
  {
    boost::mutex::scoped_lock lock(publishers_mutex_);

    local_publishers = publishers_;
  }

  for(vector<PublisherPtr>::iterator i = local_publishers.begin();
      i != local_publishers.end(); ++i)
  {
    const PublisherPtr& publisher = (*i);
    publisher->enqueueMessage(buf, num_bytes);
  }

  return true;
}

void Publication::addPublisher(const PublisherPtr& publisher)
{
  {
    boost::mutex::scoped_lock lock(publishers_mutex_);

    publishers_.push_back(publisher);
  }

  if (dropped_)
  {
    return;
  }

  // This call invokes the subscribe callback if there is one.
  // This must happen *after* the push_back above, in case the
  // callback uses publish().
  peerConnect(publisher);
}

void Publication::removePublisher(const PublisherPtr& publisher)
{
  PublisherPtr pub;
  {
    boost::mutex::scoped_lock lock(publishers_mutex_);

    std::vector<PublisherPtr>::iterator it = std::find(publishers_.begin(), publishers_.end(), publisher);
    if (it != publishers_.end())
    {
      pub = *it;
      publishers_.erase(it);
    }
  }

  if (dropped_)
  {
    return;
  }

  peerDisconnect(pub);
}

XmlRpcValue Publication::getStats()
{
  XmlRpcValue stats;
  stats[0] = name_;
  XmlRpcValue conn_data;
  conn_data.setSize(0); // force to be an array, even if it's empty

  boost::mutex::scoped_lock lock(publishers_mutex_);

  uint32_t cidx = 0;
  for (vector<PublisherPtr>::iterator c = publishers_.begin();
       c != publishers_.end(); ++c, cidx++)
  {
    const Publisher::Stats& s = (*c)->getStats();
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
void Publication::getInfo(XmlRpcValue& info)
{
  boost::mutex::scoped_lock lock(publishers_mutex_);

  for (vector<PublisherPtr>::iterator c = publishers_.begin();
       c != publishers_.end(); ++c)
  {
    XmlRpcValue curr_info;
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
  vector<PublisherPtr> local_publishers;

  {
    boost::mutex::scoped_lock lock(publishers_mutex_);

    local_publishers.swap(publishers_);
  }

  for (vector<PublisherPtr>::iterator i = local_publishers.begin();
           i != local_publishers.end(); ++i)
  {
    (*i)->getConnection()->drop();
  }
}

void Publication::peerConnect(const PublisherPtr& publisher)
{
  if (sub_connect_cb_)
  {
    sub_connect_cb_(publisher);
  }
}

void Publication::peerDisconnect(const PublisherPtr& publisher)
{
  if (sub_disconnect_cb_)
  {
    sub_disconnect_cb_(publisher);
  }
}

} // namespace ros
