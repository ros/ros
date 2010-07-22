/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/intraprocess_publisher_link.h"
#include "ros/intraprocess_subscriber_link.h"
#include "ros/subscription.h"
#include "ros/header.h"
#include "ros/connection.h"
#include "ros/transport/transport.h"
#include "ros/this_node.h"
#include "ros/connection_manager.h"
#include "ros/file_log.h"

#include <boost/bind.hpp>

#include <sstream>

namespace ros
{

IntraProcessPublisherLink::IntraProcessPublisherLink(const SubscriptionPtr& parent, const std::string& xmlrpc_uri, const TransportHints& transport_hints)
: PublisherLink(parent, xmlrpc_uri, transport_hints)
, dropped_(false)
{
}

IntraProcessPublisherLink::~IntraProcessPublisherLink()
{
}

void IntraProcessPublisherLink::setPublisher(const IntraProcessSubscriberLinkPtr& publisher)
{
  publisher_ = publisher;

  SubscriptionPtr parent = parent_.lock();
  ROS_ASSERT(parent);

  Header header;
  M_stringPtr values = header.getValues();
  (*values)["callerid"] = this_node::getName();
  (*values)["topic"] = parent->getName();
  (*values)["type"] = publisher->getDataType();
  (*values)["md5sum"] = publisher->getMD5Sum();
  (*values)["message_definition"] = publisher->getMessageDefinition();
  (*values)["latching"] = publisher->isLatching() ? "1" : "0";
  setHeader(header);
}

void IntraProcessPublisherLink::drop()
{
  {
    boost::recursive_mutex::scoped_lock lock(drop_mutex_);
    if (dropped_)
    {
      return;
    }

    dropped_ = true;
  }

  if (publisher_)
  {
    publisher_->drop();
    publisher_.reset();
  }

  if (SubscriptionPtr parent = parent_.lock())
  {
    ROSCPP_LOG_DEBUG("Connection to local publisher on topic [%s] dropped", parent->getName().c_str());

    parent->removePublisherLink(shared_from_this());
  }
}

void IntraProcessPublisherLink::handleMessage(const SerializedMessage& m, bool ser, bool nocopy)
{
  boost::recursive_mutex::scoped_lock lock(drop_mutex_);
  if (dropped_)
  {
    return;
  }

  stats_.bytes_received_ += m.num_bytes;
  stats_.messages_received_++;

  SubscriptionPtr parent = parent_.lock();

  if (parent)
  {
    stats_.drops_ += parent->handleMessage(m, ser, nocopy, header_.getValues(), shared_from_this());
  }
}

std::string IntraProcessPublisherLink::getTransportType()
{
  return std::string("INTRAPROCESS");
}

void IntraProcessPublisherLink::getPublishTypes(bool& ser, bool& nocopy, const std::type_info& ti)
{
  boost::recursive_mutex::scoped_lock lock(drop_mutex_);
  if (dropped_)
  {
    ser = false;
    nocopy = false;
    return;
  }

  SubscriptionPtr parent = parent_.lock();
  if (parent)
  {
    parent->getPublishTypes(ser, nocopy, ti);
  }
  else
  {
    ser = true;
    nocopy = false;
  }
}

} // namespace ros

