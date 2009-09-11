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

#include "ros/publisher.h"
#include "ros/publication.h"
#include "ros/header.h"
#include "ros/connection.h"
#include "ros/node.h"
#include "ros/transport/transport.h"

#include <boost/bind.hpp>

namespace ros
{

Publisher::Publisher()
: writing_message_(false)
, header_written_(false)
, queue_full_(false)
, immediate_write_on_publish_(true)
{

}

Publisher::~Publisher()
{

}

bool Publisher::initialize(const ConnectionPtr& connection)
{
  connection_ = connection;
  connection_->addDropListener(boost::bind(&Publisher::onConnectionDropped, this, _1));

  return true;
}

void Publisher::setImmediateWriteOnPublish(bool immediate)
{
  immediate_write_on_publish_ = immediate;
}

bool Publisher::handleHeader(const Header& header)
{
  std::string md5sum, topic, client_callerid;
  if (!header.getValue("md5sum", md5sum)
   || !header.getValue("topic", topic)
   || !header.getValue("callerid", client_callerid))
  {
    std::string msg("TCPROS header from subscriber did not have the required elements: md5sum, topic, callerid");

    ROS_ERROR("%s", msg.c_str());
    connection_->sendHeaderError(msg);

    return false;
  }

  ROS_DEBUG("Client [%s] wants topic [%s] with md5sum [%s]", client_callerid.c_str(), topic.c_str(), md5sum.c_str());
  PublicationPtr pt = g_node->lookupTopic(topic);
  if (!pt)
  {
    std::string msg = std::string("received a tcpros connection for a nonexistent topic [") +
                    topic + std::string("] from [" + connection_->getTransport()->getTransportInfo() + "] [" + client_callerid +"].");

    ROS_ERROR("%s", msg.c_str());
    connection_->sendHeaderError(msg);

    return false;
  }

  if (pt->getMD5Sum() != md5sum &&
      (md5sum != std::string("*") && pt->getMD5Sum() != std::string("*")))
  {
    std::string datatype;
    header.getValue("type", datatype);

    std::string msg;
    if (!datatype.empty() && datatype != pt->getDataType())
    {
      msg = std::string("Client [") + client_callerid + std::string("] wants topic ") + topic +
                    std::string(" to have datatype ") + datatype +
                    std::string(", but it has ") + pt->getDataType() +
                    std::string(". Dropping connection.");
    }
    else
    {
      msg = std::string("Client [") + client_callerid + std::string("] wants topic ") + topic +
              std::string(" to have md5sum ") + md5sum +
              std::string(", but it has ") + pt->getMD5Sum() +
              std::string(". Dropping connection.");
    }

    ROS_ERROR("%s", msg.c_str());
    connection_->sendHeaderError(msg);

    return false;
  }

  // Check whether the topic (pt here) has been deleted from
  // advertised_topics through a call to unadvertise(), which could
  // have happened while we were waiting for the subscriber to
  // provide the md5sum.
  if(pt->isDropped())
  {
    std::string msg = std::string("received a tcpros connection for a nonexistent topic [") +
                topic + std::string("] from [" + connection_->getTransport()->getTransportInfo() + "] [" + client_callerid +"].");

    ROS_ERROR("%s", msg.c_str());
    connection_->sendHeaderError(msg);

    return false;
  }
  else
  {
    destination_caller_id_ = client_callerid;
    connection_id_ = g_node->getNewConnectionID();
    parent_ = PublicationWPtr(pt);

    // Send back a success, with info
    M_string m;
    m["type"] = pt->getDataType();
    m["md5sum"] = pt->getMD5Sum();
    m["callerid"] = g_node->getName();
    connection_->writeHeader(m, boost::bind(&Publisher::onHeaderWritten, this, _1));

    pt->addPublisher(shared_from_this());
  }

  return true;
}

void Publisher::onConnectionDropped(const ConnectionPtr& conn)
{
  ROS_ASSERT(conn == connection_);

  PublicationPtr parent = parent_.lock();

  if (parent)
  {
    ROS_DEBUG("Connection to subscriber [%s] to topic [%s] dropped", connection_->getTransport()->getTransportInfo().c_str(), parent->getName().c_str());

    parent->removePublisher(shared_from_this());
  }
}

void Publisher::onHeaderWritten(const ConnectionPtr& conn)
{
  header_written_ = true;
  startMessageWrite(true);
}

void Publisher::onMessageWritten(const ConnectionPtr& conn)
{
  writing_message_ = false;
  startMessageWrite(true);
}

void Publisher::startMessageWrite(bool immediate_write)
{
  SerializedMessage m(boost::shared_array<uint8_t>(), 0);

  {
    boost::mutex::scoped_lock lock(outbox_mutex_);
    if (writing_message_ || !header_written_)
    {
      return;
    }

    if (!outbox_.empty())
    {
      writing_message_ = true;
      m = outbox_.front();
      outbox_.pop();
    }
  }

  if (m.num_bytes > 0)
  {
    connection_->write(m.buf, m.num_bytes, boost::bind(&Publisher::onMessageWritten, this, _1), immediate_write);
  }
}

bool Publisher::publish(Message& m)
{
  if (!verifyDatatype(m.__getDataType()))
  {
    return false;
  }

  uint32_t msg_len = m.serializationLength();
  boost::shared_array<uint8_t> buf = boost::shared_array<uint8_t>(new uint8_t[msg_len + 4]);
  *((uint32_t*)buf.get()) = msg_len;

  int seq = 0;
  if (PublicationPtr parent = parent_.lock())
  {
    seq = parent->getSequence();
  }

  m.serialize(buf.get() + 4, seq);
  enqueueMessage(buf, msg_len + 4);

  return true;
}

void Publisher::enqueueMessage(boost::shared_array<uint8_t> buf, size_t num_bytes)
{
  SerializedMessage m(buf, num_bytes);

  {
    boost::mutex::scoped_lock lock(outbox_mutex_);

    int max_queue = 0;
    std::string parent_name;
    if (PublicationPtr parent = parent_.lock())
    {
      max_queue = parent->getMaxQueue();
      parent_name = parent->getName();
    }

    ROS_DEBUG_NAMED("superdebug", "Publisher on topic [%s] to caller [%s], queueing message (queue size [%d])", parent_name.c_str(), destination_caller_id_.c_str(), (int)outbox_.size());

    if (max_queue > 0 && (int)outbox_.size() >= max_queue)
    {
      if (!queue_full_)
      {
        ROS_DEBUG("Outgoing queue full for topic \"%s\".  "
               "Discarding oldest message\n",
               parent_name.c_str());
      }

      outbox_.pop(); // toss out the oldest thing in the queue to make room for us
      queue_full_ = true;
    }
    else
    {
      queue_full_ = false;
    }

    outbox_.push(m);
  }

  startMessageWrite(immediate_write_on_publish_);

  stats_.messages_sent_++;
  stats_.bytes_sent_ += num_bytes;
  stats_.message_data_sent_ += num_bytes;
}

bool Publisher::verifyDatatype(const std::string &datatype)
{
  PublicationPtr parent = parent_.lock();
  if (!parent)
  {
    ROS_ERROR("Trying to verify the datatype on a publisher without a parent");
    ROS_BREAK();

    return false;
  }

  if (datatype != parent->getDataType())
  {
    ROS_ERROR( "tried to send a message with type %s on a " \
                       "Publisher that has datatype %s",
                datatype.c_str(), parent->getDataType().c_str());
    return false; // todo: figure out a way to log this error
  }

  return true;
}

std::string Publisher::getTransportType()
{
  return connection_->getTransport()->getType();
}

} // namespace ros
