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

#include "ros/connection.h"
#include "ros/transport/transport.h"
#include "ros/file_log.h"

#include <ros/assert.h>

#include <boost/shared_array.hpp>
#include <boost/bind.hpp>

namespace ros
{

Connection::Connection()
: is_server_(false)
, dropped_(false)
, read_filled_(0)
, read_size_(0)
, reading_(false)
, has_read_callback_(0)
, write_sent_(0)
, write_size_(0)
, writing_(false)
, has_write_callback_(0)
, sending_header_error_(false)
{
}

Connection::~Connection()
{
  ROS_DEBUG_NAMED("superdebug", "Connection destructing, dropped=%s", dropped_ ? "true" : "false");

  drop(Destructing);
}

void Connection::initialize(const TransportPtr& transport, bool is_server, const HeaderReceivedFunc& header_func)
{
  ROS_ASSERT(transport);

  transport_ = transport;
  header_func_ = header_func;
  is_server_ = is_server;

  transport_->setReadCallback(boost::bind(&Connection::onReadable, this, _1));
  transport_->setWriteCallback(boost::bind(&Connection::onWriteable, this, _1));
  transport_->setDisconnectCallback(boost::bind(&Connection::onDisconnect, this, _1));

  if (header_func)
  {
    read(4, boost::bind(&Connection::onHeaderLengthRead, this, _1, _2, _3, _4));
  }
}

boost::signals::connection Connection::addDropListener(const DropFunc& slot)
{
  boost::recursive_mutex::scoped_lock lock(drop_mutex_);
  return drop_signal_.connect(slot);
}

void Connection::removeDropListener(const boost::signals::connection& c)
{
  boost::recursive_mutex::scoped_lock lock(drop_mutex_);
  c.disconnect();
}

void Connection::onReadable(const TransportPtr& transport)
{
  ROS_ASSERT(transport == transport_);

  readTransport();
}

void Connection::readTransport()
{
  boost::recursive_mutex::scoped_try_lock lock(read_mutex_);

  if (!lock.owns_lock() || dropped_ || reading_)
  {
    return;
  }

  reading_ = true;

  while (!dropped_ && has_read_callback_)
  {
    ROS_ASSERT(read_buffer_);
    uint32_t to_read = read_size_ - read_filled_;
    if (to_read > 0)
    {
      int32_t bytes_read = transport_->read(read_buffer_.get() + read_filled_, to_read);
      ROS_DEBUG_NAMED("superdebug", "Connection read %d bytes", bytes_read);
      if (dropped_)
      {
        return;
      }
      else if (bytes_read < 0)
      {
        // Bad read, throw away results and report error
        ReadFinishedFunc callback;
        callback = read_callback_;
        read_callback_.clear();
        read_buffer_.reset();
        uint32_t size = read_size_;
        read_size_ = 0;
        read_filled_ = 0;
        has_read_callback_ = 0;

        if (callback)
        {
          callback(shared_from_this(), read_buffer_, size, false);
        }

        break;
      }

      read_filled_ += bytes_read;
    }

    ROS_ASSERT((int32_t)read_size_ >= 0);
    ROS_ASSERT((int32_t)read_filled_ >= 0);
    ROS_ASSERT_MSG(read_filled_ <= read_size_, "read_filled_ = %d, read_size_ = %d", read_filled_, read_size_);

    if (read_filled_ == read_size_ && !dropped_)
    {
      ReadFinishedFunc callback;
      uint32_t size;
      boost::shared_array<uint8_t> buffer;

      ROS_ASSERT(has_read_callback_);

      // store off the read info in case another read() call is made from within the callback
      callback = read_callback_;
      size = read_size_;
      buffer = read_buffer_;
      read_callback_.clear();
      read_buffer_.reset();
      read_size_ = 0;
      read_filled_ = 0;
      has_read_callback_ = 0;

      ROS_DEBUG_NAMED("superdebug", "Calling read callback");
      callback(shared_from_this(), buffer, size, true);
    }
    else
    {
      break;
    }
  }

  if (!has_read_callback_)
  {
    transport_->disableRead();
  }

  reading_ = false;
}

void Connection::writeTransport()
{
  boost::recursive_mutex::scoped_try_lock lock(write_mutex_);

  if (!lock.owns_lock() || dropped_ || writing_)
  {
    return;
  }

  writing_ = true;
  bool can_write_more = true;

  while (has_write_callback_ && can_write_more && !dropped_)
  {
    uint32_t to_write = write_size_ - write_sent_;
    ROS_DEBUG_NAMED("superdebug", "Connection writing %d bytes", to_write);
    int32_t bytes_sent = transport_->write(write_buffer_.get() + write_sent_, to_write);
    ROS_DEBUG_NAMED("superdebug", "Connection wrote %d bytes", bytes_sent);

    if (bytes_sent < 0)
    {
      writing_ = false;
      return;
    }

    write_sent_ += bytes_sent;

    if (bytes_sent < (int)write_size_ - (int)write_sent_)
    {
      can_write_more = false;
    }

    if (write_sent_ == write_size_ && !dropped_)
    {
      WriteFinishedFunc callback;

      {
        boost::mutex::scoped_lock lock(write_callback_mutex_);
        ROS_ASSERT(has_write_callback_);
        // Store off a copy of the callback in case another write() call happens in it
        callback = write_callback_;
        write_callback_ = WriteFinishedFunc();
        write_buffer_ = boost::shared_array<uint8_t>();
        write_sent_ = 0;
        write_size_ = 0;
        has_write_callback_ = 0;
      }

      ROS_DEBUG_NAMED("superdebug", "Calling write callback");
      callback(shared_from_this());
    }
  }

  {
    boost::mutex::scoped_lock lock(write_callback_mutex_);
    if (!has_write_callback_)
    {
      transport_->disableWrite();
    }
  }

  writing_ = false;
}

void Connection::onWriteable(const TransportPtr& transport)
{
  ROS_ASSERT(transport == transport_);

  writeTransport();
}

void Connection::read(uint32_t size, const ReadFinishedFunc& callback)
{
  if (dropped_ || sending_header_error_)
  {
    return;
  }

  {
    boost::recursive_mutex::scoped_lock lock(read_mutex_);

    ROS_ASSERT(!read_callback_);

    read_callback_ = callback;
    read_buffer_ = boost::shared_array<uint8_t>(new uint8_t[size]);
    read_size_ = size;
    read_filled_ = 0;
    has_read_callback_ = 1;
  }

  transport_->enableRead();

  // read immediately if possible
  readTransport();
}

void Connection::write(const boost::shared_array<uint8_t>& buffer, uint32_t size, const WriteFinishedFunc& callback, bool immediate)
{
  if (dropped_ || sending_header_error_)
  {
    return;
  }

  {
    boost::mutex::scoped_lock lock(write_callback_mutex_);

    ROS_ASSERT(!write_callback_);

    write_callback_ = callback;
    write_buffer_ = buffer;
    write_size_ = size;
    write_sent_ = 0;
    has_write_callback_ = 1;
  }

  transport_->enableWrite();

  if (immediate)
  {
    // write immediately if possible
    writeTransport();
  }
}

void Connection::onDisconnect(const TransportPtr& transport)
{
  ROS_ASSERT(transport == transport_);

  drop(TransportDisconnect);
}

void Connection::drop(DropReason reason)
{
  ROSCPP_LOG_DEBUG("Connection::drop(%u)", reason);
  bool did_drop = false;
  {
    boost::recursive_mutex::scoped_lock lock(drop_mutex_);
    if (!dropped_)
    {
      dropped_ = true;
      did_drop = true;

      drop_signal_(shared_from_this(), reason);
    }
  }

  if (did_drop)
  {
    transport_->close();
  }
}

bool Connection::isDropped()
{
  boost::recursive_mutex::scoped_lock lock(drop_mutex_);
  return dropped_;
}

void Connection::writeHeader(const M_string& key_vals, const WriteFinishedFunc& finished_callback)
{
  ROS_ASSERT(!header_written_callback_);
  header_written_callback_ = finished_callback;

  if (!transport_->requiresHeader())
  {
    onHeaderWritten(shared_from_this());
    return;
  }

  boost::shared_array<uint8_t> buffer;
  uint32_t len;
  Header::write(key_vals, buffer, len);

  uint32_t msg_len = len + 4;
  boost::shared_array<uint8_t> full_msg(new uint8_t[msg_len]);
  memcpy(full_msg.get() + 4, buffer.get(), len);
  *((uint32_t*)full_msg.get()) = len;

  write(full_msg, msg_len, boost::bind(&Connection::onHeaderWritten, this, _1), false);
}

void Connection::sendHeaderError(const std::string& error_msg)
{
  M_string m;
  m["error"] = error_msg;

  writeHeader(m, boost::bind(&Connection::onErrorHeaderWritten, this, _1));
  sending_header_error_ = true;
}

void Connection::onHeaderLengthRead(const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success)
{
  ROS_ASSERT(conn.get() == this);
  ROS_ASSERT(size == 4);

  if (!success)
    return;

  uint32_t len = *((uint32_t*)buffer.get());

  if (len > 1000000000)
  {
    ROS_ERROR("a header of over a gigabyte was " \
                "predicted in tcpros. that seems highly " \
                "unlikely, so I'll assume protocol " \
                "synchronization is lost.");
    conn->drop(HeaderError);
  }

  read(len, boost::bind(&Connection::onHeaderRead, this, _1, _2, _3, _4));
}

void Connection::onHeaderRead(const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success)
{
  ROS_ASSERT(conn.get() == this);

  if (!success)
    return;

  std::string error_msg;
  if (!header_.parse(buffer, size, error_msg))
  {
    drop(HeaderError);
  }
  else
  {
    std::string error_val;
    if (header_.getValue("error", error_val))
    {
      ROSCPP_LOG_DEBUG("Received error message in header for connection to [%s]: [%s]", transport_->getTransportInfo().c_str(), error_val.c_str());
      drop(HeaderError);
    }
    else
    {
      ROS_ASSERT(header_func_);

      transport_->parseHeader(header_);

      header_func_(conn, header_);
    }
  }

}

void Connection::onHeaderWritten(const ConnectionPtr& conn)
{
  ROS_ASSERT(conn.get() == this);
  ROS_ASSERT(header_written_callback_);

  header_written_callback_(conn);
  header_written_callback_ = WriteFinishedFunc();
}

void Connection::onErrorHeaderWritten(const ConnectionPtr& conn)
{
  drop(HeaderError);
}

void Connection::setHeaderReceivedCallback(const HeaderReceivedFunc& func)
{
  header_func_ = func;

  if (transport_->requiresHeader())
    read(4, boost::bind(&Connection::onHeaderLengthRead, this, _1, _2, _3, _4));
}

std::string Connection::getCallerId()
{
  std::string callerid;
  if (header_.getValue("callerid", callerid))
  {
    return callerid;
  }

  return std::string("unknown");
}

std::string Connection::getRemoteString()
{
  std::stringstream ss;
  ss << "callerid=[" << getCallerId() << "] address=[" << transport_->getTransportInfo() << "]";
  return ss.str();
}

}
