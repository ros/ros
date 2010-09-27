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

#include "ros/transport/transport_udp.h"
#include "ros/poll_set.h"
#include "ros/file_log.h"

#include <ros/assert.h>

#include <boost/bind.hpp>

#include <sys/uio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>

namespace ros
{

TransportUDP::TransportUDP(PollSet* poll_set, int flags, int max_datagram_size)
: sock_(-1)
, closed_(false)
, expecting_read_(false)
, expecting_write_(false)
, is_server_(false)
, server_port_(-1)
, poll_set_(poll_set)
, flags_(flags)
, connection_id_(0)
, current_message_id_(0)
, total_blocks_(0)
, last_block_(0)
, max_datagram_size_(max_datagram_size)
, data_filled_(0)
, reorder_buffer_(0)
, reorder_bytes_(0)
{
  // This may eventually be machine dependent
  if (max_datagram_size_ == 0)
    max_datagram_size_ = 1500;
  reorder_buffer_ = new uint8_t[max_datagram_size_];
  reorder_start_ = reorder_buffer_;
  data_buffer_ = new uint8_t[max_datagram_size_];
  data_start_ = data_buffer_;
}

TransportUDP::~TransportUDP()
{
  ROS_ASSERT_MSG(sock_ == -1, "TransportUDP socket [%d] was never closed", sock_);
  delete [] reorder_buffer_;
  delete [] data_buffer_;
}

bool TransportUDP::setSocket(int sock)
{
  sock_ = sock;
  return initializeSocket();
}

void TransportUDP::socketUpdate(int events)
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if((events & POLLERR) ||
     (events & POLLHUP) ||
     (events & POLLNVAL))
  {
    ROSCPP_LOG_DEBUG("Socket %d closed with (ERR|HUP|NVAL) events %d", sock_, events);
    close();
  }
  else
  {
    if ((events & POLLIN) && expecting_read_)
    {
      if (read_cb_)
      {
        read_cb_(shared_from_this());
      }
    }

    if ((events & POLLOUT) && expecting_write_)
    {
      if (write_cb_)
      {
        write_cb_(shared_from_this());
      }
    }
  }

}

std::string TransportUDP::getTransportInfo()
{
  return "UDPROS connection to [" + cached_remote_host_ + "]";
}

bool TransportUDP::connect(const std::string& host, int port, int connection_id)
{
  sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  connection_id_ = connection_id;

  if (sock_ == -1)
  {
    ROS_ERROR("socket() failed with error [%s]", strerror(errno));
    return false;
  }

  sockaddr_in sin;
  sin.sin_family = AF_INET;
  if (inet_addr(host.c_str()) == INADDR_NONE)
  {
    struct addrinfo* addr;
    if (getaddrinfo(host.c_str(), NULL, NULL, &addr) != 0)
    {
      close();
      ROS_ERROR("couldn't resolve host [%s]", host.c_str());
      return false;
    }

    bool found = false;
    struct addrinfo* it = addr;
    for (; it; it = it->ai_next)
    {
      if (it->ai_family == AF_INET)
      {
        memcpy(&sin, it->ai_addr, it->ai_addrlen);
        sin.sin_family = it->ai_family;
        sin.sin_port = htons(port);

        found = true;
        break;
      }
    }

    freeaddrinfo(addr);

    if (!found)
    {
      ROS_ERROR("Couldn't find an AF_INET address for [%s]\n", host.c_str());
      return false;
    }

    ROSCPP_LOG_DEBUG("Resolved host [%s] to [%s]", host.c_str(), inet_ntoa(sin.sin_addr));
  }
  else
  {
    sin.sin_addr.s_addr = inet_addr(host.c_str()); // already an IP addr
  }

  sin.sin_port = htons(port);

  if (::connect(sock_, (sockaddr *)&sin, sizeof(sin)))
  {
    ROSCPP_LOG_DEBUG("Connect to udpros host [%s:%d] failed with error [%s]", host.c_str(), port, strerror(errno));
    close();

    return false;
  }

  if (!initializeSocket())
  {
    return false;
  }

  ROSCPP_LOG_DEBUG("Connect succeeded to [%s:%d] on socket [%d]", host.c_str(), port, sock_);

  return true;
}

bool TransportUDP::createIncoming(int port, bool is_server)
{
  is_server_ = is_server;

  sock_ = socket(AF_INET, SOCK_DGRAM, 0);

  if (sock_ <= 0)
  {
    ROS_ERROR("socket() failed with error [%s]", strerror(errno));
    return false;
  }

  server_address_.sin_family = AF_INET;
  server_address_.sin_port = htons(port);
  server_address_.sin_addr.s_addr = INADDR_ANY;
  if (bind(sock_, (sockaddr *)&server_address_, sizeof(server_address_)) < 0)
  {
    ROS_ERROR("bind() failed with error [%s]", strerror(errno));
    return false;
  }

  socklen_t len = sizeof(server_address_);
  getsockname(sock_, (sockaddr *)&server_address_, &len);
  server_port_ = ntohs(server_address_.sin_port);
  ROSCPP_LOG_DEBUG("UDPROS server listening on port [%d]", server_port_);

  if (!initializeSocket())
  {
    return false;
  }

  enableRead();

  return true;
}

bool TransportUDP::initializeSocket()
{
  ROS_ASSERT(sock_ != -1);

  if (!(flags_ & SYNCHRONOUS))
  {
    // make the socket non-blocking
    if(fcntl(sock_, F_SETFL, O_NONBLOCK) == -1)
    {
      ROS_ERROR("fcntl (non-blocking) to socket [%d] failed with error [%s]", sock_, strerror(errno));

      close();
      return false;
    }
  }

  ROS_ASSERT(poll_set_ || (flags_ & SYNCHRONOUS));
  if (poll_set_)
  {
    poll_set_->addSocket(sock_, boost::bind(&TransportUDP::socketUpdate, this, _1), shared_from_this());
  }

  return true;
}

void TransportUDP::close()
{
  Callback disconnect_cb;

  if (!closed_)
  {
    {
      boost::mutex::scoped_lock lock(close_mutex_);

      if (!closed_)
      {
        closed_ = true;

        ROSCPP_LOG_DEBUG("UDP socket [%d] closed", sock_);

        ROS_ASSERT(sock_ != -1);

        if (poll_set_)
        {
          poll_set_->delSocket(sock_);
        }

        if (::close(sock_) < 0)
        {
          ROS_ERROR("Error closing socket [%d]: [%s]", sock_, strerror(errno));
        }

        sock_ = -1;

        disconnect_cb = disconnect_cb_;

        disconnect_cb_ = Callback();
        read_cb_ = Callback();
        write_cb_ = Callback();
      }
    }
  }

  if (disconnect_cb)
  {
    disconnect_cb(shared_from_this());
  }
}

int32_t TransportUDP::read(uint8_t* buffer, uint32_t size)
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);
    if (closed_)
    {
      ROSCPP_LOG_DEBUG("Tried to read on a closed socket [%d]", sock_);
      return -1;
    }
  }

  ROS_ASSERT((int32_t)size > 0);

  uint32_t bytes_read = 0;

  while (bytes_read < size)
  {
    TransportUDPHeader header;

    ssize_t num_bytes = 0;
    bool from_previous = false;
    if (reorder_bytes_)
    {
      if (reorder_start_ != reorder_buffer_)
      {
        from_previous = true;
      }

      num_bytes = std::min(size - bytes_read, reorder_bytes_);
      header = reorder_header_;
      memcpy(buffer + bytes_read, reorder_start_, num_bytes);
      reorder_bytes_ -= num_bytes;
      reorder_start_ += num_bytes;
    }
    else
    {
      if (data_filled_ == 0)
      {
        struct iovec iov[2];
        iov[0].iov_base = &header;
        iov[0].iov_len = sizeof(header);
        iov[1].iov_base = data_buffer_;
        iov[1].iov_len = max_datagram_size_ - sizeof(header);

        // Read a datagram with header
        num_bytes = readv(sock_, iov, 2);

        if (num_bytes < 0)
        {
          if (errno == EAGAIN || errno == EWOULDBLOCK)
          {
            num_bytes = 0;
            break;
          }
          else
          {
            ROSCPP_LOG_DEBUG("readv() failed with error [%s]", strerror(errno));
            close();
            break;
          }
        }
        else if (num_bytes == 0)
        {
          ROSCPP_LOG_DEBUG("Socket [%d] received 0/%d bytes, closing", sock_, size);
          close();
          return -1;
        }
        else if (num_bytes < (ssize_t)sizeof(header))
        {
          ROS_ERROR("Socket [%d] received short header (%d bytes): %s", sock_, int(num_bytes), strerror(errno));
          close();
          return -1;
        }

        num_bytes -= sizeof(header);
        data_filled_ = num_bytes;
        data_start_ = data_buffer_;
      }
      else
      {
        from_previous = true;
      }

      num_bytes = std::min(size - bytes_read, data_filled_);
      // Copy from the data buffer, whether it has data left in it from a previous datagram or
      // was just filled by readv()
      memcpy(buffer + bytes_read, data_start_, num_bytes);
      data_filled_ = std::max((int64_t)0, (int64_t)data_filled_ - (int64_t)size);
      data_start_ += num_bytes;
    }


    if (from_previous)
    {
      bytes_read += num_bytes;
    }
    else
    {
      // Process header
      switch (header.op_)
      {
        case ROS_UDP_DATA0:
          if (current_message_id_)
          {
            ROS_DEBUG("Received new message [%d:%d], while still working on [%d] (block %d of %d)", header.message_id_, header.block_, current_message_id_, last_block_ + 1, total_blocks_);
            reorder_header_ = header;
            reorder_bytes_ = num_bytes;
            memcpy(reorder_buffer_, buffer + bytes_read, num_bytes);
            reorder_start_ = reorder_buffer_;
            current_message_id_ = 0;
            total_blocks_ = 0;
            last_block_ = 0;

            data_filled_ = 0;
            data_start_ = data_buffer_;
            return -1;
          }
          total_blocks_ = header.block_;
          last_block_ = 0;
          current_message_id_ = header.message_id_;
          break;
        case ROS_UDP_DATAN:
          if (header.message_id_ != current_message_id_)
          {
            ROS_DEBUG("Message Id mismatch: %d != %d", header.message_id_, current_message_id_);
            return 0;
          }
          if (header.block_ != last_block_ + 1)
          {
            ROS_DEBUG("Expected block %d, received %d", last_block_ + 1, header.block_);
            return 0;
          }
          last_block_ = header.block_;

          break;
        default:
          ROS_ERROR("Unexpected UDP header OP [%d]", header.op_);
          return -1;
      }

      bytes_read += num_bytes;

      if (last_block_ == (total_blocks_ - 1))
      {
        current_message_id_ = 0;
        break;
      }
    }
  }

  return bytes_read;
}

int32_t TransportUDP::write(uint8_t* buffer, uint32_t size)
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      ROSCPP_LOG_DEBUG("Tried to write on a closed socket [%d]", sock_);
      return -1;
    }
  }

  ROS_ASSERT((int32_t)size > 0);

  const uint32_t max_payload_size = max_datagram_size_ - sizeof(TransportUDPHeader);

  uint32_t bytes_sent = 0;
  uint32_t this_block = 0;
  if (++current_message_id_ == 0)
    ++current_message_id_;
  while (bytes_sent < size)
  {
    TransportUDPHeader header;
    header.connection_id_ = connection_id_;
    header.message_id_ = current_message_id_;
    if (this_block == 0)
    {
      header.op_ = ROS_UDP_DATA0;
      header.block_ = (size + max_payload_size - 1) / max_payload_size;
    }
    else
    {
      header.op_ = ROS_UDP_DATAN;
      header.block_ = this_block;
    }
    ++this_block;
    struct iovec iov[2];
    iov[0].iov_base = &header;
    iov[0].iov_len = sizeof(header);
    iov[1].iov_base = buffer + bytes_sent;
    iov[1].iov_len = std::min(max_payload_size, size - bytes_sent);
    ssize_t num_bytes = writev(sock_, iov, 2);
    //usleep(100);
    if (num_bytes < 0)
    {
      if(errno != EAGAIN)
      {
        ROSCPP_LOG_DEBUG("writev() failed with error [%s]", strerror(errno));
        close();
        break;
      }
      else
      {
        num_bytes = 0;
      }
    }
    else if (num_bytes < ssize_t(sizeof(header)))
    {
      ROSCPP_LOG_DEBUG("Socket [%d] short write (%d bytes), closing", sock_, int(num_bytes));
      close();
      break;
    }
    else
    {
      num_bytes -= sizeof(header);
    }
    bytes_sent += num_bytes;
  }

  return bytes_sent;
}

void TransportUDP::enableRead()
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);
  
    if (closed_)
    {
      return;
    }
  }

  if (!expecting_read_)
  {
    poll_set_->addEvents(sock_, POLLIN);
    expecting_read_ = true;
  }
}

void TransportUDP::disableRead()
{
  ROS_ASSERT(!(flags_ & SYNCHRONOUS));

  {
    boost::mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (expecting_read_)
  {
    poll_set_->delEvents(sock_, POLLIN);
    expecting_read_ = false;
  }
}

void TransportUDP::enableWrite()
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (!expecting_write_)
  {
    poll_set_->addEvents(sock_, POLLOUT);
    expecting_write_ = true;
  }
}

void TransportUDP::disableWrite()
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (expecting_write_)
  {
    poll_set_->delEvents(sock_, POLLOUT);
    expecting_write_ = false;
  }
}

TransportUDPPtr TransportUDP::createOutgoing(std::string host, int port, int connection_id, int max_datagram_size)
{
  ROS_ASSERT(is_server_);
  
  TransportUDPPtr transport(new TransportUDP(poll_set_, flags_, max_datagram_size));
  if (!transport->connect(host, port, connection_id))
  {
    ROS_ERROR("Failed to create outgoing connection");
    return TransportUDPPtr();
  }
  return transport;

}

}
