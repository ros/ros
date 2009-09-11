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

#include "ros/transport/transport_tcp.h"
#include "ros/node.h"

#include <sstream>

#include <boost/bind.hpp>

#include <sys/poll.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <errno.h>

namespace ros
{

TransportTCP::TransportTCP()
: sock_(-1)
, closed_(false)
, expecting_read_(false)
, expecting_write_(false)
, is_server_(false)
, server_port_(-1)
{

}

TransportTCP::~TransportTCP()
{
  ROS_ASSERT_MSG(sock_ == -1, "TransportTCP socket [%d] was never closed", sock_);
}

bool TransportTCP::setSocket(int sock)
{
  sock_ = sock;
  return initializeSocket();
}

bool TransportTCP::initializeSocket()
{
  ROS_ASSERT(sock_ != -1);

  // make the socket non-blocking
  if(fcntl(sock_, F_SETFL, O_NONBLOCK) == -1)
  {
    ROS_ERROR("fcntl (non-blocking) to socket [%d] failed with error [%s]", sock_, strerror(errno));

    close();
    return false;
  }

  if (is_server_)
  {
    cached_remote_host_ = "TCPServer Socket";
  }
  else
  {
    cached_remote_host_ = getClientURI();
  }

  enableRead();

  return true;
}

bool TransportTCP::connect(const std::string& host, int port)
{
  sock_ = socket(AF_INET, SOCK_STREAM, 0);

  if (sock_ == -1)
  {
    ROS_ERROR("socket() failed with error [%s]", strerror(errno));
    return false;
  }

  sockaddr_in sin;
  sin.sin_family = AF_INET;
  sin.sin_port = htons(port);
  if (inet_addr(host.c_str()) == INADDR_NONE)
  {
    hostent *hp = gethostbyname(host.c_str());
    if (!hp)
    {
      close();
      ROS_ERROR("couldn't resolve publisher host [%s]", host.c_str());
      return false;
    }

    sin.sin_addr.s_addr = *((unsigned long *)hp->h_addr);
    ROS_DEBUG("Resolved publisher host [%s] to [%s]",
                host.c_str(), inet_ntoa(sin.sin_addr));
  }
  else
  {
    sin.sin_addr.s_addr = inet_addr(host.c_str()); // already an IP addr
  }

  if (::connect(sock_, (sockaddr *)&sin, sizeof(sin)))
  {
    ROS_DEBUG("Connect to tcpros publisher [%s:%d] failed with error [%s]", host.c_str(), port, strerror(errno));
    close();

    return false;
  }

  if (!initializeSocket())
  {
    return false;
  }

  ROS_DEBUG("Connect succeeded to [%s:%d] on socket [%d]", host.c_str(), port, sock_);

  return true;
}

bool TransportTCP::listen(int port, int backlog, const AcceptCallback& accept_cb)
{
  is_server_ = true;
  accept_cb_ = accept_cb;

  sock_ = socket(AF_INET, SOCK_STREAM, port);

  if (sock_ <= 0)
  {
    ROS_ERROR("socket() failed with error [%s]", strerror(errno));
    return false;
  }

  server_address_.sin_family = AF_INET;
  server_address_.sin_port = 0;
  server_address_.sin_addr.s_addr = INADDR_ANY;
  if (bind(sock_, (sockaddr *)&server_address_, sizeof(server_address_)) < 0)
  {
    ROS_ERROR("bind() failed with error [%s]", strerror(errno));
    return false;
  }

  ::listen(sock_, backlog);
  socklen_t len = sizeof(server_address_);
  getsockname(sock_, (sockaddr *)&server_address_, &len);
  server_port_ = ntohs(server_address_.sin_port);

  if (!initializeSocket())
  {
    return false;
  }

  return true;
}

void TransportTCP::close()
{
  if (!closed_)
  {
    {
      boost::mutex::scoped_lock lock(close_mutex_);

      if (!closed_)
      {
        closed_ = true;

        ROS_DEBUG("TCP socket [%d] closed", sock_);

        ROS_ASSERT(sock_ != -1);

        ::shutdown(sock_, SHUT_RDWR);
        if (::close(sock_) < 0)
        {
          ROS_ERROR("Error closing socket [%d]: [%s]", sock_, strerror(errno));
        }

        disableRead();
        disableWrite();

        closed_ = true;

        sock_ = -1;

        if (disconnect_cb_)
        {
          disconnect_cb_(shared_from_this());
        }

        disconnect_cb_ = Callback();
        read_cb_ = Callback();
        write_cb_ = Callback();
        accept_cb_ = AcceptCallback();
      }
    }
  }
}

int32_t TransportTCP::read(uint8_t* buffer, uint32_t size)
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);
    if (closed_)
    {
      ROS_DEBUG("Tried to read on a closed socket [%d]", sock_);
      return -1;
    }
  }

  ROS_ASSERT((int32_t)size > 0);

  int num_bytes = ::recv(sock_, buffer, size, 0);

  if (num_bytes < 0)
  {
    if (errno != EAGAIN)
    {
      ROS_DEBUG("recv() failed with error [%s]", strerror(errno));
    }
    else
    {
      num_bytes = 0;
    }
  }
  else if (num_bytes == 0)
  {
    ROS_DEBUG("Socket [%d] received 0/%d bytes, closing", sock_, size);
    close();
    return -1;
  }

  return num_bytes;
}

int32_t TransportTCP::write(uint8_t* buffer, uint32_t size)
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      ROS_DEBUG("Tried to write on a closed socket [%d]", sock_);
      return -1;
    }
  }

  ROS_ASSERT((int32_t)size > 0);

  int num_bytes = ::send(sock_, buffer, size, 0);

  if (num_bytes < 0)
  {
    if(errno != EAGAIN)
    {
      ROS_DEBUG("send() failed with error [%s]", strerror(errno));

      close();
    }
    else
    {
      num_bytes = 0;
    }
  }

  return num_bytes;
}

void TransportTCP::enableRead()
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
    g_node->addPollSocket(sock_, POLLIN, shared_from_this());
    expecting_read_ = true;
  }
}

void TransportTCP::disableRead()
{
  /*{
    boost::mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }*/

  if (expecting_read_)
  {
    g_node->delPollSocket(sock_, POLLIN, shared_from_this());
    expecting_read_ = false;
  }
}

void TransportTCP::enableWrite()
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
    g_node->addPollSocket(sock_, POLLOUT, shared_from_this());
    expecting_write_ = true;
  }
}

void TransportTCP::disableWrite()
{
  /*{
    boost::mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }*/

  if (expecting_write_)
  {
    g_node->delPollSocket(sock_, POLLOUT, shared_from_this());
    expecting_write_ = false;
  }
}

void TransportTCP::socketUpdate(int events)
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
    ROS_DEBUG("Socket %d closed with (ERR|HUP|NVAL) events %d", sock_, events);
    close();
  }
  else
  {
    if ((events & POLLIN) && expecting_read_)
    {
      if (is_server_)
      {
        // Should not block here, because poll() said that it's ready
        // for reading
        sockaddr client_address;
        socklen_t len = sizeof(client_address);
        int new_sock = ::accept(sock_, (sockaddr *)&client_address, &len);
        if (new_sock >= 0)
        {
          ROS_DEBUG("Accepted connection on socket [%d]", new_sock);

          TransportTCPPtr transport(new TransportTCP());
          if (transport->setSocket(new_sock))
          {
            accept_cb_(transport);
          }
          else
          {
            ROS_ERROR("Failed to set socket on transport for socket %d", new_sock);
          }
        }
        else
        {
          ROS_ERROR("accept() on socket [%d] failed with error [%s]", sock_, strerror(errno));
        }
      }
      else
      {
        if (read_cb_)
        {
          read_cb_(shared_from_this());
        }
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

std::string TransportTCP::getTransportInfo()
{
  return "TCPROS connection to [" + cached_remote_host_ + "]";
}

std::string TransportTCP::getClientURI()
{
  ROS_ASSERT(!is_server_);

  sockaddr_in addr;
  socklen_t len = sizeof(addr);
  getpeername(sock_, (sockaddr *)&addr, &len);
  int port = ntohs(addr.sin_port);
  std::string ip = inet_ntoa(addr.sin_addr);

  std::stringstream uri;
  uri << ip << ":" << port;

  return uri.str();
}

} // namespace ros
