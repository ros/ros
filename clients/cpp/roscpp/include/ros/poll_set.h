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

#ifndef ROSCPP_POLL_SET_H
#define ROSCPP_POLL_SET_H

#include <vector>
#include "io.h"
#include "common.h"
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

namespace ros
{

class Transport;
typedef boost::shared_ptr<Transport> TransportPtr;

/**
 * \brief Manages a set of sockets being polled through the poll() function call.
 *
 * PollSet provides thread-safe ways of adding and deleting sockets, as well as adding
 * and deleting events.
 */
class ROSCPP_DECL PollSet
{
public:
  PollSet();
  ~PollSet();

  typedef boost::function<void(int)> SocketUpdateFunc;
  /**
   * \brief Add a socket.
   *
   * addSocket() may be called from any thread.
   * \param sock The socket to add
   * \param update_func The function to call when a socket has events
   * \param transport The (optional) transport associated with this socket. Mainly
   * used to prevent the transport from being deleted while we're calling the update function
   */
  bool addSocket(int sock, const SocketUpdateFunc& update_func, const TransportPtr& transport = TransportPtr());
  /**
   * \brief Delete a socket
   *
   * delSocket() may be called from any thread.
   * \param sock The socket to delete
   */
  bool delSocket(int sock);

  /**
   * \brief Add events to be polled on a socket
   *
   * addEvents() may be called from any thread.
   * \param sock The socket to add events to
   * \param events The events to add
   */
  bool addEvents(int sock, int events);
  /**
   * \brief Delete events to be polled on a socket
   *
   * delEvents() may be called from any thread.
   * \param sock The socket to delete events from
   * \param events The events to delete
   */
  bool delEvents(int sock, int events);

  /**
   * \brief Process all socket events
   *
   * This function will actually call poll() on the available sockets, and allow
   * them to do their processing.
   *
   * update() may only be called from one thread at a time
   *
   * \param poll_timeout The time, in milliseconds, for the poll() call to timeout after
   * if there are no events.  Note that this does not provide an upper bound for the entire
   * function, just the call to poll()
   */
  void update(int poll_timeout);

  /**
   * \brief Signal our poll() call to finish if it's blocked waiting (see the poll_timeout
   * option for update()).
   */
  void signal();

private:
  /**
   * \brief Creates the native pollset for our sockets, if any have changed
   */
  void createNativePollset();

  /**
   * \brief Called when events have been triggered on our signal pipe
   */
  void onLocalPipeEvents(int events);

  struct SocketInfo
  {
    TransportPtr transport_;
    SocketUpdateFunc func_;
    int fd_;
    int events_;
  };
  typedef std::map<int, SocketInfo> M_SocketInfo;
  M_SocketInfo socket_info_;
  boost::mutex socket_info_mutex_;
  bool sockets_changed_;

  boost::mutex just_deleted_mutex_;
  typedef std::vector<int> V_int;
  V_int just_deleted_;

  std::vector<socket_pollfd> ufds_;

  boost::mutex signal_mutex_;
  signal_fd_t signal_pipe_[2];
};

}

#endif // ROSCPP_POLL_SET_H
