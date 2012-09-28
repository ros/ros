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
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef ROSCPP_IO_H_
#define ROSCPP_IO_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include "common.h"

#ifdef WIN32
	#include <winsock2.h> // For struct timeval
	#include <ws2tcpip.h> // Must be after winsock2.h because MS didn't put proper inclusion guards in their headers.
	#include <sys/types.h>
	#include <io.h>
	#include <fcntl.h>
	#include <process.h> // for _getpid
#else
	#include <poll.h> // should get cmake to explicitly check for poll.h?
	#include <sys/poll.h>
	#include <arpa/inet.h>
	#include <netdb.h>
    #include <unistd.h>
   	#include <pthread.h>
    #include <netdb.h>       // getnameinfo in network.cpp
    #include <netinet/in.h>  // sockaddr_in in network.cpp
	#include <netinet/tcp.h> // TCP_NODELAY in transport/transport_tcp.cpp
#endif

/*****************************************************************************
** Cross Platform Macros
*****************************************************************************/

#ifdef WIN32
    #define getpid _getpid
	#define ROS_INVALID_SOCKET INVALID_SOCKET
	#define ROS_SOCKETS_SHUT_RDWR SD_BOTH /* Used by ::shutdown() */
	#define ROS_SOCKETS_ASYNCHRONOUS_CONNECT_RETURN WSAEWOULDBLOCK
	#ifndef POLLRDNORM
		#define POLLRDNORM  0x0100 /* mapped to read fds_set */
	#endif
	#ifndef POLLRDBAND
		#define POLLRDBAND  0x0200 /* mapped to exception fds_set */
	#endif
	#ifndef POLLIN
		#define POLLIN      (POLLRDNORM | POLLRDBAND) /* There is data to read.  */
	#endif
	#ifndef POLLPRI
		#define POLLPRI     0x0400 /* There is urgent data to read.  */
	#endif

	#ifndef POLLWRNORM
		#define POLLWRNORM  0x0010 /* mapped to write fds_set */
	#endif
	#ifndef POLLOUT
		#define POLLOUT     (POLLWRNORM) /* Writing now will not block.  */
	#endif
	#ifndef POLLWRBAND
		#define POLLWRBAND  0x0020 /* mapped to write fds_set */
	#endif
	#ifndef POLLERR
		#define POLLERR     0x0001 /* Error condition.  */
	#endif
	#ifndef POLLHUP
		#define POLLHUP     0x0002 /* Hung up.  */
	#endif
	#ifndef POLLNVAL
		#define POLLNVAL    0x0004 /* Invalid polling request.  */
	#endif
#else
	#define ROS_SOCKETS_SHUT_RDWR SHUT_RDWR /* Used by ::shutdown() */
	#define ROS_INVALID_SOCKET ((int) -1)
	#define ROS_SOCKETS_ASYNCHRONOUS_CONNECT_RETURN EINPROGRESS
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros {

/*****************************************************************************
** Cross Platform Types
*****************************************************************************/

#ifdef WIN32
  typedef SOCKET socket_fd_t;
  typedef SOCKET signal_fd_t;
  /* poll emulation support */
  typedef struct socket_pollfd {
    socket_fd_t fd;      /* file descriptor */
    short events;     /* requested events */
    short revents;    /* returned events */
  } socket_pollfd;

  typedef unsigned long int nfds_t;
  #ifdef _MSC_VER
    typedef int pid_t; /* return type for windows' _getpid */
  #endif
#else
  typedef int socket_fd_t;
  typedef int signal_fd_t;
  typedef struct pollfd socket_pollfd;
#endif

/*****************************************************************************
** Functions
*****************************************************************************/

ROSCPP_DECL int last_socket_error();
ROSCPP_DECL const char* last_socket_error_string();
ROSCPP_DECL bool last_socket_error_is_would_block();
ROSCPP_DECL int poll_sockets(socket_pollfd *fds, nfds_t nfds, int timeout);
ROSCPP_DECL int set_non_blocking(socket_fd_t &socket);
ROSCPP_DECL int close_socket(socket_fd_t &socket);
ROSCPP_DECL int create_signal_pair(signal_fd_t signal_pair[2]);

/*****************************************************************************
** Inlines - almost direct api replacements, should stay fast.
*****************************************************************************/

/**
 * Closes the signal pair - on windows we're using sockets (because windows
 * select() function cant handle pipes). On linux, we're just using the pipes.
 * @param signal_pair : the signal pair type.
 */
inline void close_signal_pair(signal_fd_t signal_pair[2]) {
#ifdef WIN32 // use a socket pair
	::closesocket(signal_pair[0]);
	::closesocket(signal_pair[1]);
#else // use a socket pair on mingw or pipe pair on linux, either way, close works
	::close(signal_pair[0]);
	::close(signal_pair[1]);
#endif
}

/**
 * Write to a signal_fd_t device. On windows we're using sockets (because windows
 * select() function cant handle pipes) so we have to use socket functions.
 * On linux, we're just using the pipes.
 */
#ifdef _MSC_VER
	inline int write_signal(const signal_fd_t &signal, const char *buffer, const unsigned int &nbyte) {
		return ::send(signal, buffer, nbyte, 0);
//		return write(signal, buffer, nbyte);
	}
#else
	inline ssize_t write_signal(const signal_fd_t &signal, const void *buffer, const size_t &nbyte) {
		return write(signal, buffer, nbyte);
	}
#endif


/**
 * Read from a signal_fd_t device. On windows we're using sockets (because windows
 * select() function cant handle pipes) so we have to use socket functions.
 * On linux, we're just using the pipes.
 */
#ifdef _MSC_VER
	inline int read_signal(const signal_fd_t &signal, char *buffer, const unsigned int &nbyte) {
		return ::recv(signal, buffer, nbyte, 0);
//		return _read(signal, buffer, nbyte);
	}
#else
	inline ssize_t read_signal(const signal_fd_t &signal, void *buffer, const size_t &nbyte) {
		return read(signal, buffer, nbyte);
	}
#endif

} // namespace ros

#endif /* ROSCPP_IO_H_ */

