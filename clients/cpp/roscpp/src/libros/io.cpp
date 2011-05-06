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
** Includes
*****************************************************************************/

#include "../../include/ros/io.h"
#include <ros/assert.h> // don't need if we dont call the pipe functions.
#include <errno.h> // for EFAULT and co.
#include <iostream>
#include <sstream>
#ifdef WIN32
#else
  #include <cstring> // strerror
  #include <fcntl.h> // for non-blocking configuration
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros {

int last_socket_error() {
	#ifdef WIN32
		return WSAGetLastError();
	#else
		return errno;
	#endif
}
const char* last_socket_error_string() {
	#ifdef WIN32
		// could fix this to use FORMAT_MESSAGE and print a real string later,
		// but not high priority.
		std::stringstream ostream;
		ostream << "WSA Error: " << WSAGetLastError();
		return ostream.str().c_str();
	#else
		return strerror(errno);
	#endif
}

bool last_socket_error_is_would_block() {
#if defined(WIN32)
	if ( WSAGetLastError() == WSAEWOULDBLOCK ) {
		return true;
	} else {
		return false;
	}
#else
	if ( ( errno == EAGAIN  ) || ( errno == EWOULDBLOCK ) ) { // posix permits either
		return true;
	} else {
		return false;
	}
#endif
}

/*****************************************************************************
** Service Robotics/Libssh Functions
*****************************************************************************/
/**
 * @brief A cross platform polling function for sockets.
 *
 * Windows doesn't have a polling function until Vista (WSAPoll) and even then
 * its implementation is not supposed to be great. This works for a broader
 * range of platforms and will suffice.
 * @param fds - the socket set (descriptor's and events) to poll for.
 * @param nfds - the number of descriptors to poll for.
 * @param timeout - timeout in milliseconds.
 * @return int : -1 on error, 0 on timeout, +ve number of structures with received events.
 */
int poll_sockets(socket_pollfd *fds, nfds_t nfds, int timeout) {
#if defined(WIN32)
	fd_set readfds, writefds, exceptfds;
	struct timeval tv, *ptv;
	socket_fd_t max_fd;
	int rc;
	nfds_t i;

	if (fds == NULL) {
		errno = EFAULT;
		return -1;
	}

	FD_ZERO (&readfds);
	FD_ZERO (&writefds);
	FD_ZERO (&exceptfds);

	/*********************
	** Compute fd sets
	**********************/
	// also find the largest descriptor.
	for (rc = -1, max_fd = 0, i = 0; i < nfds; i++) {
		if (fds[i].fd == INVALID_SOCKET) {
			continue;
		}
		if (fds[i].events & (POLLIN | POLLRDNORM)) {
			FD_SET (fds[i].fd, &readfds);
		}
		if (fds[i].events & (POLLOUT | POLLWRNORM | POLLWRBAND)) {
			FD_SET (fds[i].fd, &writefds);
		}
		if (fds[i].events & (POLLPRI | POLLRDBAND)) {
			FD_SET (fds[i].fd, &exceptfds);
		}
		if (fds[i].fd > max_fd &&
			  (fds[i].events & (POLLIN | POLLOUT | POLLPRI |
								POLLRDNORM | POLLRDBAND |
								POLLWRNORM | POLLWRBAND))) {
			max_fd = fds[i].fd;
			rc = 0;
		}
	}

	if (rc == -1) {
		errno = EINVAL;
		return -1;
	}
	/*********************
	** Setting the timeout
	**********************/
	if (timeout < 0) {
		ptv = NULL;
	} else {
		ptv = &tv;
		if (timeout == 0) {
			tv.tv_sec = 0;
			tv.tv_usec = 0;
		} else {
			tv.tv_sec = timeout / 1000;
			tv.tv_usec = (timeout % 1000) * 1000;
		}
	}

	rc = select (max_fd + 1, &readfds, &writefds, &exceptfds, ptv);
	if (rc < 0) {
		return -1;
	} else if ( rc == 0 ) {
		return 0;
	}

	for (rc = 0, i = 0; i < nfds; i++) {
		if (fds[i].fd != INVALID_SOCKET) {
			fds[i].revents = 0;

			if (FD_ISSET(fds[i].fd, &readfds)) {
				int save_errno = errno;
				char data[64] = {0};
				int ret;

				/* support for POLLHUP */
				// just check if there's incoming data, without removing it from the queue.
				ret = recv(fds[i].fd, data, 64, MSG_PEEK);
				#ifdef WIN32
				if ((ret == -1) &&
						(errno == WSAESHUTDOWN || errno == WSAECONNRESET ||
						(errno == WSAECONNABORTED) || errno == WSAENETRESET))
				#else
				if ((ret == -1) &&
						(errno == ESHUTDOWN || errno == ECONNRESET ||
						(errno == ECONNABORTED) || errno == ENETRESET))
				#endif
				{
					fds[i].revents |= POLLHUP;
				} else {
					fds[i].revents |= fds[i].events & (POLLIN | POLLRDNORM);
				}
				errno = save_errno;
			}
			if (FD_ISSET(fds[i].fd, &writefds)) {
				fds[i].revents |= fds[i].events & (POLLOUT | POLLWRNORM | POLLWRBAND);
			}

			if (FD_ISSET(fds[i].fd, &exceptfds)) {
				fds[i].revents |= fds[i].events & (POLLPRI | POLLRDBAND);
			}

			if (fds[i].revents & ~POLLHUP) {
				rc++;
			}
		} else {
				fds[i].revents = POLLNVAL;
		}
	}
	return rc;
#else
	// use an existing poll implementation
	int result = poll(fds, nfds, timeout);
	if ( result < 0 ) {
		// EINTR means that we got interrupted by a signal, and is not an error
		if(errno == EINTR) {
			result = 0;
		}
	}
	return result;
#endif // poll_sockets functions
}
/*****************************************************************************
** Socket Utilities
*****************************************************************************/
/**
 * Sets the socket as non blocking.
 * @return int : 0 on success, WSAGetLastError()/errno on failure.
 */
int set_non_blocking(socket_fd_t &socket) {
#ifdef WIN32
    u_long non_blocking = 1;
	if(ioctlsocket( socket, FIONBIO, &non_blocking ) != 0 )
	{
      return WSAGetLastError();
	}
#else
    if(fcntl(socket, F_SETFL, O_NONBLOCK) == -1)
    {
      return errno;
    }
#endif
    return 0;
}

/**
 * @brief Close the socket.
 *
 * @return int : 0 on success, -1 on failure.
 */
int close_socket(socket_fd_t &socket) {
#ifdef WIN32
	if(::closesocket(socket) == SOCKET_ERROR ) {
		return -1;
	} else {
		return 0;
	}
#else
	if (::close(socket) < 0) {
		return -1;
	} else {
		return 0;
	}
#endif //WIN32
}

/*****************************************************************************
** Signal Pair
*****************************************************************************/
/**
 * This code is primarily from the msdn socket tutorials.
 * @param signal_pair : a pair of sockets linked to each other over localhost.
 * @return 0 on success, -1 on failure.
 */
int create_signal_pair(signal_fd_t signal_pair[2]) {
#ifdef WIN32 // use a socket pair
	signal_pair[0] = INVALID_SOCKET;
	signal_pair[1] = INVALID_SOCKET;

    union {
       struct sockaddr_in inaddr;
       struct sockaddr addr;
    } a;
    socklen_t addrlen = sizeof(a.inaddr);

    /*********************
	** Listen Socket
	**********************/
	socket_fd_t listen_socket = INVALID_SOCKET;
    listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (listen_socket == INVALID_SOCKET) {
		return -1;
	}

    // allow it to be bound to an address already in use - do we actually need this?
    int reuse = 1;
    if (setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, (char*) &reuse, (socklen_t) sizeof(reuse)) == SOCKET_ERROR ) {
    	::closesocket(listen_socket);
		return -1;
    }

    memset(&a, 0, sizeof(a));
    a.inaddr.sin_family = AF_INET;
    a.inaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    // For TCP/IP, if the port is specified as zero, the service provider assigns
    // a unique port to the application from the dynamic client port range.
    a.inaddr.sin_port = 0;

    if  (bind(listen_socket, &a.addr, sizeof(a.inaddr)) == SOCKET_ERROR) {
    	::closesocket(listen_socket);
		return -1;
    }
    // we need this below because the system auto filled in some entries, e.g. port #
    if  (getsockname(listen_socket, &a.addr, &addrlen) == SOCKET_ERROR) {
    	::closesocket(listen_socket);
		return -1;
    }
    // max 1 connection permitted
    if (listen(listen_socket, 1) == SOCKET_ERROR) {
    	::closesocket(listen_socket);
		return -1;
    }

    /*********************
	** Connection
	**********************/
    // do we need io overlapping?
    // DWORD flags = (make_overlapped ? WSA_FLAG_OVERLAPPED : 0);
    DWORD overlapped_flag = 0;
    signal_pair[0] = WSASocket(AF_INET, SOCK_STREAM, 0, NULL, 0, overlapped_flag);
    if (signal_pair[0] == INVALID_SOCKET) {
    	::closesocket(listen_socket);
    	::closesocket(signal_pair[0]);
    	return -1;
    }
    // reusing the information from above to connect to the listener
    if (connect(signal_pair[0], &a.addr, sizeof(a.inaddr)) == SOCKET_ERROR) {
    	::closesocket(listen_socket);
    	::closesocket(signal_pair[0]);
    	return -1;
    }
    /*********************
	** Accept
	**********************/
    signal_pair[1] = accept(listen_socket, NULL, NULL);
    if (signal_pair[1] == INVALID_SOCKET) {
    	::closesocket(listen_socket);
    	::closesocket(signal_pair[0]);
    	::closesocket(signal_pair[1]);
    	return -1;
    }
	/*********************
	** Nonblocking
	**********************/
    // should we do this or should we set io overlapping?
    if ( (set_non_blocking(signal_pair[0]) != 0) || (set_non_blocking(signal_pair[1]) != 0)  ) {
		::closesocket(listen_socket);
		::closesocket(signal_pair[0]);
		::closesocket(signal_pair[1]);
    	return -1;
    }
	/*********************
	** Cleanup
	**********************/
    ::closesocket(listen_socket);  // the listener has done its job.
    return 0;
#else // use a pipe pair
	// initialize
	signal_pair[0] = -1;
	signal_pair[1] = -1;

	if(pipe(signal_pair) != 0) {
		ROS_FATAL( "pipe() failed");
		return -1;
	}
	if(fcntl(signal_pair[0], F_SETFL, O_NONBLOCK) == -1) {
		ROS_FATAL( "fcntl() failed");
		return -1;
	}
	if(fcntl(signal_pair[1], F_SETFL, O_NONBLOCK) == -1) {
		ROS_FATAL( "fcntl() failed");
		return -1;
	}
	return 0;
#endif // create_pipe
}

} // namespace ros
