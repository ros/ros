# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: transport.py 2941 2008-11-26 03:19:48Z sfkwc $

import cStringIO
import select
import socket
import struct
import logging
import thread
import threading
import traceback

from roslib.message import DeserializationError, Message
from roslib.network import read_ros_handshake_header, write_ros_handshake_header

# TODO: remove * import from core
from rospy.core import *
#from rospy.core import logwarn, loginfo, logerr, logdebug
from rospy.exceptions import ROSInternalException, TransportException, TransportTerminated, TransportInitError
from rospy.msg import deserialize_messages, serialize_message
from rospy.transport import Transport, BIDIRECTIONAL

logger = logging.getLogger('rospy.tcpros')

# Receive buffer size for topics/services (in bytes)
DEFAULT_BUFF_SIZE = 65536

## name of our customized TCP protocol for accepting flows over server socket
TCPROS = "TCPROS" 

## read data from \a sock into buffer \a b.
## @param sock: socket to read from
## @param b StringIO: buffer to receive into
## @param buff_size int: recv read size
## @return int number of bytes read
def recv_buff(sock, b, buff_size):
    d = sock.recv(buff_size)
    if d:
        b.write(d)
        return len(d)
    else: #bomb out
        raise TransportTerminated("unable to receive data from sender, check sender's logs for details")

## Simple server that accepts inbound TCP/IP connections and hands
## them off to a handler function. TCPServer obeys the
## ROS_IP/ROS_HOSTNAME environment variables
class TCPServer:

    ## Setup a server socket listening on the specified port. If the
    ## port is omitted, will choose any open port.
    ## @param self
    ## @param inbound_handler fn(sock, addr): handler to invoke with
    ## new connection
    ## @param port int: port to bind to, omit/0 to bind to any
    def __init__(self, inbound_handler, port=0): 
        self.port = port #will get overwritten if port=0
        self.addr = None #set at socket bind
        self.is_shutdown = False
        self.inbound_handler = inbound_handler
        try:
            self.server_sock = self._create_server_sock()
        except:
            self.server_sock = None
            raise

    ## Runs the run() loop in a separate thread
    def start(self):
        thread.start_new_thread(self.run, ())
        
    ## Main TCP receive loop. Should be run in a separate thread -- use start()
    ## to do this automatically.
    def run(self):
        self.is_shutdown = False
        if not self.server_sock:
            raise ROSInternalException("%s did not connect"%self.__class__.__name__)
        while not self.is_shutdown:
            try:
                (client_sock, client_addr) = self.server_sock.accept()
                #leave threading decisions up to inbound_handler
                self.inbound_handler(client_sock, client_addr)
            except socket.error, e:
                if not self.is_shutdown:
                    logwarn("TCPServer: socket error: %s"%e)
        logdebug("TCPServer[%s] shutting down", self.port)

    ## @param self
    ## @return (str, int): (ip address, port) of server socket binding
    def get_full_addr(self):
        # return roslib.network.get_host_name() instead of address so that it
        # obeys ROS_IP/ROS_HOSTNAME behavior
        return (roslib.network.get_host_name(), self.port)
    
    ## binds the server socket. ROS_IP/ROS_HOSTNAME may restrict
    ## binding to loopback interface.
    ## @param self
    def _create_server_sock(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        server_sock.bind((roslib.network.get_bind_address(), self.port))
        (self.addr, self.port) = server_sock.getsockname()
        server_sock.listen(5)
        return server_sock

    ## shutdown I/O resources uses by this server
    ## @param self
    def shutdown(self):
        if not self.is_shutdown:
            self.is_shutdown = True
            #self.server_sock.shutdown(socket.SHUT_RDWR)
            self.server_sock.close()

## TCPROS Server

# base maintains a tcpros_server singleton that is shared between
# services and topics for inbound connections. This global is set in
# the tcprosserver constructor. Constructor is called by init_tcpros()
_tcpros_server = None

## starts the TCPROS server socket for inbound connections
def init_tcpros_server():
    global _tcpros_server
    if _tcpros_server is None:
        _tcpros_server = TCPROSServer()
    return _tcpros_server
    
## start the TCPROS server if it has not started already
## @throws Exception if tcpros server has not been created
def start_tcpros_server():
    if _tcpros_server is None:
        init_tcpros_server()
    return _tcpros_server.start_server()

# provide an accessor of this so that the TCPROS Server is entirely hidden from upper layers

## get the address of the tcpros server.
## @throws Exception if tcpros server has not been started or created
def get_tcpros_server_address():
    return _tcpros_server.get_address()

## utility handler that does nothing more than provide a rejection header
## @param sock: socket connection
## @param client_addr client address
## @param header dict: request header
def _error_connection_handler(sock, client_addr, header):
    return {'error': "unhandled connection"}

## ROS Protocol handler for TCPROS. Accepts both TCPROS topic
## connections as well as ROS service connections over TCP. TCP server
## socket is run once start_server() is called -- this is implicitly
## called during init_publisher().
class TCPROSServer(object):
    ## @param self
    def __init__(self):
        self.tcp_ros_server = None #: server for receiving tcp conn
        self.lock = threading.Lock()
        ## should be set to fn(sock, client_addr, header) for topic connections
        self.topic_connection_handler = _error_connection_handler
        ## should be set to fn(sock, client_addr, header) for service connections        
        self.service_connection_handler = _error_connection_handler
        
    ## Starts the TCP socket server if one is not already running
    ## @param self
    def start_server(self):
        if self.tcp_ros_server:
            return
        try:
            self.lock.acquire()
            try:
                if not self.tcp_ros_server:
                    self.tcp_ros_server = TCPServer(self._tcp_server_callback, 0) #bind to any port
                    self.tcp_ros_server.start()
            except Exception, e:
                self.tcp_ros_server = None
                logerr("unable to start TCPROS server: %s\n%s"%(e, traceback.format_exc()))
                return 0, "unable to establish TCPROS server: %s"%e, []
        finally:
            self.lock.release()

    ## @param self
    ## @return str, int: address and port of TCP server socket for
    ## accepting inbound connections
    def get_address(self):
        if self.tcp_ros_server is not None:
            return self.tcp_ros_server.get_full_addr()
        return None, None
    
    ## stops the TCP/IP server responsible for receiving inbound connections
    ## @param self
    def shutdown(self):
        if self.tcp_ros_server:
            self.tcp_ros_server.shutdown()

    ## TCPServer callback: detects incoming topic or service connection and passes connection accordingly
    ## @param self
    ## @param sock socket: socket connection
    ## @param client_addr (str, int): client address
    ## @throws TransportInitError If transport cannot be succesfully initialized
    def _tcp_server_callback(self, sock, client_addr):
        #TODOXXX:rewrite this logic so it is possible to create TCPROSTransport object first, set its protocol,
        #and then use that to do the writing
        try:
            buff_size = 4096 # size of read buffer
            header = read_ros_handshake_header(sock, cStringIO.StringIO(), buff_size)
            if 'topic' in header:
                err_msg = self.topic_connection_handler(sock, client_addr, header)
            elif 'service' in header:
                err_msg = self.service_connection_handler(sock, client_addr, header)
            else:
                err_msg = 'no topic or service name detected'
            if err_msg:
                write_ros_handshake_header(sock, {'error' : err_msg})
                raise TransportInitError(err_msg)
        except rospy.exceptions.TransportInitError, e:
            logwarn("Inbound TCP/IP connected failed: %s", str(e))
        except Exception, e:
            logerr("Inbound TCP/IP connected failed: %s\n%s", e, traceback.format_exc(e))
            if sock is not None:
                sock.close()

## Abstraction of TCPROS connections. Implementations Services/Publishers/Subscribers must implement this
## protocol, which defines how messages are deserialized from an inbound connection (read_messages()) as
## well as which fields to send when creating a new connection (get_header_fields()).                
class TCPROSTransportProtocol(object):

    ## ctor
    ## @param self
    ## @param name str: service or topic name
    ## @param recv_data_class Class: message class for deserializing inbound messages
    ## @param queue_size int: maximum number of inbound messages to maintain
    ## @param buff_size int: recieve buffer size (in bytes) for reading data from the inbound connection.
    def __init__(self, name, recv_data_class, queue_size=None, buff_size=DEFAULT_BUFF_SIZE):
        if recv_data_class and not issubclass(recv_data_class, Message):
            raise TransportInitError("Unable to initialize transport: data class is not a message data class")
        self.name = name
        self.recv_data_class = recv_data_class
        self.queue_size = queue_size
        self.buff_size = buff_size
        self.direction = BIDIRECTIONAL
        
    ## @param self
    ## @param b StringIO: read buffer        
    ## @param msg_queue [Message]: queue of deserialized messages
    ## @param sock socket: protocol can optionally read more data from
    ## the socket, but in most cases the required data will already be
    ## in \a b
    def read_messages(self, b, msg_queue, sock):
        # default implementation
        deserialize_messages(b, msg_queue, self.recv_data_class, queue_size=self.queue_size)
        
    ## @param self
    ## @return dict {str : str}: header fields to send when connecting to server
    def get_header_fields(self):
        return {}

# TODO: this still isn't as clean and seamless as I want it to
# be. This code came from the merger of publisher, subscriber, and
# service code into a common TCPROS transport class. The transport is
# customized by a 'protocol' class, which is how the different
# pub/sub/service behaviors are achieved. More behavior needs to be
# transferred from the transport class into the protocol class,
# e.g. deserialization as the state each maintains is somewhat
# duplicative. I would also come up with a better name than
# protocol.

## Generic implementation of TCPROS exchange routines for both topics and services
class TCPROSTransport(Transport):
    transport_type = 'TCPROS'
    
    ## ctor
    ## @param self
    ## @param name str: topic or service name    
    ## @param protocol TCPROSTransportProtocol protocol implementation    
    ## @param header dict: (optional) handshake header if transport handshake header was
    ## already read off of transport.
    ## @throws TransportInitError if transport cannot be initialized according to arguments
    def __init__(self, protocol, name, header=None):
        super(TCPROSTransport, self).__init__(protocol.direction, name=name)
        if not name:
            raise TransportInitError("Unable to initialize transport: name is not set")

        self.protocol = protocol

        self.socket = None
        self.endpoint_id = 'unknown'
        self.read_buff = cStringIO.StringIO()
        self.write_buff = cStringIO.StringIO()
            
        self.header = header
        
        # these fields are actually set by the remote
        # publisher/service. they are set for tools that connect
        # without knowing the actual field name
        self.md5sum = None
        self.type = None 
            
    ## Set the socket for this transport
    ## @param self
    ## @param sock: socket
    ## @param endpoint_id str: identifier for connection endpoint
    ## @throws TransportInitError if socket has already been set
    def set_socket(self, sock, endpoint_id):
        if self.socket is not None:
            raise TransportInitError("socket already initialized")
        self.socket = sock
        self.endpoint_id = endpoint_id

    ## Establish TCP connection to the specified
    ## address/port. connect() always calls write_header() and
    ## read_header() after the connection is made
    ## @param self
    ## @param dest_addr str
    ## @param dest_port int        
    ## @param endpoint_id str: string identifier for connection (for statistics)
    ## @param timeout float: (optional keyword) timeout in seconds        
    def connect(self, dest_addr, dest_port, endpoint_id, timeout=None):
        try:
            self.endpoint_id = endpoint_id
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if timeout is not None:
                self.socket.settimeout(timeout)
            self.socket.connect((dest_addr, dest_port))
            self.write_header()
            self.read_header()
        except TransportInitError, tie:
            logerr("Unable to initiate TCP/IP socket to %s:%s (%s): %s"%(dest_addr, dest_port, endpoint_id, traceback.format_exc()))
            raise
        except Exception, e:
            self.done = True
            if self.socket:
                self.socket.close()
            self.socket = None
            
            logerr("Unknown error initiating TCP/IP socket to %s:%s (%s): %s"%(dest_addr, dest_port, endpoint_id, traceback.format_exc()))
            raise TransportInitError(str(e)) #re-raise i/o error
                
    ## Validate header and initialize fields accordingly
    ## @param self
    ## @param header dict: header fields from publisher
    ## @throws TransportInitError if header fails to validate
    def _validate_header(self, header):
        self.header = header
        if 'error' in header:
            raise TransportInitError("remote error reported: %s"%header['error'])
        for required in ['md5sum', 'type']:
            if not required in header:
                raise TransportInitError("header missing required field [%s]"%required)
        self.md5sum = header['md5sum']
        self.type = header['type']

    ## Writes the TCPROS header to the active connection.
    ## @param self
    def write_header(self):
        sock = self.socket
        # socket may still be getting spun up, so wait for it to be writable
        fileno = sock.fileno()
        ready = None
        while not ready:
            _, ready, _ = select.select([], [fileno], [])
        logger.debug("[%s]: writing header", self.name)
        sock.setblocking(1)
        self.stat_bytes += write_ros_handshake_header(sock, self.protocol.get_header_fields())

    ## Read TCPROS header from socket
    ## @param self
    ## @throws TransportInitError if header fails to validate
    def read_header(self):
        self.socket.setblocking(1)
        self._validate_header(read_ros_handshake_header(self.socket, self.read_buff, self.protocol.buff_size))
                
    ## Convenience routine for services to send a message across a
    ## particular connection. NOTE: write_data is much more efficient
    ## if same message is being sent to multiple connections. Not
    ## threadsafe.
    ## @param self
    ## @param msg Msg: message to send
    ## @param seq int: sequence number for message
    ## @throws TransportException if error occurred sending message 
    def send_message(self, msg, seq):
        # this will call write_data(), so no need to keep track of stats
        serialize_message(self.write_buff, seq, msg)
        self.write_data(self.write_buff.getvalue())
        self.write_buff.truncate(0)

    ## Write raw data to transport
    ## @throws TransportInitialiationError could not be initialized
    ## @throws TransportTerminated no longer open for publishing
    def write_data(self, data):
        if not self.socket:
            raise TransportInitError("Cannot publish: TCPROS transport was not successfully initialized")
        if self.done:
            raise TransportTerminated("connection closed")
        try:
            #TODO: get rid of sendalls and replace with async-style publishing
            self.socket.sendall(data)
            self.stat_bytes  += len(data)
            self.stat_num_msg += 1
        except IOError, (errno, msg):
            #for now, just document common errno's in code
            if errno == 32: #broken pipe
                logdebug("ERROR: Broken Pipe")
                self.close()
                raise TransportTerminated(str(errno)+msg)
            raise #re-raise
        except socket.error, (errno, msg):
            #for now, just document common errno's in code
            if errno == 32: #broken pipe
                logdebug("ERROR: Broken Pipe")
                self.close()
                raise TransportTerminated(msg)
            elif errno == 104: #connection reset by peer
                logdebug("ERROR: connection reset by peer") 
                self.close()
                raise TransportTerminated(msg)
            else:
                logdebug("ERROR: %s"%msg) 
                self.close()
                raise TransportTerminated(str(errno)+' '+msg)                
        except:
            #TODO: try to figure out common errors here
            raise
        return True
    
    ## block until messages are read off of socket
    ## @return [Msg] : list of newly received messages
    ## @throws TransportException     
    def receive_once(self):
        sock = self.socket
        if sock is None:
            raise TransportException("connection not initialized")
        b = self.read_buff
        msg_queue = []
        p = self.protocol
        try:
            sock.setblocking(1)                
            while not msg_queue and not self.done and not is_shutdown():
                if b.tell() >= 4:
                    p.read_messages(b, msg_queue, sock) 
                if not msg_queue:
                    recv_buff(sock, b, p.buff_size)
            self.stat_num_msg += len(msg_queue) #STATS
            # set the _connection_header field
            for m in msg_queue:
                m._connection_header = self.header
            return msg_queue

        except DeserializationError, e:
            raise TransportException("receive_once[%s]: DeserializationError %s"%(self.name, traceback.format_exc()))
        except TransportTerminated, e:
            raise #reraise
        except:
            raise TransportException("receive_once[%s]: EXCEPTION %s"%(self.name, traceback.format_exc()))
        return retval
        
    ## Receive messages until shutdown
    ## @param self
    ## @param msgs_callback fn([msg]): callback to invoke for new messages received    
    def receive_loop(self, msgs_callback):
        # - use assert here as this would be an internal error, aka bug
        logger.debug("receive_loop for [%s]", self.name)
        try:
            try:
                while not self.done and not is_shutdown():
                    msgs = self.receive_once()
                    if not self.done and not is_shutdown():
                        msgs_callback(msgs)

                logdebug("receive_loop[%s]: done condition met, exited loop"%self.name)
            except DeserializationError, e:
                logerr("[%s] error deserializing incoming request: %s"%self.name, traceback.format_exc()) 
            except:
                # in many cases this will be a normal hangup, but log anyways
                try:
                    #1467 sometimes we get exceptions due to
                    #interpreter shutdow, so blanket ignore those if
                    #the reporting fails
                    logdebug("exception in receive loop for [%s], may be normal. Exception is %s",self.name, traceback.format_exc())
                except: pass
        finally:
            if not self.done:
                self.close()
                self.done = True

    ## close i/o and release resources
    def close(self):
        super(TCPROSTransport, self).close()
        self.done = True
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        self.read_buff = self.write_buff = self.protocol = None

