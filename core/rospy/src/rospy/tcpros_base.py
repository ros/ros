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
# Revision $Id$

"""Internal use: common TCPROS libraries"""

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
from rospy.core import logwarn, loginfo, logerr, logdebug, rospydebug, rospyerr, rospywarn
from rospy.exceptions import ROSInternalException, TransportException, TransportTerminated, TransportInitError
from rospy.msg import deserialize_messages, serialize_message
from rospy.transport import Transport, BIDIRECTIONAL
from rospy.service import ServiceException

logger = logging.getLogger('rospy.tcpros')

# Receive buffer size for topics/services (in bytes)
DEFAULT_BUFF_SIZE = 65536

## name of our customized TCP protocol for accepting flows over server socket
TCPROS = "TCPROS" 

def recv_buff(sock, b, buff_size):
    """
    Read data from socket into buffer.
    @param sock: socket to read from
    @type  sock: socket.socket
    @param b: buffer to receive into
    @type  b: StringIO
    @param buff_size: recv read size
    @type  buff_size: int
    @return: number of bytes read
    @rtype: int
    """
    d = sock.recv(buff_size)
    if d:
        b.write(d)
        return len(d)
    else: #bomb out
        raise TransportTerminated("unable to receive data from sender, check sender's logs for details")

class TCPServer:
    """
    Simple server that accepts inbound TCP/IP connections and hands
    them off to a handler function. TCPServer obeys the
    ROS_IP/ROS_HOSTNAME environment variables
    """

    def __init__(self, inbound_handler, port=0):
        """
        Setup a server socket listening on the specified port. If the
        port is omitted, will choose any open port.
        @param inbound_handler: handler to invoke with
        new connection
        @type  inbound_handler: fn(sock, addr)
        @param port: port to bind to, omit/0 to bind to any
        @type  port: int
        """
        self.port = port #will get overwritten if port=0
        self.addr = None #set at socket bind
        self.is_shutdown = False
        self.inbound_handler = inbound_handler
        try:
            self.server_sock = self._create_server_sock()
        except:
            self.server_sock = None
            raise

    def start(self):
        """Runs the run() loop in a separate thread"""
        thread.start_new_thread(self.run, ())
        
    def run(self):
        """
        Main TCP receive loop. Should be run in a separate thread -- use start()
        to do this automatically.
        """
        self.is_shutdown = False
        if not self.server_sock:
            raise ROSInternalException("%s did not connect"%self.__class__.__name__)
        while not self.is_shutdown:
            try:
                (client_sock, client_addr) = self.server_sock.accept()
            except socket.timeout:
                continue
                
            try:
                #leave threading decisions up to inbound_handler
                self.inbound_handler(client_sock, client_addr)
            except socket.error, e:
                if not self.is_shutdown:
                    traceback.print_exc()
                    logwarn("Failed to handle inbound connection due to socket error: %s"%e)
        logdebug("TCPServer[%s] shutting down", self.port)

    
    def get_full_addr(self):
        """
        @return: (ip address, port) of server socket binding
        @rtype: (str, int)
        """
        # return roslib.network.get_host_name() instead of address so that it
        # obeys ROS_IP/ROS_HOSTNAME behavior
        return (roslib.network.get_host_name(), self.port)
    
    def _create_server_sock(self):
        """
        binds the server socket. ROS_IP/ROS_HOSTNAME may restrict
        binding to loopback interface.
        """
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        server_sock.bind((roslib.network.get_bind_address(), self.port))
        (self.addr, self.port) = server_sock.getsockname()
        server_sock.listen(5)
        return server_sock

    def shutdown(self):
        """shutdown I/O resources uses by this server"""
        if not self.is_shutdown:
            self.is_shutdown = True
            #self.server_sock.shutdown(socket.SHUT_RDWR)
            self.server_sock.close()

# base maintains a tcpros_server singleton that is shared between
# services and topics for inbound connections. This global is set in
# the tcprosserver constructor. Constructor is called by init_tcpros()
_tcpros_server = None

def init_tcpros_server():
    """starts the TCPROS server socket for inbound connections"""
    global _tcpros_server
    if _tcpros_server is None:
        _tcpros_server = TCPROSServer()
        rospy.core.add_shutdown_hook(_tcpros_server.shutdown)
    return _tcpros_server
    
def start_tcpros_server():
    """
    start the TCPROS server if it has not started already
    """
    if _tcpros_server is None:
        init_tcpros_server()
    return _tcpros_server.start_server()

# provide an accessor of this so that the TCPROS Server is entirely hidden from upper layers

def get_tcpros_server_address():
    """
    get the address of the tcpros server.
    @raise Exception: if tcpros server has not been started or created
    """
    return _tcpros_server.get_address()

def _error_connection_handler(sock, client_addr, header):
    """
    utility handler that does nothing more than provide a rejection header
    @param sock: socket connection
    @type  sock: socket.socket
    @param client_addr: client address
    @type  client_addr: str
    @param header: request header
    @type  header: dict
    """
    return {'error': "unhandled connection"}

class TCPROSServer(object):
    """
    ROS Protocol handler for TCPROS. Accepts both TCPROS topic
    connections as well as ROS service connections over TCP. TCP server
    socket is run once start_server() is called -- this is implicitly
    called during init_publisher().
    """
    
    def __init__(self):
        """ctor."""
        self.tcp_ros_server = None #: server for receiving tcp conn
        self.lock = threading.Lock()
        # should be set to fn(sock, client_addr, header) for topic connections
        self.topic_connection_handler = _error_connection_handler
        # should be set to fn(sock, client_addr, header) for service connections        
        self.service_connection_handler = _error_connection_handler
        
    def start_server(self):
        """Starts the TCP socket server if one is not already running"""
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

    
    def get_address(self):
        """
        @return: address and port of TCP server socket for accepting
        inbound connections
        @rtype: str, int
        """
        if self.tcp_ros_server is not None:
            return self.tcp_ros_server.get_full_addr()
        return None, None
    
    def shutdown(self, reason=''):
        """stops the TCP/IP server responsible for receiving inbound connections"""
        if self.tcp_ros_server:
            self.tcp_ros_server.shutdown()

    def _tcp_server_callback(self, sock, client_addr):
        """
        TCPServer callback: detects incoming topic or service connection and passes connection accordingly
    
        @param sock: socket connection
        @type  sock: socket.socket
        @param client_addr: client address
        @type  client_addr: (str, int)
        @raise TransportInitError: If transport cannot be succesfully initialized
        """
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
                # shutdown race condition: nodes that come up and down quickly can receive connections during teardown
                if not rospy.core.is_shutdown():
                    write_ros_handshake_header(sock, {'error' : err_msg})
                    raise TransportInitError("Could not process inbound connection: "+err_msg+str(header))
                else:
                    write_ros_handshake_header(sock, {'error' : 'node shutting down'})
                    return
        except rospy.exceptions.TransportInitError, e:
            logwarn(str(e))
            if sock is not None:
                sock.close()
        except Exception, e:
            # collect stack trace separately in local log file
            logwarn("Inbound TCP/IP connection failed: %s", e)
            rospyerr("Inbound TCP/IP connection failed:\n%s", traceback.format_exc(e))
            if sock is not None:
                sock.close()

class TCPROSTransportProtocol(object):
    """
    Abstraction of TCPROS connections. Implementations Services/Publishers/Subscribers must implement this
    protocol, which defines how messages are deserialized from an inbound connection (read_messages()) as
    well as which fields to send when creating a new connection (get_header_fields()).
    """

    def __init__(self, resolved_name, recv_data_class, queue_size=None, buff_size=DEFAULT_BUFF_SIZE):
        """
        ctor
        @param resolved_name: resolved service or topic name
        @type  resolved_name: str
        @param recv_data_class: message class for deserializing inbound messages
        @type  recv_data_class: Class
        @param queue_size: maximum number of inbound messages to maintain
        @type  queue_size: int
        @param buff_size: receive buffer size (in bytes) for reading from the connection.
        @type  buff_size: int
        """
        if recv_data_class and not issubclass(recv_data_class, Message):
            raise TransportInitError("Unable to initialize transport: data class is not a message data class")
        self.resolved_name = resolved_name
        self.recv_data_class = recv_data_class
        self.queue_size = queue_size
        self.buff_size = buff_size
        self.direction = BIDIRECTIONAL
        
    
    def read_messages(self, b, msg_queue, sock):
        """
        @param b StringIO: read buffer        
        @param msg_queue [Message]: queue of deserialized messages
        @type  msg_queue: [Message]
        @param sock socket: protocol can optionally read more data from
        the socket, but in most cases the required data will already be
        in b
        """
        # default implementation
        deserialize_messages(b, msg_queue, self.recv_data_class, queue_size=self.queue_size)
        
    def get_header_fields(self):
        """
        Header fields that should be sent over the connection. The header fields
        are protocol specific (i.e. service vs. topic, publisher vs. subscriber).
        @return: {str : str}: header fields to send over connection
        @rtype: dict
        """
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

class TCPROSTransport(Transport):
    """
    Generic implementation of TCPROS exchange routines for both topics and services
    """
    transport_type = 'TCPROS'
    
    def __init__(self, protocol, name, header=None):
        """
        ctor
        @param name str: identifier
        @param protocol TCPROSTransportProtocol protocol implementation    
        @param header dict: (optional) handshake header if transport handshake header was
        already read off of transport.
        @raise TransportInitError if transport cannot be initialized according to arguments
        """
        super(TCPROSTransport, self).__init__(protocol.direction, name=name)
        if not name:
            raise TransportInitError("Unable to initialize transport: name is not set")

        self.protocol = protocol

        self.socket = None
        self.endpoint_id = 'unknown'
        self.read_buff = cStringIO.StringIO()
        self.write_buff = cStringIO.StringIO()
            
        self.header = header

        # #1852 have to hold onto latched messages on subscriber side
        self.is_latched = False
        self.latch = None
        
        # these fields are actually set by the remote
        # publisher/service. they are set for tools that connect
        # without knowing the actual field name
        self.md5sum = None
        self.type = None 
            
    def set_socket(self, sock, endpoint_id):
        """
        Set the socket for this transport
        @param sock: socket
        @type  sock: socket.socket
        @param endpoint_id: identifier for connection endpoint
        @type  endpoint_id: str
        @raise TransportInitError: if socket has already been set
        """
        if self.socket is not None:
            raise TransportInitError("socket already initialized")
        self.socket = sock
        self.endpoint_id = endpoint_id

    def connect(self, dest_addr, dest_port, endpoint_id, timeout=None):
        """
        Establish TCP connection to the specified
        address/port. connect() always calls L{write_header()} and
        L{read_header()} after the connection is made
        @param dest_addr: destination IP address
        @type  dest_addr: str
        @param dest_port: destination port
        @type  dest_port: int                
        @param endpoint_id: string identifier for connection (for statistics)
        @type  endpoint_id: str
        @param timeout: (optional keyword) timeout in seconds
        @type  timeout: float
        @raise TransportInitError: if unable to create connection
        """
        try:
            self.endpoint_id = endpoint_id
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if timeout is not None:
                self.socket.settimeout(timeout)
            self.socket.connect((dest_addr, dest_port))
            self.write_header()
            self.read_header()
        except TransportInitError, tie:
            logwarn("Unable to initiate TCP/IP socket to %s:%s (%s): %s"%(dest_addr, dest_port, endpoint_id, tie))
            rospyerr("Unable to initiate TCP/IP socket to %s:%s (%s): %s"%(dest_addr, dest_port, endpoint_id, traceback.format_exc()))            
            raise
        except Exception, e:
            self.done = True
            if self.socket:
                self.socket.close()
            self.socket = None
            
            logerr("Unknown error initiating TCP/IP socket to %s:%s (%s): %s"%(dest_addr, dest_port, endpoint_id, str(e)))
            rospywarn("Unknown error initiating TCP/IP socket to %s:%s (%s): %s"%(dest_addr, dest_port, endpoint_id, traceback.format_exc()))            
            raise TransportInitError(str(e)) #re-raise i/o error
                
    def _validate_header(self, header):
        """
        Validate header and initialize fields accordingly
        @param header: header fields from publisher
        @type  header: dict
        @raise TransportInitError: if header fails to validate
        """
        self.header = header
        if 'error' in header:
            raise TransportInitError("remote error reported: %s"%header['error'])
        for required in ['md5sum', 'type']:
            if not required in header:
                raise TransportInitError("header missing required field [%s]"%required)
        self.md5sum = header['md5sum']
        self.type = header['type']
        if header.get('latching', '0') == '1':
            self.is_latched = True

    def write_header(self):
        """Writes the TCPROS header to the active connection."""
        sock = self.socket
        # socket may still be getting spun up, so wait for it to be writable
        fileno = sock.fileno()
        ready = None
        while not ready:
            _, ready, _ = select.select([], [fileno], [])
        logger.debug("[%s]: writing header", self.name)
        sock.setblocking(1)
        self.stat_bytes += write_ros_handshake_header(sock, self.protocol.get_header_fields())

    def read_header(self):
        """
        Read TCPROS header from active socket
        @raise TransportInitError if header fails to validate
        """
        self.socket.setblocking(1)
        self._validate_header(read_ros_handshake_header(self.socket, self.read_buff, self.protocol.buff_size))
                
    def send_message(self, msg, seq):
        """
        Convenience routine for services to send a message across a
        particular connection. NOTE: write_data is much more efficient
        if same message is being sent to multiple connections. Not
        threadsafe.
        @param msg: message to send
        @type  msg: Msg
        @param seq: sequence number for message
        @type  seq: int
        @raise TransportException: if error occurred sending message
        """
        # this will call write_data(), so no need to keep track of stats
        serialize_message(self.write_buff, seq, msg)
        self.write_data(self.write_buff.getvalue())
        self.write_buff.truncate(0)

    def write_data(self, data):
        """
        Write raw data to transport
        @raise TransportInitialiationError: could not be initialized
        @raise TransportTerminated: no longer open for publishing
        """
        if not self.socket:
            raise TransportInitError("TCPROS transport was not successfully initialized")
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
                logdebug("[%s]: Closing connection [%s] due to broken pipe", self.name, self.endpoint_id)
                self.close()
                raise TransportTerminated(msg)
            elif errno == 104: #connection reset by peer
                logdebug("[%s]: Peer [%s] has closed connection", self.name, self.endpoint_id) 
                self.close()
                raise TransportTerminated(msg)
            else:
                rospydebug("unknown socket error writing data: %s",traceback.format_exc())
                logdebug("[%s]: closing connection [%s] due to unknown socket error: %s", self.name, self.endpoint_id, msg) 
                self.close()
                raise TransportTerminated(str(errno)+' '+msg)                
        except:
            #TODO: try to figure out common errors here
            raise
        return True
    
    def receive_once(self):
        """
        block until messages are read off of socket
        @return: list of newly received messages
        @rtype: [Msg]
        @raise TransportException: if unable to receive message due to error
        """
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
                
            # #1852: keep track of last latched message
            if self.is_latched:
                self.latch = msg_queue[-1]
            
            return msg_queue

        except DeserializationError, e:
            rospyerr(traceback.format_exc())            
            raise TransportException("receive_once[%s]: DeserializationError %s"%(self.name, str(e)))
        except TransportTerminated, e:
            raise #reraise
        except ServiceException, e:
            raise
        except Exception, e:
            rospyerr(traceback.format_exc())
            raise TransportException("receive_once[%s]: unexpected error %s"%(self.name, str(e)))
        return retval
        
    def receive_loop(self, msgs_callback):
        """
        Receive messages until shutdown
        @param msgs_callback: callback to invoke for new messages received    
        @type  msgs_callback: fn([msg])
        """
        # - use assert here as this would be an internal error, aka bug
        logger.debug("receive_loop for [%s]", self.name)
        try:
            try:
                while not self.done and not is_shutdown():
                    msgs = self.receive_once()
                    if not self.done and not is_shutdown():
                        msgs_callback(msgs)

                rospydebug("receive_loop[%s]: done condition met, exited loop"%self.name)
            except DeserializationError, e:
                logerr("[%s] error deserializing incoming request: %s"%self.name, str(e))
                rospyerr("[%s] error deserializing incoming request: %s"%self.name, traceback.format_exc())                 
            except:
                # in many cases this will be a normal hangup, but log internally
                try:
                    #1467 sometimes we get exceptions due to
                    #interpreter shutdown, so blanket ignore those if
                    #the reporting fails
                    rospydebug("exception in receive loop for [%s], may be normal. Exception is %s",self.name, traceback.format_exc())                    
                except: pass
        finally:
            if not self.done:
                self.close()
                self.done = True

    def close(self):
        """close i/o and release resources"""
        super(TCPROSTransport, self).close()
        self.done = True
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        self.read_buff = self.write_buff = self.protocol = None

