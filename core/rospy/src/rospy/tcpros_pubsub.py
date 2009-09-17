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

"""Internal use: Topic-specific extensions for TCPROS support"""

import logging
import socket
import thread

from rospy.core import logwarn, logerr, logdebug
import rospy.exceptions
import rospy.names
import rospy.registration
import rospy.transport

from rospy.tcpros_base import TCPROSTransport, TCPROSTransportProtocol, \
    get_tcpros_server_address, start_tcpros_server,\
    DEFAULT_BUFF_SIZE, TCPROS

class TCPROSSub(TCPROSTransportProtocol):
    """
    Subscription transport implementation for receiving topic data via
    peer-to-peer TCP/IP sockets
    """

    def __init__(self, name, recv_data_class, queue_size=None, \
                     buff_size=DEFAULT_BUFF_SIZE, tcp_nodelay=False):
        """
        ctor.
        @param recv_data_class: class to instantiate to receive
        messages
        @type recv_data_class: L{rospy.Message}
        @param queue_size: maximum number of messages to
        deserialize from newly read data off socket
        @type queue_size: int
        @param buff_size: recv buffer size
        @type buff_size: int
        @param tcp_nodelay: If True, request TCP_NODELAY from publisher
        @type tcp_nodelay: bool
        """
        super(TCPROSSub, self).__init__(name, recv_data_class, queue_size, buff_size)
        self.direction = rospy.transport.INBOUND
        self.tcp_nodelay = tcp_nodelay
        
    def get_header_fields(self):
        """
        @return: dictionary of subscriber fields
        @rtype: dict
        """
        return {'topic': self.name,
                'message_definition': self.recv_data_class._full_text,
                'tcp_nodelay': '1' if self.tcp_nodelay else '0',
                'md5sum': self.recv_data_class._md5sum,
                'type': self.recv_data_class._type,
                'callerid': rospy.names.get_caller_id()}        

# Separate method for easier testing
def _configure_pub_socket(sock, is_tcp_nodelay):
    """
    Configure socket options on a new publisher socket.
    @param sock: socket.socket
    @type sock: socket.socket
    @param is_tcp_nodelay: if True, TCP_NODELAY will be set on outgoing socket if available
    @param is_tcp_nodelay: bool
    """
    # #956: low latency, TCP_NODELAY support
    if is_tcp_nodelay:
        if hasattr(socket, 'TCP_NODELAY'):
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        else:
            logwarn("WARNING: cannot enable TCP_NODELAY as its not supported on this platform")

#TODO:POLLING: TCPROSPub currently doesn't actually do anything -- not until polling is implemented

class TCPROSPub(TCPROSTransportProtocol):
    """
    Publisher transport implementation for publishing topic data via
    peer-to-peer TCP/IP sockets. 
    """
    
    def __init__(self, name, pub_data_class, is_latch=False, headers=None):
        """
        ctor.
        @param name: topic name
        @type name: str
        @param pub_data_class: class to instance to receive messages
        @type pub_data_class: L{rospy.Message} class
        @param is_latch: If True, Publisher is latching
        @type is_latch: bool
        """
        # very small buffer size for publishers as the messages they receive are very small
        super(TCPROSPub, self).__init__(name, None, queue_size=None, buff_size=128)
        self.pub_data_class = pub_data_class
        self.direction = rospy.transport.OUTBOUND
        self.is_latch = is_latch
        self.headers = headers if headers else {}
        
    def get_header_fields(self):
        base = {'topic': self.name,
                'type': self.pub_data_class._type,
                'latching': '1' if self.is_latch else '0',
                'message_definition': self.pub_data_class._full_text,
                'md5sum': self.pub_data_class._md5sum, 
                'callerid': rospy.names.get_caller_id() }
        
        # this implementation allows the user to override builtin
        # fields.  this could potentially enable some interesting
        # features... or it could be really bad.
        if self.headers:
            base.update(self.headers)
        return base

class TCPROSHandler(rospy.transport.ProtocolHandler):
    """
    ROS Protocol handler for TCPROS. Accepts both TCPROS topic
    connections as well as ROS service connections over TCP. TCP server
    socket is run once start_server() is called -- this is implicitly
    called during init_publisher().
    """

    def __init__(self):
        """ctor"""
        self.tcp_nodelay_map = {} # { topic : tcp_nodelay}
    
    def set_tcp_nodelay(self, topic, tcp_nodelay):
        """
        @param tcp_nodelay: If True, sets TCP_NODELAY on publisher's
        socket (disables Nagle algorithm). This results in lower
        latency publishing at the cost of efficiency.
        @type tcp_nodelay: bool
        """
        self.tcp_nodelay_map[rospy.names.resolve_name(topic)] = tcp_nodelay

    def shutdown(self):
        """
        stops the TCP/IP server responsible for receiving inbound connections        
        """
        pass

    def create_transport(self, topic_name, pub_uri, protocol_params):
        """
        Connect to topic topic_name on Publisher pub_uri using TCPROS.
        @param topic_name str: topic name
        @type topic_name: str
        @param pub_uri: XML-RPC URI of publisher 
        @type pub_uri: str
        @param protocol_params: protocol parameters to use for connecting
        @type protocol_params: [XmlRpcLegal]
        @return: code, message, debug
        @rtype: (int, str, int)
        """
        
        #Validate protocol params = [TCPROS, address, port]
        if type(protocol_params) != list or len(protocol_params) != 3:
            return 0, "ERROR: invalid TCPROS parameters", 0
        if protocol_params[0] != TCPROS:
            return 0, "INTERNAL ERROR: protocol id is not TCPROS: %s"%id, 0
        id, dest_addr, dest_port = protocol_params

        sub = rospy.registration.get_topic_manager().get_subscriber_impl(topic_name)

        #Create connection 
        try:
            protocol = TCPROSSub(topic_name, sub.data_class, \
                                 queue_size=sub.queue_size, buff_size=sub.buff_size,
                                 tcp_nodelay=sub.tcp_nodelay)
            conn = TCPROSTransport(protocol, topic_name)
            # timeout is really generous. for now just choosing one that is large but not infinite
            conn.connect(dest_addr, dest_port, pub_uri, timeout=60.)
            thread.start_new_thread(conn.receive_loop, (sub.receive_callback,))
        except rospy.exceptions.TransportInitError, e:
            logerr("unable to create TCPROSSub: %s", e)
            return 0, "Internal error creating inbound TCP connection for [%s]: %s"%(topic_name, e), -1

        # Attach connection to _SubscriberImpl
        if sub.add_connection(conn): #pass tcp connection to handler
            return 1, "Connected topic[%s]. Transport impl[%s]"%(topic_name, conn.__class__.__name__), dest_port
        else:
            conn.close()
            return 0, "ERROR: Race condition failure: duplicate topic subscriber [%s] was created"%(topic_name), 0

    def supports(self, protocol):
        """
        @param protocol: name of protocol
        @type protocol: str
        @return: True if protocol is supported
        @rtype: bool
        """
        return protocol == TCPROS
    
    def get_supported(self):
        """
        Get supported protocols
        """
        return [[TCPROS]]
        
    def init_publisher(self, topic_name, protocol):
        """
        Initialize this node to receive an inbound TCP connection,
        i.e. startup a TCP server if one is not already running.
        @param topic_name: topic name
        @type topic_name: str    
        @param protocol: negotiated protocol
        parameters. protocol[0] must be the string 'TCPROS'
        @type protocol: [str, value*]
        @return: (code, msg, [TCPROS, addr, port])
        @rtype: (int, str, list)
        """
        if protocol[0] != TCPROS:
            return 0, "Internal error: protocol does not match TCPROS: %s"%protocol, []
        start_tcpros_server()
        addr, port = get_tcpros_server_address()
        return 1, "ready on %s:%s"%(addr, port), [TCPROS, addr, port]

    def topic_connection_handler(self, sock, client_addr, header):
        """
        Process incoming topic connection. Reads in topic name from
        handshake and creates the appropriate L{TCPROSPub} handler for the
        connection.
        @param sock: socket connection
        @type sock: socket.socket
        @param client_addr: client address
        @type client_addr: (str, int)
        @param header: key/value pairs from handshake header
        @type header: dict
        @return: error string or None 
        @rtype: str
        """
        for required in ['topic', 'md5sum', 'callerid']:
            if not required in header:
                return "Missing required '%s' field"%required
        else:
            topic_name = header['topic']
            md5sum = header['md5sum']
            tm = rospy.registration.get_topic_manager()
            topic = tm.get_publisher_impl(topic_name)
            if not topic:
                return "[%s] is not a publisher of  [%s]. Topics are %s"%(rospy.names.get_caller_id(), topic_name, tm.get_publications())
            elif md5sum != rospy.names.TOPIC_ANYTYPE and md5sum != topic.data_class._md5sum:
                # check to see if subscriber sent 'type' header. If they did, check that
                # types are same first as this provides a better debugging message
                if 'type' in header and header['type'] != topic.data_class._type:
                    return "topic types do not match: [%s] vs. [%s]"%(header['type'], topic.data_class._type)
                return "md5sums do not match: [%s] vs. [%s]"%(md5sum, topic.data_class._md5sum)
            else:
                #TODO:POLLING if polling header is present, have to spin up receive loop as well

                # #1334: tcp_nodelay support from subscriber option
                if 'tcp_nodelay' in header:
                    tcp_nodelay = True if header['tcp_nodelay'].strip() == '1' else False
                else:
                    tcp_nodelay = self.tcp_nodelay_map.get(topic_name, False)

                _configure_pub_socket(sock, tcp_nodelay)
                protocol = TCPROSPub(topic_name, topic.data_class, is_latch=topic.is_latch, headers=topic.headers)
                transport = TCPROSTransport(protocol, topic_name)
                transport.set_socket(sock, header['callerid'])
                transport.write_header()
                topic.add_connection(transport)
            

