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
"""
UDPROS connection protocol.
"""
## UDPROS connection protocol.
#  http://pr.willowgarage.com/wiki/ROS/UDPROS
# 

import rospy.registration
import rospy.transport

## ROS Protocol handler for UDPROS. 
class UDPROSHandler(rospy.transport.ProtocolHandler):
    
    ## ctor.
    ## @param self
    def __init__(self):
        self._connections = {} # { "addr:port" : UDPROSTransport}
    
    ## noop
    ## @param self
    def shutdown(self):
        pass

    ## Get UDPROSTransport instance associated with \a topic or create
    ## a new one if necessary. We only store one UDPTransport instance
    ## per topic as UDP is connection-less.
    def _get_transport(self, topic):
        if topic in self._transport:
            return self._transport[topic]
        else:
            #TODO: no point in too much bookkeeping as we can't tell when the client disappears anyway
            #TODO: have to determine constructor args
            #TODO: transport needs to keep track of clients?
            val = UDPROSTransport(topic)
            self._connections[topic] = val
            return val

    ## Establish communication to topic \a topic_name on Publisher \a pub_uri.
    ## @param self
    ## @param topic_name str: topic name
    ## @param pub_uri str: XML-RPC URI of publisher 
    ## @param protocol_params [XmlRpcLegal]: protocol parameters to use for connecting
    ## @return int, str, int: code, message, debug
    def create_transport(self, topic_name, pub_uri, protocol_params):
        
        #Validate protocol params = [UDPROS, address, port, headers]
        if type(protocol_params) != list or len(protocol_params) != 4:
            return 0, "ERROR: invalid UDPROS parameters", 0
        if protocol_params[0] != UDPROS:
            return 0, "INTERNAL ERROR: protocol id is not UDPROS: %s"%id, 0
        id, dest_addr, dest_port, headers = protocol_params

        #TODO: parse/validate headers

        sub = rospy.registration.get_topic_manager().get_subscriber_impl(topic_name)
        # Create Transport
        
        # TODO: create just a single 'connection' instance to represent
        # all UDP connections. 'connection' can take care of unifying
        # publication if addresses are the same
        transport = UDPTransport(protocol, topic_name, sub.receive_callback)        
        
        # Attach connection to _SubscriberImpl
        if sub.add_connection(transport): #pass udp connection to handler
            return 1, "Connected topic[%s]. Transport impl[%s]"%(topic_name, transport.__class__.__name__), dest_port
        else:
            transport.close()
            return 0, "ERROR: Race condition failure: duplicate topic subscriber [%s] was created"%(topic_name), 0

    ## @param self
    ## @param protocol str: name of protocol
    ## @return bool: True if protocol is supported
    def supports(self, protocol):
        return protocol == UDPROS
    
    ## @param self
    def get_supported(self):
        return [[UDPROS]]
        
    ## Prepare a transport based on one of the supported protocols
    ## declared by a Subscriber. Subscribers supply a list of
    ## supported protocols, of which one is selected by the Publisher
    ## and passed to init_publisher(). init_publisher is responsible
    ## for initializing the publisher based on the selection.
    ## @param self
    ## @param topic_name str
    ## @param protocol [str, value*]: negotiated protocol
    ## parameters. protocol[0] must be the string 'UDPROS'
    ## @return (int, str, list): (code, msg, [UDPROS, addr, port])
    def init_publisher(self, topic_name, protocol): 
        if protocol[0] != UDPROS:
            return 0, "Internal error: protocol does not match UDPROS: %s"%protocol, []
        #TODO
        return 1, "ready", [UDPROS]


## UDPROS communication routines
class UDPROSTransport(rospy.transport.Transport):
    transport_type = 'UDPROS'
    
    ## ctor
    ## @param self
    ## @param name str: topic or service name    
    ## @param protocol UDPROSTransportProtocol protocol implementation    
    ## @param header dict: (optional) handshake header if transport handshake header was
    ## already read off of transport.
    ## @throws TransportInitError if transport cannot be initialized according to arguments
    def __init__(self, protocol, name, header=None):
        super(UDPROSTransport, self).__init__(protocol.direction, name=name)
        if not name:
            raise TransportInitError("Unable to initialize transport: name is not set")

        self.done = False
        self.header = header
            
    ## Convenience routine for sending a message across a particular
    ## connection. NOTE: write_data is much more efficient if same
    ## message is being sent to multiple connections.
    ## @param self
    ## @param msg Msg: message to send
    ## @param seq int: sequence number for message
    ## @throws TransportException if error occurred sending message 
    def send_message(self, msg, seq):
        pass

    ## Write raw data to transport
    ## @throws TransportInitialiationError could not be initialized
    ## @throws TransportTerminated no longer open for publishing
    def write_data(self, data):
        # TODO
        # - cut into packets
        # write to address
        pass
    
    ## block until messages are read off of socket
    ## @return [Msg] : list of newly received messages
    ## @throws TransportException     
    def receive_once(self):
        pass

    ## Receive messages until shutdown
    ## @param self
    ## @param msgs_callback fn([msg]): callback to invoke for new messages received    
    def receive_loop(self, msgs_callback):
        pass
    
    ## close i/o and release resources
    def close(super):
        self(UDPROSTransport, self).close()
        #TODO
        self.done = True
    
_handler = UDPROSHandler()

def init_udpros():
    #TODO: initialize datagram server for receiving packets
    pass

def get_udpros_handler():
    return _handler
