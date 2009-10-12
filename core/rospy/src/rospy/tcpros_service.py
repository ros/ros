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

"""Internal use: Service-specific extensions for TCPROS support"""

import cStringIO
import socket
import struct
import sys
import logging
import thread
import traceback

import roslib.scriptutil 

from rospy.exceptions import TransportInitError, TransportTerminated
from rospy.registration import get_service_manager
from rospy.service import _Service, ServiceException
from rospy.tcpros_base import TCPROSTransport, TCPROSTransportProtocol, \
    get_tcpros_server_address, start_tcpros_server, recv_buff, \
    DEFAULT_BUFF_SIZE

from rospy.core import logwarn, loginfo, logerr, logdebug
import rospy.core
import rospy.msg
import rospy.names
import rospy.validators

logger = logging.getLogger('rospy.service')

## Process incoming service connection. For use with
## TCPROSServer. Reads in service name from handshake and creates the
## appropriate service handler for the connection.
## @param sock socket: socket connection
## @param client_addr (str, int): client address
## @param header dict: key/value pairs from handshake header
## @return str: error string or None 
def service_connection_handler(sock, client_addr, header):
    for required in ['service', 'md5sum', 'callerid']:
        if not required in header:
            return "Missing required '%s' field"%required
    else:
        logger.debug("connection from %s:%s", client_addr[0], client_addr[1])
        service_name = header['service']
        sm = get_service_manager()
        md5sum = header['md5sum']
        service = sm.get_service(service_name)
        if not service:
            return "[%s] is not a provider of  [%s]"%(rospy.names.get_caller_id(), service_name)
        elif md5sum != rospy.names.SERVICE_ANYTYPE and md5sum != service.service_class._md5sum:
            return "request from [%s]: md5sums do not match: [%s] vs. [%s]"%(header['callerid'], md5sum, service.service_class._md5sum)
        else:
            transport = TCPROSTransport(service.protocol, service_name, header=header)
            transport.set_socket(sock, header['callerid'])
            transport.write_header()
            # using threadpool reduced performance by an order of
            # magnitude, need to investigate better
            thread.start_new_thread(service.handle, (transport, header))
                
        
class TCPService(TCPROSTransportProtocol):
    """
    Protocol implementation for Services over TCPROS
    """

    def __init__(self, resolved_name, service_class, buff_size=DEFAULT_BUFF_SIZE):
        """
        ctor.
        @param resolved_name: name of service
        @type  resolved_name: str
        @param service_class: Service data type class
        @type  service_class: Service
        @param buff_size int: size of buffer (bytes) to use for reading incoming requests.
        @type  buff_size: int
        """
        super(TCPService, self).__init__(resolved_name, service_class._request_class, buff_size=buff_size)
        self.service_class = service_class

    ## @param self

    def get_header_fields(self):
        """
        Protocol API
        @return: header fields
        @rtype: dict
        """
        return {'service': self.resolved_name, 'type': self.service_class._type,
                'md5sum': self.service_class._md5sum, 'callerid': rospy.names.get_caller_id() }

class TCPROSServiceClient(TCPROSTransportProtocol):
    """Protocol Implementation for Service clients over TCPROS"""
    
    def __init__(self, resolved_name, service_class, headers=None, buff_size=DEFAULT_BUFF_SIZE):
        """
        ctor.
        @param resolved_name: resolved service name 
        @type  resolved_name: str
        @param service_class: Service data type class
        @type  service_class: Service
        @param headers: identifier for Service session
        @type  headers: dict
        @param buff_size: size of buffer (bytes) for reading responses from Service. 
        @type  buff_size: int
        """
        super(TCPROSServiceClient, self).__init__(resolved_name, service_class._response_class)
        self.service_class = service_class
        self.headers = headers or {}
        self.buff_size = buff_size
        
    def get_header_fields(self):
        """
        TCPROSTransportProtocol API        
        """
        headers = {'service': self.resolved_name, 'md5sum': self.service_class._md5sum,
                   'callerid': rospy.names.get_caller_id()}
        # The current implementation allows user-supplied headers to
        # override protocol-specific headers.  We may want to
        # eliminate this in the future if it is abused too severely.
        for k, v in self.headers.iteritems():
            headers[k] = v
        return headers
    
    def _read_ok_byte(self, b, sock):
        """
        Utility for reading the OK-byte/error-message header preceding each message.
        @param sock: socket connection. Will be read from if OK byte is
        false and error message needs to be read
        @type  sock: socket.socket
        @param b: buffer to read from
        @type  b: StringIO
        """
        if b.tell() == 0:
            return
        pos = b.tell()
        b.seek(0)
        ok = struct.unpack('<B', b.read(1))[0] # read in ok byte
        b.seek(pos)
        if not ok:
            str = self._read_service_error(sock, b)
            raise ServiceException("service [%s] responded with an error: %s"%(self.resolved_name, str))
        
    def read_messages(self, b, msg_queue, sock):
        """
        In service implementation, reads in OK byte that preceeds each
        response. The OK byte allows for the passing of error messages
        instead of a response message
        @param b: buffer
        @type  b: StringIO
        @param msg_queue: Message queue to append to
        @type  msg_queue: [Message]
        @param sock: socket to read from
        @type  sock: socket.socket
        """
        self._read_ok_byte(b, sock)
        rospy.msg.deserialize_messages(b, msg_queue, self.recv_data_class, queue_size=self.queue_size, max_msgs=1, start=1) #rospy.msg
        #deserialize_messages only resets the buffer to the start
        #point if everything was consumed, so we have to further reset
        #it.
        if b.tell() == 1:
            b.seek(0)
        
    ## read service error from sock 
    ## @param self
    ## @param sock socket: socket to read from
    ## @param b StringIO: currently read data from sock
    def _read_service_error(self, sock, b):
        buff_size = 256 #can be small given that we are just reading an error string
        while b.tell() < 5:
            recv_buff(sock, b, buff_size)
        bval = b.getvalue()
        (length,) = struct.unpack('<I', bval[1:5]) # ready in len byte
        while b.tell() < 5 + length:
            recv_buff(sock, b, buff_size)
        bval = b.getvalue()
        return struct.unpack('<%ss'%length, bval[5:5+length])[0] # ready in len byte

    
## \ingroup clientapi
class ServiceProxy(_Service):
    """
    Create a handle to a ROS service for invoking calls.

    Usage::
      add_two_ints = ServiceProxy('add_two_ints', AddTwoInts)
      resp = add_two_ints(1, 2)
    """
    
    def __init__(self, name, service_class, persistent=False, headers=None):
        """
        ctor.
        @param name: name of service to call
        @type  name: str
        @param service_class: auto-generated service class
        @type  service_class: Service class
        @param persistent: if True, proxy maintains a persistent
        connection to service. While this results in better call
        performance, persistent connections are discouraged as they are
        less resistent to network issues and service restarts.
        @type  persistent: bool
        @param headers: arbitrary headers 
        @type  headers: dict
        """
        super(ServiceProxy, self).__init__(name, service_class)
        self.uri = None
        self.seq = 0
        self.buff_size = DEFAULT_BUFF_SIZE
        self.persistent = persistent
        if persistent:
            if not headers:
                headers = {}
            headers['persistent'] = '1'
        self.protocol = TCPROSServiceClient(self.resolved_name,
                                            self.service_class, headers=headers)
        self.transport = None #for saving persistent connections

    def __call__(self, *args, **kwds):
        """
        callable-style version of the service api (#425)
        @param args: arguments to remote service
        @param kwds: set timeout=float to override service call timeout
        @raise ROSSerializationException: If unable to serialize
        message. This is usually a type error with one of the fields.
        """
        if len(args) == 1 and self.request_class == args[0].__class__:
            return self.call(args[0], **kwds)
        else:
            return self.call(self.request_class(*args), **kwds)
    
    def _get_service_uri(self, request, timeout):
        """
        private routine for getting URI of service to call
        @param request: request message
        @type  request: Message
        @param timeout: timeout in seconds
        @type  timeout: float
        """
        if not isinstance(request, rospy.msg.Message):
            raise TypeError("request object is not a valid request message instance")
        if not self.request_class == request.__class__:
            raise TypeError("request object type [%s] does not match service type [%s]"%(request.__class__, self.request_class))

        #TODO: subscribe to service changes
        #if self.uri is None:
        if 1: #always do lookup for now, in the future we need to optimize
            try:
                try:
                    code, msg, self.uri = roslib.scriptutil.get_master().lookupService(rospy.names.get_caller_id(), self.resolved_name)
                except:
                    raise ServiceException("unable to contact master")
                if code != 1:
                    logger.error("[%s]: lookup service failed with message [%s]", self.resolved_name, msg)
                    raise ServiceException("service [%s] unavailable"%self.resolved_name)
                
                # validate
                try:
                    rospy.core.parse_rosrpc_uri(self.uri)
                except rospy.validators.ParameterInvalid:
                    raise ServiceException("master returned invalid ROSRPC URI: %s"%self.uri)
            except socket.error, e:
                logger.error("[%s]: socket error contacting service, master is probably unavailable",self.resolved_name)
        return self.uri

    def call(self, request, timeout=None):
        """
        Call the service 
        @param request: service request message instance
        @type  request: Messsage
        @param timeout: (keyword arg) number of seconds before request times out
        @type  timeout: float
        @raise TypeError: if request is not of the valid type (Message)
        @raise ServiceException: if communication with remote service fails
        @raise ROSSerializationException: If unable to serialize
        message. This is usually a type error with one of the fields.
        """
        
        # initialize transport
        if self.transport is None:
            service_uri = self._get_service_uri(request, timeout)
            dest_addr, dest_port = rospy.core.parse_rosrpc_uri(service_uri)

            # connect to service            
            transport = TCPROSTransport(self.protocol, self.resolved_name)
            transport.buff_size = self.buff_size
            try:
                transport.connect(dest_addr, dest_port, service_uri, timeout=timeout)
            except TransportInitError, e:
                # can be a connection or md5sum mismatch
                raise ServiceException("unable to connect to service: %s"%e)
            self.transport = transport
        else:
            transport = self.transport

        # send the actual request message
        self.seq += 1
        transport.send_message(request, self.seq)
        responses = transport.receive_once()
        if len(responses) == 0:
            raise ServiceException("service [%s] returned no response"%self.resolved_name)
        elif len(responses) > 1:
            raise ServiceException("service [%s] returned multiple responses: %s"%(self.resolved_name, len(responses)))
        
        if not self.persistent:
            transport.close()
            self.transport = None
        return responses[0]

    
    def close(self):
        """Close this ServiceProxy. This only has an effect on persistent ServiceProxy instances."""
        if self.transport is not None:
            self.transport.close()

class Service(_Service):
    """
    Declare a ROS service. Service requests are passed to the
    specified handler. 

    Service Usage::
      s = Service('getmapservice', GetMap, get_map_handler)
    """

    def __init__(self, name, service_class, handler, buff_size=DEFAULT_BUFF_SIZE):
        """
        ctor.
        @param name: service name
        @type  name: str
        @param service_class: ServiceDefinition class
        @type  service_class: ServiceDefinition class
        @param handler: callback function for processing service
        request. Function takes in a ServiceRequest and returns a
        ServiceResponse of the appropriate type.
        @type  handler: fn(req)->resp
        @param buff_size: size of buffer for reading incoming requests. Should be at least size of request message
        @type  buff_size: int
        """
        super(Service, self).__init__(name, service_class)
        self.handler = handler
        self.registered = False
        self.seq = 0
        self.done = False
        self.buff_size=buff_size

        start_tcpros_server() #lazy-init the tcprosserver
        host, port = get_tcpros_server_address()
        self.uri = '%s%s:%s'%(rospy.core.ROSRPC, host, port)
        logdebug("... service URL is %s"%self.uri)

        self.protocol = TCPService(self.resolved_name, service_class, self.buff_size)

        logdebug("[%s]: new Service instance"%self.resolved_name)
        get_service_manager().register(self.resolved_name, self)

    # TODO: should consider renaming to unregister
    
    def shutdown(self, reason=''):
        """
        Stop this service
        @param reason: human-readable shutdown reason
        @type  reason: str
        """
        self.done = True
        logdebug('[%s].shutdown: reason [%s]'%(self.resolved_name, reason))
        try:
            get_service_manager().unregister(self.resolved_name, self)
        except Exception, e:
            logerr("Unable to unregister with master: "+traceback.format_exc())
            raise ServiceException("Unable to connect to master: %s"%e)

    def spin(self):
        """
        Let service run and take over thread until shutdown. Use this method to keep
        your scripts from exiting execution.
        """
        try:
            while not rospy.core.is_shutdown() and not self.done:
                time.sleep(0.5)
        except KeyboardInterrupt:
            logdebug("keyboard interrupt, shutting down")

    def _write_service_error(self, transport, err_msg):
        """
        Send error message to client
        @param transport: transport connection to client 
        @type  transport: Transport
        @param err_msg: error message to send to client
        @type  err_msg: str
        """
        transport.write_data(struct.pack('<BI%ss'%len(err_msg), 0, len(err_msg), err_msg))
        
    def _handle_request(self, transport, request):
        """
        Process a single incoming request.
        @param transport: transport instance
        @type  transport: L{TCPROSTransport}
        @param request: Message
        """
        try:
            response = self.handler(request)
            if response is None:
                try:
                    self._write_service_error(transport, "service did not generate a response")
                except:
                    logger.error(traceback.format_exc())
            else:
                self.seq += 1
                # ok byte
                transport.write_buff.write(struct.pack('<B', 1))
                transport.send_message(response, self.seq)
        except Exception, e:
            logerr("Error processing request: %s\n%s"%(e,traceback.print_exc()))
            self._write_service_error(transport, "error processing request: %s"%e)
    
    def handle(self, transport, header):
        """
        Process incoming request. This method should be run in its
        own thread. If header['persistent'] is set to 1, method will
        block until connection is broken.
        @param transport: transport instance
        @type  transport: L{TCPROSTransport}
        @param header: headers from client
        @type  header: dict
        """
        if 'persistent' in header and \
               header['persistent'].lower() in ['1', 'true']:
            persistent = True
        else:
            persistent = False
        if header.get('probe', None) == '1':
            #this will likely do more in the future
            transport.close()
            return
        handle_done = False
        while not handle_done:
            try:
                requests = transport.receive_once()
                for request in requests:
                    self._handle_request(transport, request)
                if not persistent:
                    handle_done = True
            except rospy.exceptions.TransportTerminated, e:
                if not persistent:
                    logerr("incoming connection failed: %s"%e)
                logdebug("service[%s]: transport terminated"%self.resolved_name)
                handle_done = True
        transport.close()
