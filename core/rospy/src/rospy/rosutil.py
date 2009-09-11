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
"""Utilities for ROS"""

## Utilities for ROS
#
# This is generally utilities that are not ROS-specific such as code
# decorators and os/environment/network routines.

import os
import sys
import socket
import string
import thread
import logging

import roslib.network 

## \ingroup validators
#  Exception that is raised when a parameter fails validation checks
class ParameterInvalid(Exception):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return str(self.message)
    
ROSRPC = "rosrpc://"

## utility function for parsing ROS-RPC URIs
## @param uri str: ROSRPC URI
## @return (str, int): address, port
## @throws ParameterInvalid if \a uri is not a valid ROSRPC URI
def parse_rosrpc_uri(uri):
    if uri.startswith(ROSRPC):
        dest_addr = uri[len(ROSRPC):]            
    else:
        dest_addr = uri
    try:
        dest_addr, dest_port = dest_addr.split(':')
        dest_port = string.atoi(dest_port)
    except:
        raise ParameterInvalid("ROS service URL is invalid: %s"%uri)
    return dest_addr, dest_port

##################################################################3
# Convenience Decorators

## \ingroup validators
#  Validator that checks that parameter is a positive int
def notNegative(paramName):
    def validator(param, context):
        if param is None or type(param) != int or param < 0:
            raise ParameterInvalid("ERROR: parameter [%s] cannot be negative"%paramName)
        return param
    return validator

## \ingroup validators
#  Validator that checks that parameter is not None
def notNone(paramName):
    def validator(param, context):
        if param is None:
            raise ParameterInvalid("ERROR: parameter [%s] must be specified"%paramName)
        return param
    return validator

## \ingroup validators
#  Validator that checks that parameter is not empty
def nonEmpty(paramName):
    def validator(param, context):
        if not param:
            raise ParameterInvalid("ERROR: parameter [%s] must be specified and non-empty"%paramName)
        return param
    return validator

## \ingroup validators
#  Validator that checks that parameter is a string and non-empty
def nonEmptyStr(paramName):
    def validator(param, context):
        if not param:
            raise ParameterInvalid("ERROR: parameter [%s] must be specified and non-empty"%paramName)
        elif type(param) != str:
            raise ParameterInvalid("ERROR: parameter [%s] must be a string"%paramName)            
        return param
    return validator
        
## \ingroup validators
#  Validator that checks that parameter is a string 
def isStr(paramName):
    def validator(param, context):
        if param is None or type(param) == str:
            return param
        raise ParameterInvalid("ERROR: parameter [%s] must be a string"%paramName)            
    return validator

##################################################################3
# Network utilities

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
            raise Exception("%s did not connect"%self.__class__.__name__)
        while not self.is_shutdown:
            try:
                (client_sock, client_addr) = self.server_sock.accept()
                #leave threading decisions up to inbound_handler
                self.inbound_handler(client_sock, client_addr)
            except socket.error, (errno, string):
                if not self.is_shutdown:
                    print >> sys.stderr, "TCPServer: socket error", errno, string
                    logging.getLogger("rospy.util.tcpserver").error("socket.error[%s]: %s", errno, string)
        logging.getLogger("rospy.rosutil.tcpserver").info("TCPServer[%s] shutting down", self.port)

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
        self.is_shutdown = True
        self.server_sock.shutdown(socket.SHUT_RDWR)
        self.server_sock.close()
