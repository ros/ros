#!/usr/bin/env python
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
# Revision $Id: rosservice.py 3813 2009-02-11 21:16:34Z sfkwc $

import roslib; roslib.load_manifest('rosservice')

NAME='rosservice'

import cStringIO
import os
import sys
import socket
import struct
import threading
import time

import roslib.message 
import roslib.names
import roslib.network
import roslib.scriptutil
import rospy
import rosmsg

from optparse import OptionParser

class ROSServiceException(Exception): pass

class ROSServiceIOException(ROSServiceException): pass
class ROSServiceIOException(ROSServiceException): pass

## Utility that raises a ROSServiceException if ROS XMLRPC command fails
## @param args (code, msg, val): ROS XMLRPC call return args
## @return value argument from ROS XMLRPC call (third arg of tuple)
## @throws ROSServiceException if XMLRPC command does not return a SUCCESS code
def succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSServiceException("remote call failed: %s"%msg)
    return val

## Utility for connecting to a service and retrieving the TCPROS
## headers. Services currently do not declare their type with the
## master, so instead we probe the service for its headers.
## @param service_name str: name of service
## @param service_uri str: ROSRPC URI of service
## @return dict: map of header fields
## @throws ROSServiceIOException
def get_service_headers(service_name, service_uri):
    dest_addr, dest_port = rospy.parse_rosrpc_uri(service_uri)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        try:
            # connect to service and probe it to get the headers
            s.settimeout(5.0)
            s.connect((dest_addr, dest_port))
            header = { 'probe':'1', 'md5sum':'*',
                       'callerid':'/rosservice', 'service':service_name}
            roslib.network.write_ros_handshake_header(s, header)
            return roslib.network.read_ros_handshake_header(s, cStringIO.StringIO(), 2048)
        except socket.error:
            raise ROSServiceIOException("Unable to communicate with service [%s], address [%s]"%(service_name, service_uri))
    finally:
        if s is not None:
            s.close()
            
## Get the type of the specified \a service_name. May print errors to stderr.
## @param service_name str: name of service
## @return str: type of service or None
## @throws ROSServiceIOException
def get_service_type(service_name):
    master = roslib.scriptutil.get_master()
    try:
        code, msg, service_uri = master.lookupService('/rosservice', service_name)
    except socket.error:
        raise ROSServiceIOException("Unable to communicate with master!")
    if code == -1:
        print >> sys.stderr, "Unknown service [%s]"%service_name
        return None
    elif code == 0:
        raise ROSServiceIOException("Master is malfunctioning: %s"%msg)
    else:
        try:
            return get_service_headers(service_name, service_uri).get('type', None)
        except socket.error:
            raise ROSServiceIOException("Unable to communicate with service [%s]! Service address is [%s]"%(service_name, service_uri))

## Implements 'type' command. Prints service type to stdout
## @param service_name str: name of service
def rosservice_type(service_name):
    service_type = get_service_type(service_name)
    if service_type is None:
        print "unknown"
    else:
        print service_type

## @param service_name str: name of service to lookup
## @return str: ROSRPC URI for \a service_name
def get_service_uri(service_name):
    try:
        master = roslib.scriptutil.get_master()
        code, msg, url = master.lookupService('/rosservice', service_name)
        if code == 1:
            return url
        return None
    except socket.error:
        raise ROSServiceIOException("Unable to communicate with master!")

## Implements rosservice uri command
## @param service_name str: name of service to lookup
## @throws ROSServiceIOException
def rosservice_uri(service_name):
    uri = get_service_uri(service_name)
    if uri:
        print uri
    else:
        print >> sys.stderr, "Unknown service: %s"%service_name

## Implements rosservice node command
## @param service_name str: name of service to lookup
## @throws ROSServiceIOException
def rosservice_node(service_name):
    srvs = get_service_list(include_nodes=True)
    s = [s for s in srvs if s[0] == service_name]
    if s:
        if s[0][1]:
            print s[0][1][0]
        else:
            print >> sys.stderr, "Service %s no longer has a provider"%service_name
    else:
        print >> sys.stderr, "Unknown service: %s"%service_name

## Get the list of services
## @param node str: Name of node to print services for or None to return all services
## @param include_nodes bool: If True, return list will be [service_name, [node]]
## @return [services]: if \a include_nodes, services is service_name,
## [node]. Otherwise, it is just the service_name
## @throws ROSServiceIOException
def get_service_list(node=None, include_nodes=False):
    try:
        master = roslib.scriptutil.get_master()
        state = succeed(master.getSystemState('/rosservice'))
        srvs = state[2]

        if include_nodes:
            if node is None:
                return srvs
            else:
                return [(s, nodelist) for s, nodelist in srvs if node in nodelist]
        else:
            if node is None:
                return [s for s,_ in srvs]
            else:
                return [s for s,nodelist in srvs if node in nodelist]
    except socket.error:
        raise ROSServiceIOException("Unable to communicate with master!")
    
## Implements 'rosservice list'
## @param node str: Name of node to print services for or None to print all services
## @param print_nodes bool: If True, also print nodes providing service
## @throws ROSServiceIOException
def rosservice_list(node=None, print_nodes=False):
    srvs = get_service_list(node=node, include_nodes=print_nodes)
    for s in srvs:
        if print_nodes:
            print s[0]+' '+','.join(s[1])
        else:
            print s

## Lookup services by \a service_type
## @param service_type str: type of service to find
## @return [str]: list of service names that use \a service_type    
def rosservice_find(service_type):
    master = roslib.scriptutil.get_master()
    matches = []
    try:
        _, _, services = succeed(master.getSystemState('/rosservice'))
        for s, l in services:
            t = get_service_type(s)
            if t == service_type:
                matches.append(s)
    except socket.error:
        raise ROSServiceIOException("Unable to communicate with master!")
    return matches
    
## Implements 'rosservice type'
## @param argv [str]: command-line args
def rosservice_cmd_find(argv=sys.argv):
    args = argv[2:]
    parser = OptionParser(usage="usage: %prog find msg-type", prog=NAME)
    options, args = parser.parse_args(args)
    if not len(args):
        parser.error("please specify a message type")
    if len(args) > 1:
        parser.error("you may only specify one message type")
    print '\n'.join(rosservice_find(args[0]))

## Get the service class using the name of the service. NOTE: this
## call results in a probe call to the service.
## @param service_name str: fully-resolved name of service to call
## @return ServiceDefinition: service class
## @throws ROSServiceException if service class cannot be retrieved
def get_service_class_by_name(service_name):
    # lookup the service type
    service_type = get_service_type(service_name)
    if not service_type:
        # diagnose error
        srvs = get_service_list()
        if service_name not in srvs:
            raise ROSServiceException("Service [%s] is not available."%service_name)
        else:
            raise ROSServiceException("Unable to determine type of service [%s]."%service_name)

    # get the Service class so we can populate the request
    service_class = roslib.scriptutil.get_service_class(service_type)

    # #1083: roscpp services are currently returning the wrong type
    if service_class and service_type.endswith('Request') and \
            not hasattr(service_class, "_request_class"):
        service_type = service_type[:-7]
        service_class = roslib.scriptutil.get_service_class(service_type)
        
    if service_class is None:
        pkg = roslib.names.resource_name_package(service_type)
        raise ROSServiceException("Unable to load type [%s].\n"%service_type+
                                  "Have you typed 'make' in [%s]?"%pkg)
    return service_class

## Call the specified \a service_name
## @param service_name str: fully-resolved name of service to call
## @param service_args [args]: args to pass to service
## @param service_class Message class: (optional) service type
## class. If this argument is provided, it saves a probe call against
## the service
## @return roslib.message.Message: service response
## @throws ROSServiceException if call command cannot be executed
def call_service(service_name, service_args, service_class=None):
    if service_class is None:
        service_class = get_service_class_by_name(service_name)
    request = service_class._request_class()
    try:
        roslib.message.fill_message_args(request, service_args)
    except roslib.message.ROSMessageException:
        raise ROSServiceException("Not enough arguments to call service.\n"+\
                                      "Args are: [%s]"%roslib.message.get_printable_message_args(request))

    try:
        return request, rospy.ServiceProxy(service_name, service_class)(request)
    except rospy.ROSSerializationException, e:
        raise ROSServiceException("Unable to send request. One of the fields has an incorrect type:\n"+\
                                      "  %s\n\nsrv file:\n%s"%(e, rosmsg.get_srv_text(service_class._type)))

## Implements 'rosservice call'
## @param service_name str: name of service to call
## @param service_args [args]: arguments to call service with
## @param verbose bool: if True, print extra output
## @param service_class Message class: (optional) service type
## class. If this argument is provided, it saves a probe call against
## the service
## @throws ROSServiceException if call command cannot be executed
def rosservice_call(service_name, service_args, verbose=False, service_class=None):
    service_name = roslib.scriptutil.script_resolve_name('rosservice', service_name)
    request, response = call_service(service_name, service_args, service_class=service_class)
    if verbose:
        print str(request)
        print '---'
    print str(response)

## Check if service requires arguments
## @param service_name str: name of service being called
## @param service_class Message class: (optional) service type
## class. If this argument is provided, it saves a probe call against
## the service
## @return bool: True if \a service_name has request arguments
def has_service_args(service_name, service_class=None):
    service_name = roslib.scriptutil.script_resolve_name('rosservice', service_name)
    if service_class is None:
        service_class = get_service_class_by_name(service_name)
    return len(service_class._request_class.__slots__) > 0 
    
## Implements 'rosservice args'
## @param service_name str: name of service to get arguments for
## @throws ROSServiceException if call command cannot be executed
def rosservice_args(service_name):
    service_name = roslib.scriptutil.script_resolve_name('rosservice', service_name)
    service_class = get_service_class_by_name(service_name)
    print roslib.message.get_printable_message_args(service_class._request_class)
    
##########################################################################################
# COMMAND PROCESSING #####################################################################

## Parse command-line arguments for commands that take a service name
## only.  Will cause a system exit if command-line argument parsing
## fails.
## @param cmd str: command name, e.g. 'type'
## @param argv [str]: command-line arguments
def _optparse_service_only(cmd, argv=sys.argv):
    args = argv[2:]
    parser = OptionParser(usage="usage: %%prog %s /service"%cmd, prog=NAME)
    (options, args) = parser.parse_args(args)
    if len(args) == 0:
        parser.error("service must be specified")        
    if len(args) > 1:
        parser.error("you may only specify one input service")
    return roslib.scriptutil.script_resolve_name('rosservice', args[0])

## Parse 'type' command arguments and run command Will cause a system
## exit if command-line argument parsing fails.
## @param argv [str]: command-line arguments
## @throws ROSServiceException if type command cannot be executed
def rosservice_cmd_type(argv):
    rosservice_type(_optparse_service_only('type', argv=argv))
    
## Parse 'uri' command arguments and run command Will cause a system
## exit if command-line argument parsing fails.
## @param argv [str]: command-line arguments
## @throws ROSServiceException if uri command cannot be executed
def rosservice_cmd_uri(argv, ):
    rosservice_uri(_optparse_service_only('uri', argv=argv))
    
## Parse 'node' command arguments and run command Will cause a system
## exit if command-line argument parsing fails.
## @param argv [str]: command-line arguments
## @throws ROSServiceException if node command cannot be executed
def rosservice_cmd_node(argv, ):
    rosservice_node(_optparse_service_only('node', argv=argv))

## Parse 'args' command arguments and run command Will cause a system
## exit if command-line argument parsing fails.
## @param argv [str]: command-line arguments
## @throws ROSServiceException if args command cannot be executed
def rosservice_cmd_args(argv, ):
    rosservice_args(_optparse_service_only('args', argv=argv))    
    
## Parse 'call' command arguments and run command Will cause a system
## exit if command-line argument parsing fails.
## @param argv [str]: command-line arguments
## @throws ROSServiceException if call command cannot be executed
def rosservice_cmd_call(argv):
    try:
        import yaml
    except ImportError, e:
        raise ROSServiceException("Cannot import yaml. Please make sure the pyyaml system dependency is installed")

    args = argv[2:]
    parser = OptionParser(usage="usage: %prog call /service [args...]", prog=NAME)
    parser.add_option("-v", dest="verbose", default=False,
                      action="store_true",
                      help="print verbose output")

    (options, args) = parser.parse_args(args)
    if len(args) == 0:
        parser.error("service must be specified")
    service_name = args[0]

    # optimization: in order to prevent multiple probe calls against a service, lookup the service_class
    service_name = roslib.scriptutil.script_resolve_name('rosservice', args[0])
    service_class = get_service_class_by_name(service_name)
    
    # type-case using YAML 
    service_args = []
    for arg in args[1:]:
        service_args.append(yaml.load(arg))
    if not service_args and has_service_args(service_name, service_class=service_class):
        for service_args in _stdin_yaml_arg():
            if service_args:
                rosservice_call(service_name, service_args, verbose=options.verbose, service_class=service_class) 
    else:
        rosservice_call(service_name, service_args, verbose=options.verbose, service_class=service_class)

## @return iterator for next yaml document on stdin
def _stdin_yaml_arg():
    import yaml
    import select
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    try:
        arg = 'x'
        while not rospy.is_shutdown() and arg != '\n':
            buff = ''
            while arg != '\n' and arg.strip() != '---':
                val = poll.poll(1.0)
                if not val:
                    continue
                arg = sys.stdin.readline() + '\n'
                if arg.startswith('... logging'):
                    # temporary, until we fix rospy logging
                    continue
                elif arg.strip() != '---':
                    buff = buff + arg
            yield yaml.load(buff.rstrip())
    except select.error:
        return # most likely ctrl-c interrupt

## Parse 'list' command arguments and run command
## Will cause a system exit if command-line argument parsing fails.
## @param argv [str]: command-line arguments
## @throws ROSServiceException if list command cannot be executed
def rosservice_cmd_list(argv):
    args = argv[2:]
    parser = OptionParser(usage="usage: %prog list [/node]", prog=NAME)
    parser.add_option("-n", "--nodes",
                      dest="print_nodes", default=False, action="store_true",
                      help="print nodes that provide service")
    (options, args) = parser.parse_args(args)
    nodename = None
    if len(args) == 1:
        nodename = roslib.scriptutil.script_resolve_name('rosservice', args[0])
    elif len(args) > 1:
        parser.error("you may only specify one input node")
    rosservice_list(nodename, print_nodes=options.print_nodes)
    
## Print generic usage for rosservice
def fullusage():
    print """Commands:
\trosservice list\tprint information about active topics
\trosservice call\tcall the service with the provided args
\trosservice type\tprint service type
\trosservice find\tfind services by service type
\trosservice uri\tprint service ROSRPC uri

Type rosservice <command> -h for more detailed usage, e.g. 'rosservice call -h'
"""
    sys.exit(os.EX_USAGE)

## main entry point for rosservice command-line tool
def rosservicemain(argv=sys.argv):
    if len(argv) == 1:
        fullusage()
    try:
        command = argv[1]
        if command == 'list':
            rosservice_cmd_list(argv)
        elif command == 'type':
            rosservice_cmd_type(argv)
        elif command == 'uri':
            rosservice_cmd_uri(argv)
        elif command == 'node':
            rosservice_cmd_node(argv)
        elif command == 'call':
            rosservice_cmd_call(argv)
        elif command == 'args':
            rosservice_cmd_args(argv)
        elif command == 'find':
            rosservice_cmd_find(argv)
        else:
            fullusage()
    except socket.error:
        print >> sys.stderr, "Network communication failed with the master or a node."
        sys.exit(1)
    except ROSServiceException, e:
        print >> sys.stderr, "ERROR: "+str(e)
        sys.exit(2)
    except KeyboardInterrupt:
        pass
    
if __name__ == '__main__':
    rosservicemain()
