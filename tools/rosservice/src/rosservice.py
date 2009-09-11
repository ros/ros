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

import roslib.names
import roslib.scriptutil
import rospy

from optparse import OptionParser

class RosServiceException(Exception): pass

def succeed(args):
    code, msg, val = args
    if code != 1:
        raise RosServiceException("remote call failed: %s"%msg)
    return val

def _get_service_headers(service_name, service_uri):
    dest_addr, dest_port = rospy.parse_rosrpc_uri(service_uri)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        # connect to service and probe it to get the headers
        s.settimeout(5.0)
        s.connect((dest_addr, dest_port))
        h = "probe=1\nmd5sum=*\ncallerid=/roservice\nservice=%s\n"%service_name
        s.sendall(struct.pack('<I', len(h)) + h)

        # read the response
        buff = cStringIO.StringIO()
        bytes = 0
        while 1:
            data = s.recv(512)
            if not data:
                break
            else:
                # rospy automatically breaks the connection after
                # sending the header on a probe.  roscpp does not so
                # we have to break the connection ourselves once we
                # have the header.
                buff.write(data)
                bytes += len(data)
                # check to see if we've read the full header
                if bytes > 3:
                    bval = buff.getvalue()
                    (length,) = struct.unpack('<I', bval[0:4])
                    if length+4 >= len(bval):
                        break

        # check loop termination state
        if bytes < 4:
            print >> sys.stderr, "Did not receive full header back from service, cannot interrogate"
            return{}
        if length+4 < len(bval):
            print >> sys.stderr, "Did not receive full header back from service, cannot interrogate"
            return {}
        
        bval = bval[4:4+length]
        headers = {}
        for l in bval.split('\n'):
            idx = l.find('=')
            if idx > 0:
                headers[l[:idx]] = l[idx+1:]
        return headers
    finally:
        if s is not None:
            s.close()
    
def get_service_type(service_name):
    master = roslib.scriptutil.get_master()
    code, msg, service_uri = master.lookupService('/rosservice', service_name)
    if code == -1:
        print >> sys.stderr, "Unknown service [%s]"%service_name
        return None
    elif code == 0:
        print >> sys.stderr, "Master is malfunctioning: %s"%msg
        return None
    else:
        return _get_service_headers(service_name, service_uri).get('type', None)

def rosservice_type(service_name):
    service_type = get_service_type(service_name)
    if service_type is None:
        print "unknown"
    else:
        print service_type

## @param service_name str: name of service to lookup
## @return str: ROSRPC URI for \a service_name
def get_service_uri(service_name):
    master = roslib.scriptutil.get_master()
    code, msg, url = master.lookupService('/rosservice', service_name)
    if code == 1:
        return url
    return None

## Implements rosservice uri command
## @param service_name str: name of service to lookup
def rosservice_uri(service_name):
    uri = get_service_uri(service_name)
    if uri:
        print uri
    else:
        print >> sys.stderr, "Unknown service: %s"%service_name

## Get the list of services
## @param node str: Name of node to print services for or None to return all services
## @param include_nodes bool: If True, return list will be [service_name, [node]]
## @return [services]: if \a include_nodes, services is service_name,
## [node]. Otherwise, it is just the service_name
def get_service_list(node=None, include_nodes=False):
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
    
## Implements 'rosservice list'
## @param node str: Name of node to print services for or None to print all services
## @param print_nodes bool: If True, also print nodes providing service
def rosservice_list(node=None, print_nodes=False):
    srvs = get_service_list(node=node, include_nodes=print_nodes)
    for s in srvs:
        if print_nodes:
            print s[0]+' '+','.join(s[1])
        else:
            print s

##########################################################################################
# COMMAND PROCESSING #####################################################################

def _optparse_service_only(cmd, argv=sys.argv):
    args = argv[2:]
    parser = OptionParser(usage="usage: %%prog %s /service"%cmd, prog=NAME)
    (options, args) = parser.parse_args(args)
    if len(args) == 0:
        parser.error("service must be specified")        
    if len(args) > 1:
        parser.error("you may only specify one input service")
    return roslib.scriptutil.script_resolve_name('rosservice', args[0])

def rosservice_cmd_type():
    rosservice_type(_optparse_service_only('type'))
    
def rosservice_cmd_uri():
    rosservice_uri(_optparse_service_only('type'))

def rosservice_cmd_list():
    args = sys.argv[2:]
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
    
def fullusage():
    print """Commands:
\trosservice call\tcall the service with the provided args
\trosservice type\tprint service type
\trosservice list\tprint information about active topics

Type rosservice <command> -h for more detailed usage, e.g. 'rosservice call -h'
"""
    sys.exit(os.EX_USAGE)

def rosservicemain(argv=sys.argv):
    if len(argv) == 1:
        fullusage()
    try:
        command = argv[1]
        if command == 'list':
            rosservice_cmd_list()
        elif command == 'type':
            rosservice_cmd_type()
        elif command == 'uri':
            rosservice_cmd_uri()
        elif command == 'call':
            rosservice_cmd_call(argv)
        else:
            fullusage()
    except socket.error:
        print >> sys.stderr, "Network communication failed. Most likely failed to communicate with master."
    except RosServiceException, e:
        print >> sys.stderr, str(e)

if __name__ == '__main__':
    rosservicemain()
