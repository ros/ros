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
# Revision $Id$

NAME='rosnode'
ID = '/rosnode'

import os
import sys
import socket
import xmlrpclib

from optparse import OptionParser

import roslib.scriptutil as scriptutil 
import rospy

class RosNodeException(Exception): pass

def succeed(args):
    code, msg, val = args
    if code != 1:
        raise RosNodeException("remote call failed: %s"%msg)
    return val
        
_caller_apis = {}
def get_api(master, caller_id):
    caller_api = _caller_apis.get(caller_id, None)
    if not caller_api:
        code, msg, caller_api = master.lookupNode(ID, caller_id)
        if code != 1:
            return None
        else:
            _caller_apis[caller_id] = caller_api
    return caller_api

## @return str: string list of all nodes
def _sub_rosnode_listnodes(list_uri=False, list_all=False):
    master = scriptutil.get_master()
    state = succeed(master.getSystemState(ID))

    nodes = []
    import itertools
    for s in state:
        for t, l in s:
            nodes.extend(l)

    nodes = set(nodes)
    #print '-'*80
    #print "Nodes:"
    if list_all:
        return '\n'.join(["%s \t%s"%(get_api(master, n) or 'unknown address', n) for n in nodes])
    elif list_uri:
        return '\n'.join([(get_api(master, n) or 'unknown address') for n in nodes])
    else:
        return '\n'.join(nodes)
    
## print list of all ROS nodes
def rosnode_listnodes(list_uri=False, list_all=False):
    print _sub_rosnode_listnodes(list_uri=list_uri, list_all=list_all)
    
    
def rosnode_ping(node_name, max_count=None):
    master = scriptutil.get_master()
    node_api = get_api(master,node_name)
    if not node_api:
        print >> sys.stderr, "cannot ping [%s]: unknown node"%node_name
        return

    timeout = 5.
    
    print "pinging %s with a timeout of %ss"%(node_name, timeout)
    socket.setdefaulttimeout(timeout)
    node = xmlrpclib.ServerProxy(node_api)
    import time
    lastcall = 0.
    count = 0
    acc = 0.
    while not rospy.is_shutdown():
        try:
            start = time.time()
            pid = succeed(node.getPid(ID))
            end = time.time()
            
            dur = (end-start)*1000.
            acc += dur
            count += 1
            
            print "xmlrpc reply from %s\ttime=%fms"%(node_api, dur)
            # 1s between pings
        except socket.error:
            print >> sys.stderr, "connection failed"
        if max_count and count >= max_count:
            break
        time.sleep(1.0)
            
    if count > 1:
        print "ping average: %fms"%(acc/count)

def rosnode_ping_all():
    master = scriptutil.get_master()
    state = succeed(master.getSystemState(ID))

    nodes = []
    import itertools
    for s in state:
        for t, l in s:
            nodes.extend(l)
    for node in nodes:
        rosnode_ping(node, max_count=1)
    
def rosnode_debugnode(node_name):
    def topic_type(t, pub_topics):
        matches = [t_type for t_name, t_type in pub_topics if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    master = scriptutil.get_master()

    # go through the master system state first
    state = succeed(master.getSystemState(ID))
    pub_topics = succeed(scriptutil.get_master().getPublishedTopics(ID, '/')) 
    pubs = [t for t, l in state[0] if node_name in l]
    subs = [t for t, l in state[1] if node_name in l]
    srvs = [t for t, l in state[2] if node_name in l]  

    print '-'*80
    print "Node [%s]"%node_name
    if pubs:
        print "\nPublications: "
        print '\n'.join([" * %s [%s]"%(l, topic_type(l, pub_topics)) for l in pubs])
    else:
        print "\nPublications: None"
    if subs:
        print "\nSubscriptions: "
        print '\n'.join([" * %s [%s]"%(l, topic_type(l, pub_topics)) for l in subs])
    else:
        print "\nSubscriptions: None"        
    if srvs:
        print "\nServices: "
        print '\n'.join([" * %s"%l for l in srvs])
    else:
        print "\nServices: None"
        
    node_api = get_api(master, node_name)
    if not node_api:
        print >> sys.stderr, "cannot contact [%s]: unknown node"%node_name
        return
    
    print "\ncontacting node %s ..."%node_api

    #turn down timeout on socket library
    socket.setdefaulttimeout(5.0)
    node = xmlrpclib.ServerProxy(node_api)

    pid = succeed(node.getPid(ID))
    print "Pid: %s"%pid
    #master_uri = succeed(node.getMasterUri(ID))
    businfo = succeed(node.getBusInfo(ID))
    if businfo:
        print "Connections:"
        for info in businfo:
            dest_id   = info[1]
            direction = info[2]
            transport = info[3]
            topic     = info[4]
            if len(info) > 5:
                connected = info[5]
            else:
                connected = True #backwards compatibility

            if connected:
                print " * topic: %s"%topic

                # older ros publisher implementations don't report a URI
                print "    * to: %s"%dest_id
                if direction == 'i':
                    print "    * direction: inbound"
                elif direction == 'o':
                    print "    * direction: outbound"
                else:
                    print "    * direction: unknown"
                print "    * transport: %s"%transport
    print

def rosnode_cmd_list():
    args = sys.argv[2:]
    parser = OptionParser(usage="usage: %prog list", prog=NAME)
    parser.add_option("-u",
                      dest="list_uri", default=False,
                      action="store_true",
                      help="list XML-RPC URIs")
    parser.add_option("-a",
                      dest="list_all", default=False,
                      action="store_true",
                      help="list all information")
    (options, args) = parser.parse_args(args)
    if args:
        parser.error("invalid arguments '%s'"%(' '.join(args)))
    rosnode_listnodes(list_uri=options.list_uri, list_all=options.list_all)

def rosnode_cmd_info():
    args = sys.argv[2:]
    parser = OptionParser(usage="usage: %prog info node1 [node2...]", prog=NAME)
    (options, args) = parser.parse_args(args)
    for node in args:
        rosnode_debugnode(node)

def rosnode_cmd_ping():
    args = sys.argv[2:]    
    parser = OptionParser(usage="usage: %prog ping [options] <node>", prog=NAME)
    parser.add_option("--all",
                      dest="ping_all", default=False,
                      action="store_true",
                      help="ping all nodes")
    (options, args) = parser.parse_args(args)    
    node_name = None
    if not options.ping_all:
        if not args:
            parser.error("Please enter a node to ping. Available nodes are:\n"+_sub_rosnode_listnodes())
        elif len(args) > 1:
            parser.error("you may only specify one input node")
        elif len(args) == 1:
            node_name = scriptutil.script_resolve_name('rosnode', args[0])
            print "rosnode: node is [%s]"%node_name
    else:
        if args:
            parser.error("Invalid arguments '%s' used with --all"%(' '.join(args)))

    if options.ping_all:
        rosnode_ping_all()
    else:
        rosnode_ping(node_name)        
    
def fullusage():
    print """Commands:
\trosnode ping\ttest connectivity to node
\trosnode list\tlist active nodes
\trosnode info\tprint information about node

Type rosnode <command> -h for more detailed usage, e.g. 'rosnode ping -h'
"""
    sys.exit(os.EX_USAGE)

def rosnodemain():
    if len(sys.argv) == 1:
        fullusage()
    try:
        command = sys.argv[1]
        if command == 'ping':
            rosnode_cmd_ping()
        elif command == 'list':
            rosnode_cmd_list()
        elif command == 'info':
            rosnode_cmd_info()
        else:
            fullusage()
    except socket.error:
        print >> sys.stderr, "Network communication failed. Most likely failed to communicate with master."
    except RosNodeException, e:
        print >> sys.stderr, str(e)

        
        
