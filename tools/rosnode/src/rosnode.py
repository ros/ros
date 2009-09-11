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
import time
import xmlrpclib

from optparse import OptionParser

import roslib.scriptutil as scriptutil 
import rospy

class ROSNodeException(Exception): pass
class ROSNodeIOException(ROSNodeException): pass

def succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSNodeException("remote call failed: %s"%msg)
    return val
        
_caller_apis = {}
## @param master xmlrpclib.ServerProxy
## @param caller_id str: node name
## @return xmlrpc URI of \a caller_id
## @throws ROSNodeIOException
def get_api_uri(master, caller_id):
    caller_api = _caller_apis.get(caller_id, None)
    if not caller_api:
        try:
            code, msg, caller_api = master.lookupNode(ID, caller_id)
            if code != 1:
                return None
            else:
                _caller_apis[caller_id] = caller_api
        except socket.error:
            raise ROSNodeIOException("Unable to communicate with master!")
    return caller_api

## @return [str]: list of node caller IDs
## @throws ROSNodeIOException
def get_node_names():
    master = scriptutil.get_master()
    try:
        state = succeed(master.getSystemState(ID))
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")
    nodes = []
    import itertools
    for s in state:
        for t, l in s:
            nodes.extend(l)

    return list(set(nodes))
    
## @return str: string list of all nodes
def _sub_rosnode_listnodes(list_uri=False, list_all=False):
    master = scriptutil.get_master()    
    nodes = get_node_names()
    #print '-'*80
    #print "Nodes:"
    if list_all:
        return '\n'.join(["%s \t%s"%(get_api_uri(master, n) or 'unknown address', n) for n in nodes])
    elif list_uri:
        return '\n'.join([(get_api_uri(master, n) or 'unknown address') for n in nodes])
    else:
        return '\n'.join(nodes)
    
## print list of all ROS nodes
def rosnode_listnodes(list_uri=False, list_all=False):
    print _sub_rosnode_listnodes(list_uri=list_uri, list_all=list_all)
    
## Test connectivity to node by calling its XMLRPC API
## @param node_name str: name of node to ping
## @param verbose bool: print ping information to screen
## @return True if node pinged
def rosnode_ping(node_name, max_count=None, verbose=False):
    master = scriptutil.get_master()
    node_api = get_api_uri(master,node_name)
    if not node_api:
        print >> sys.stderr, "cannot ping [%s]: unknown node"%node_name
        return False

    timeout = 3.

    if verbose:
        print "pinging %s with a timeout of %ss"%(node_name, timeout)
    socket.setdefaulttimeout(timeout)
    node = xmlrpclib.ServerProxy(node_api)
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
            
            if verbose:
                print "xmlrpc reply from %s\ttime=%fms"%(node_api, dur)
            # 1s between pings
        except socket.error:
            print >> sys.stderr, "connection to [%s] timed out"%node_name
            return False
        if max_count and count >= max_count:
            break
        time.sleep(1.0)
            
    if verbose and count > 1:
        print "ping average: %fms"%(acc/count)
    return True

## Ping all runnig nodes
## @return [str], [str]: pinged nodes, un-pingable nodes
## @throws ROSNodeIOException
def rosnode_ping_all(verbose=False):
    master = scriptutil.get_master()
    try:
        state = succeed(master.getSystemState(ID))
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")

    nodes = []
    import itertools
    for s in state:
        for t, l in s:
            nodes.extend(l)
    nodes = list(set(nodes)) #uniq
    if verbose:
        print "Will ping the following nodes: \n"+''.join([" * %s\n"%n for n in nodes])
    pinged = []
    unpinged = []
    for node in nodes:
        if rosnode_ping(node, max_count=1, verbose=verbose):
            pinged.append(node)
        else:
            unpinged.append(node)
    return pinged, unpinged
    
## @param master xmlrpclib.ServerProxy
## @param blacklist [str]: list of nodes to scrub
def cleanup_master_blacklist(master, blacklist):
    pubs, subs, srvs = succeed(master.getSystemState(ID))
    for n in blacklist:
        print "Unregistering", n
        node_api = get_api_uri(master, n)
        for t, l in pubs:
            if n in l:
                succeed(master.unregisterPublisher(n, t, node_api))
        for t, l in subs:
            if n in l:
                succeed(master.unregisterSubscriber(n, t, node_api))
        for s, l in srvs:
            if n in l:
                service_api = succeed(master.lookupService(ID, s))
                succeed(master.unregisterService(n, s, service_api))

## @param master xmlrpclib.ServerProxy
## @param whitelist [str]: list of nodes to keep
def cleanup_master_whitelist(master, whitelist):
    pubs, subs, srvs = succeed(master.getSystemState(ID))
    for t, l in pubs:
        for n in l:
            if n not in whitelist:
                node_api = get_api_uri(master, n)
                succeed(master.unregisterPublisher(n, t, node_api))
    for t, l in subs:
        for n in l:
            if n not in whitelist:
                node_api = get_api_uri(master, n)
                succeed(master.unregisterSubscriber(n, t, node_api))
    for s, l in srvs:
        for n in l:
            if n not in whitelist:
                service_api = succeed(master.lookupService(ID, s))
                succeed(master.unregisterService(n, s, service_api))

def rosnode_cleanup():
    pinged, unpinged = rosnode_ping_all()
    if unpinged:
        master = scriptutil.get_master()
        print "Unable to contact the following nodes:"
        print '\n'.join(' * %s'%n for n in unpinged)
        print "cleanup will purge all information about these nodes from the master"
        print "Please type y or n to continue"
        input = sys.stdin.readline()
        while not input.strip() in ['y', 'n']:
            input = sys.stdin.readline()
        if input.strip() == 'n':
            print 'aborting'
            return
        
        cleanup_master_blacklist(master, unpinged)

        print "done"

## @throws ROSNodeIOException
def rosnode_debugnode(node_name):
    def topic_type(t, pub_topics):
        matches = [t_type for t_name, t_type in pub_topics if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    master = scriptutil.get_master()
    node_name = scriptutil.script_resolve_name('rosnode', node_name)

    # go through the master system state first
    try:
        state = succeed(master.getSystemState(ID))
        pub_topics = succeed(scriptutil.get_master().getPublishedTopics(ID, '/'))
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")
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
        
    node_api = get_api_uri(master, node_name)
    if not node_api:
        print >> sys.stderr, "cannot contact [%s]: unknown node"%node_name
        return
    
    print "\ncontacting node %s ..."%node_api

    #turn down timeout on socket library
    socket.setdefaulttimeout(5.0)
    node = xmlrpclib.ServerProxy(node_api)

    try:
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
    except socket.error:
        raise ROSNodeIOException("Communication with node[%s] failed! Node address is [%s]"%(node_name, node_api))

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
    if not args:
        parser.error("You must specify at least one node name")        
    for node in args:
        rosnode_debugnode(node)

def rosnode_cmd_cleanup():
    args = sys.argv[2:]
    parser = OptionParser(usage="usage: %prog cleanup", prog=NAME)
    (options, args) = parser.parse_args(args)
    rosnode_cleanup()

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
            try:
                parser.error("Please enter a node to ping. Available nodes are:\n"+_sub_rosnode_listnodes())
            except:
                # master is offline, but user did have invalid usage, so display correct usage first
                parser.error("Please enter a node to ping")
        elif len(args) > 1:
            parser.error("you may only specify one input node")
        elif len(args) == 1:
            node_name = scriptutil.script_resolve_name('rosnode', args[0])
            print "rosnode: node is [%s]"%node_name
    else:
        if args:
            parser.error("Invalid arguments '%s' used with --all"%(' '.join(args)))

    if options.ping_all:
        rosnode_ping_all(verbose=True)
    else:
        rosnode_ping(node_name, verbose=True)        
    
def fullusage():
    print """rosnode is a command-line tool for printing information about ROS Nodes.

Commands:
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
        elif command == 'cleanup':
            rosnode_cmd_cleanup()
        else:
            fullusage()
    except socket.error:
        print >> sys.stderr, "Network communication failed. Most likely failed to communicate with master."
    except ROSNodeException, e:
        print >> sys.stderr, "ERROR: "+str(e)
    except KeyboardInterrupt:
        pass
        
        
