# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

from __future__ import with_statement

import os
import itertools
import socket
import sys
import time
import xmlrpclib

import roslib.rosenv
import roslib.network
import roslib.scriptutil
import rosnode
import rosservice

from roswtf.context import WtfException
from roswtf.environment import paths, is_executable
from roswtf.model import WtfWarning, WtfError
from roswtf.rules import warning_rule, error_rule

def _businfo(ctx, node, bus_info):
    # [[connectionId1, destinationId1, direction1, transport1, ...]... ]
    edges = []
    for info in bus_info:
        #connection_id = info[0]
        dest_id       = info[1]
        if dest_id.startswith('http://'):
            if dest_id in ctx.uri_node_map:
                dest_id = ctx.uri_node_map[dest_id]
            else:
                dest_id = 'unknown (%s)'%dest_id
        direction     = info[2]
        #transport     = info[3]
        topic         = info[4]
        if len(info) > 5:
            connected = info[5]
        else:
            connected = True #backwards compatibility

        if connected:
            if direction == 'i':
                edges.append((topic, dest_id, node))
            elif direction == 'o':
                edges.append((topic, node, dest_id))
            elif direction == 'b':
                print >> sys.stderr, "cannot handle bidirectional edges"
            else:
                raise Exception()

    return edges

def unexpected_edges(ctx):
    if not ctx.system_state or not ctx.nodes:
        return
    unexpected = set(ctx.actual_edges) - set(ctx.expected_edges)
    return ["%s->%s (%s)"%(p, s, t) for (t, p, s) in unexpected]

def missing_edges(ctx):
    if not ctx.system_state or not ctx.nodes:
        return
    missing = set(ctx.expected_edges) - set(ctx.actual_edges)
    return ["%s->%s (%s)"%(p, s, t) for (t, p, s) in missing]
    
def ping_check(ctx):
    if not ctx.system_state or not ctx.nodes:
        return
    _, unpinged = rosnode.rosnode_ping_all()
    return unpinged

def simtime_check(ctx):
    if ctx.use_sim_time:
        master = roslib.scriptutil.get_master()
        code, msg, pubtopics = master.getPublishedTopics('/roswtf', '/')
        if code != 1:
            raise WtfException("cannot get published topics from master: %s"%msg)
        for topic, _ in pubtopics:
            if topic in ['/time', '/clock']:
                return
        return True
    
## contact each service and make sure it returns a header
def probe_all_services(ctx):
    master = roslib.scriptutil.get_master()
    errors = []
    for service_name in ctx.services:
        code, msg, service_uri = master.lookupService('/rosservice', service_name)
        if code != 1:
            ctx.warnings.append(WtfWarning("Unable to lookup service [%s]"%service_name))
        else:
            try:
                headers = rosservice.get_service_headers(service_name, service_uri)
                if not headers:
                    errors.append("service [%s] did not return service headers"%service_name)
            except roslib.network.ROSHandshakeException, e:
                errors.append("service [%s] appears to be malfunctioning"%service_name)
            except Exception, e:
                errors.append("service [%s] appears to be malfunctioning: %s"%(service_name, e))
    return errors
                
def unconnected_subscriptions(ctx):
    ret = ''
    whitelist = ['/reset_time']
    if ctx.use_sim_time:
        for sub, l in ctx.unconnected_subscriptions.iteritems():
            l = [t for t in l if t not in whitelist]
            if l:
                ret += ' * %s:\n'%sub
                ret += ''.join(["   * %s\n"%t for t in l])
    else:
        for sub, l in ctx.unconnected_subscriptions.iteritems():
            l = [t for t in l if t not in ['/time', '/clock']]
            if l:
                ret += ' * %s:\n'%sub
                ret += ''.join(["   * %s\n"%t for t in l])
    return ret

graph_warnings = [
    (unconnected_subscriptions, "The following node subscriptions are unconnected:\n"),
    (unexpected_edges, "The following nodes are unexpectedly connected:"),
    ]

graph_errors = [
    (simtime_check, "/use_simtime is set but no publisher of /time is present"),
    (ping_check, "Could not contact the following nodes:"),
    (missing_edges, "The following nodes should be connected but aren't:"),
    (probe_all_services, "Errors connecting to the following services:"),
    ]

def topic_timestamp_drift(ctx, t):
    #TODO: get msg_class, if msg_class has header, receive a message
    # and compare its time to ros time
    if 0:
        rospy.Subscriber(t, msg_class)

#TODO: these are mainly future enhancements. It's unclear to me whether or not this will be
#useful as most of the generic rules are capable of targetting these problems as well.
#The only rule that in particular seems useful is the timestamp drift. It may be too
#expensive otherwise to run, though it would be interesting to attempt to receive a
#message from every single topic.
        
#TODO: parameter audit?
service_errors = [
    ]
service_warnings = [
    ]
topic_errors = [
    (topic_timestamp_drift, "Timestamp drift:")
    ]
topic_warnings = [
    ]
node_errors = [
    ]
node_warnings = [
    ]

## cache sim_time calculation sot that multiple rules can use
def _compute_sim_time(ctx):
    param_server = roslib.scriptutil.get_param_server()
    code, msg, val = simtime = param_server.getParam('/roswtf', '/use_sim_time')
    if code == 1 and val:
        ctx.use_sim_time = True
    else:
        ctx.use_sim_time = False        
    
def _compute_system_state(ctx):
    socket.setdefaulttimeout(3.0)
    master = roslib.scriptutil.get_master()

    # store system state
    code, msg, val = master.getSystemState('/roswtf')
    if code != 1:
        return
    ctx.system_state = val

    pubs, subs, srvs = val
    
    # compute list of topics and services
    topics = []
    for t, _ in itertools.chain(pubs, subs):
        topics.append(t)
    services = []
    service_providers = []
    for s, l in srvs:
        services.append(s)
        service_providers.extend(l)
    ctx.topics = topics
    ctx.services = services
    ctx.service_providers = service_providers

    # compute list of nodes
    nodes = []
    for s in val:
        for t, l in s:
            nodes.extend(l)
    ctx.nodes = list(set(nodes)) #uniq

    # - compute reverse mapping of URI->nodename
    count = 0
    start = time.time()
    for n in ctx.nodes:
        count += 1
        try:
            code, msg, val = master.lookupNode('/roswtf', n)
        except socket.error:
            ctx.errors.append(WtfError("cannot contact ROS Master at %s"%roslib.rosenv.get_master_uri()))
            raise WtfException("roswtf lost connection to the ROS Master at %s"%roslib.rosenv.get_master_uri())
        if code == 1:
            ctx.uri_node_map[val] = n
        else:
            ctx.warnings.append(WtfWarning("Inconsistent state on master for node [%s]"%n))
    end = time.time()
    # - time thresholds currently very arbitrary
    if count:
        if ((end - start) / count) > 1.:
            ctx.warnings.append(WtfError("Communication with master is very slow (>1s average)"))        
        elif (end - start) / count > .5:
            ctx.warnings.append(WtfWarning("Communication with master is very slow (>0.5s average)")) 

import threading
class NodeInfoThread(threading.Thread):
    def __init__(self, n, ctx, master, actual_edges, lock):
        threading.Thread.__init__(self)
        self.master = master
        self.actual_edges = actual_edges
        self.lock = lock
        self.n = n
        self.done = False
        self.ctx = ctx

    def run(self):
        ctx = self.ctx
        master = self.master
        actual_edges = self.actual_edges
        lock = self.lock
        n = self.n

        try:
            socket.setdefaulttimeout(3.0)
            node_api = rosnode.get_api_uri(master, n)
            if not node_api:
                ctx.errors.append(WtfError("Master does not have lookup information for node [%s]"%n))
                return
                
            node = xmlrpclib.ServerProxy(node_api)
            start = time.time()
            socket.setdefaulttimeout(3.0)            
            code, msg, bus_info = node.getBusInfo('/roswtf')
            end = time.time()
            with lock:
                if (end-start) > 1.:
                    ctx.warnings.append(WtfWarning("Communication with node [%s] is very slow"%n))
                if code != 1:
                    ctx.warnings.append(WtfWarning("Node [%s] would not return bus info"%n))
                elif not bus_info:
                    if not n in ctx.service_providers:
                        ctx.warnings.append(WtfWarning("Node [%s] is not connected to anything"%n))
                else:
                    edges = _businfo(ctx, n, bus_info)
                    actual_edges.extend(edges)
        except socket.error, e:
            pass #ignore as we have rules to catch this
        except Exception, e:
            ctx.errors.append(WtfError("Communication with [%s] raised an error: %s"%(n, str(e))))
        finally:
            self.done = True
        
        
## retrieve graph state from master and related nodes once so we don't overload
## the network
def _compute_connectivity(ctx):
    socket.setdefaulttimeout(3.0)
    master = roslib.scriptutil.get_master()

    # Compute list of expected edges and unconnected subscriptions
    pubs, subs, _ = ctx.system_state
    expected_edges = [] # [(topic, publisher, subscriber),]
    unconnected_subscriptions = {} # { subscriber : [topics] }

    # - build up a dictionary of publishers keyed by topic
    pub_dict = {}
    for t, pub_list in pubs:
        pub_dict[t] = pub_list
    # - iterate through subscribers and add edge to each publisher of topic
    for t, sub_list in subs:
        for sub in sub_list:
            if t in pub_dict:
                expected_edges.extend([(t, pub, sub) for pub in pub_dict[t]])
            elif sub in unconnected_subscriptions:
                unconnected_subscriptions[sub].append(t)
            else:
                unconnected_subscriptions[sub] = [t]
                    
    # compute actual edges
    actual_edges = []
    lock = threading.Lock()
    threads = []
    for n in ctx.nodes:
        t =NodeInfoThread(n, ctx, master, actual_edges, lock)
        threads.append(t)
        t.start()
        
    # spend up to a minute waiting for threads to complete. each
    # thread has a 3-second timeout, but this will spike load
    timeout_t = time.time() + 60.0
    while time.time() < timeout_t and [t for t in threads if not t.done]:
        time.sleep(0.5)
    
    ctx.expected_edges = expected_edges
    ctx.actual_edges = actual_edges
    ctx.unconnected_subscriptions = unconnected_subscriptions
            
def _compute_online_context(ctx):
    # have to compute sim time first
    _compute_sim_time(ctx)
    _compute_system_state(ctx)
    _compute_connectivity(ctx)
    
def wtf_check_graph(ctx, names=None):
    master_uri = ctx.ros_master_uri
    #TODO: master rules
    # - check for stale master state
    
    # TODO: get the type for each topic from each publisher and see if they match up

    master = roslib.scriptutil.get_master()
    try:
        master.getPid('/roswtf')
    except:
        warning_rule((True, "Cannot communicate with master, ignoring online checks"), True, ctx)
        return
            
    # fill in ctx info so we only have to compute once
    print "analyzing graph..."
    _compute_online_context(ctx)
    print "... done analyzing graph"
    
    if names:
        check_topics = [t for t in names if t in ctx.topics]
        check_services = [t for t in names if t in ctx.services]
        check_nodes = [t for t in names if t in ctx.nodes]
        unknown = [t for t in names if t not in check_topics + check_services + check_nodes]
        if unknown:
            raise WtfException("The following names were not found in the list of nodes, topics, or services:\n%s"%(''.join([" * %s\n"%t for t in unknown])))

        for t in check_topics:
            for r in topic_warnings:
                warning_rule(r, r[0](ctx, t), ctx)            
            for r in topic_errors:
                error_rule(r, r[0](ctx, t), ctx)            
        for s in check_services:
            for r in service_warnings:
                warning_rule(r, r[0](ctx, s), ctx)            
            for r in service_errors:
                error_rule(r, r[0](ctx, s), ctx)            
        for n in check_nodes:
            for r in node_warnings:
                warning_rule(r, r[0](ctx, n), ctx)            
            for r in node_errors:
                error_rule(r, r[0](ctx, n), ctx)            


    print "running graph rules..."
    for r in graph_warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in graph_errors:
        error_rule(r, r[0](ctx), ctx)
    print "... done running graph rules"        
