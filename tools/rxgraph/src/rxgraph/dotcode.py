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

import roslib.names
import rosgraph.impl.graph

ORIENTATIONS = ['LR', 'TB', 'RL', 'BT']

INIT_DOTCODE = """
digraph G { initializing [label="initializing..."]; }
"""

# node/node connectivity
NODE_NODE_GRAPH = "node_node"
# node/topic connections where an actual network connection exists
NODE_TOPIC_GRAPH = "node_topic"
# all node/topic connections, even if no actual network connection
NODE_TOPIC_ALL_GRAPH = "node_topic_all" 

def safe_dotcode_name(name):
    """
    encode the name for dotcode symbol-safe syntax    
    """
    # not terribly efficient or sophisticated 
    ret = name.replace('/', '_')
    ret = ret.replace(' ', '_')
    ret = ret.replace('-', '_')
    return ret.replace('.', '_')            

def _edge_to_dot(e):
    if e.label:
        return '    %s->%s [label="%s"]'%(safe_dotcode_name(e.start), safe_dotcode_name(e.end), e.label)
    else:
        return '    %s->%s'%(safe_dotcode_name(e.start), safe_dotcode_name(e.end))

def _generate_node_dotcode(node, g, quiet):
    if node in g.bad_nodes:
        bn = g.bad_nodes[node]
        if bn.type == rosgraph.impl.graph.BadNode.DEAD:
            return '  %s [color="red", shape="doublecircle", label="%s", URL="node:%s"];'%(
                safe_dotcode_name(node), node, node)
        else:
            return '  %s [color="orange", shape="doublecircle", label="%s", URL="node:%s"];'%(
                safe_dotcode_name(node), node, node, node)
    else:
        return '  %s [label="%s", URL="node:%s"];'%(safe_dotcode_name(node), node, node)
    
QUIET_NAMES = ['/diag_agg', '/runtime_logger', '/pr2_dashboard', '/rviz', '/rosout', '/cpu_monitor', '/monitor','/hd_monitor', '/rxloggerlevel', '/clock']
def _quiet_filter(name):
    # ignore viewers
    for n in QUIET_NAMES:
        if n in name:
            return False
    return True

def _quiet_filter_edge(edge):
    for quiet_label in ['/time', '/clock', '/rosout']:
        if quiet_label == edge.label:
            return False
    return _quiet_filter(edge.start) and _quiet_filter(edge.end)
        
def generate_namespaces(g, graph_mode, quiet=False):
    """
    Determine the namespaces of the nodes being displayed
    """
    namespaces = []
    if graph_mode == NODE_NODE_GRAPH:
        nodes = g.nn_nodes
        if quiet:
            nodes = [n for n in nodes if not n in QUIET_NAMES]
        namespaces = list(set([roslib.names.namespace(n) for n in nodes]))
            
    elif graph_mode == NODE_TOPIC_GRAPH or \
             graph_mode == NODE_TOPIC_ALL_GRAPH:
        nn_nodes = g.nn_nodes
        nt_nodes = g.nt_nodes        
        if quiet:
            nn_nodes = [n for n in nn_nodes if not n in QUIET_NAMES]
            nt_nodes = [n for n in nt_nodes if not n in QUIET_NAMES]
        if nn_nodes or nt_nodes:
            namespaces = [roslib.names.namespace(n) for n in nn_nodes]
            # an annoyance with the rosgraph library is that it
            # prepends a space to topic names as they have to have
            # different graph node namees from nodes. we have to strip here
            namespaces.extend([roslib.names.namespace(n[1:]) for n in nt_nodes])

    return list(set(namespaces))

def _filter_edges(edges, nodes):
    # currently using and rule as the or rule generates orphan nodes with the current logic
    return [e for e in edges if e.start in nodes and e.end in nodes]

def generate_dotcode(g, ns_filter, graph_mode, orientation, quiet=False):
    """
    @param g: Graph instance
    @param ns_filter: namespace filter (must be canonicalized with trailing '/')
    @type  ns_filter: string
    @param graph_mode str: NODE_NODE_GRAPH | NODE_TOPIC_GRAPH | NODE_TOPIC_ALL_GRAPH
    @type  graph_mode: str
    @param orientation: rankdir value (see ORIENTATIONS dict)
    @return: dotcode generated from graph singleton
    @rtype: str
    """
    #print "generate_dotcode", graph_mode
    if ns_filter:
        name_filter = ns_filter[:-1]
    
    # create the node definitions
    if graph_mode == NODE_NODE_GRAPH:
        nodes = g.nn_nodes
        if quiet:
            nodes = [n for n in nodes if not n in QUIET_NAMES]
        if ns_filter and ns_filter != '/':
            nodes = [n for n in nodes if n.startswith(ns_filter) or n == name_filter]
        if nodes:
            nodes_str = '\n'.join([_generate_node_dotcode(n, g, quiet) for n in nodes])
        else:
            nodes_str = '  empty;'
            
    elif graph_mode == NODE_TOPIC_GRAPH or \
             graph_mode == NODE_TOPIC_ALL_GRAPH:
        nn_nodes = g.nn_nodes
        nt_nodes = g.nt_nodes        
        if quiet:
            nn_nodes = [n for n in nn_nodes if not n in QUIET_NAMES]
            nt_nodes = [n for n in nt_nodes if not n in QUIET_NAMES]
        if ns_filter and ns_filter != '/':
            nn_nodes = [n for n in nn_nodes if n.startswith(ns_filter) or n == name_filter]
            nt_nodes = [n for n in nt_nodes if n[1:].startswith(ns_filter) or n[1:] == name_filter]
            
        if nn_nodes or nt_nodes:
            nodes_str = '\n'.join([_generate_node_dotcode(n, g, quiet) for n in nn_nodes])
            nodes_str += '\n'.join(['  %s [shape=box,label="%s",URL="topic:%s"];'%(
                safe_dotcode_name(n), rosgraph.impl.graph.node_topic(n), rosgraph.impl.graph.node_topic(n)) for n in nt_nodes]) 
        else:
            nodes_str = '  empty;'
        nodes = list(nn_nodes) + list(nt_nodes)

    # create the edge definitions
    if graph_mode == NODE_NODE_GRAPH:
        edges = g.nn_edges
    elif graph_mode == NODE_TOPIC_GRAPH:
        edges = g.nt_edges
    else:
        edges = g.nt_all_edges
    if quiet:
        edges = filter(_quiet_filter_edge, edges)
        
    edges = _filter_edges(edges, nodes)
    edges_str = '\n'.join([_edge_to_dot(e) for e in edges])
    return "digraph G {\n  rankdir=%(orientation)s;\n%(nodes_str)s\n%(edges_str)s}\n"%vars()

