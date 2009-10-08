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

from __future__ import with_statement
import sys
import time
import threading
import traceback
import logging
from optparse import OptionParser

try:
    import gobject
    import gtk
    import gtk.gdk
except ImportError:
    print >> sys.stderr, "rxgraph requires GTK in order to run.\nPlease run\n\trosdep install rosgraph\nto install any missing dependencies of rxgraph."
    sys.exit(1)

import rosgraph.xdot
import rosgraph.graph

import roslib.roslogging

orientations = ['LR', 'TB', 'RL', 'BT']

# node/node connectivity
NODE_NODE_GRAPH = "node_node"
# node/topic connections where an actual network connection exists
NODE_TOPIC_GRAPH = "node_topic"
# all node/topic connections, even if no actual network connection
NODE_TOPIC_ALL_GRAPH = "node_topic_all" 
class RosDotWindow(rosgraph.xdot.DotWindow):
    
    def __init__(self, output_file=None, quiet=False, mode=NODE_NODE_GRAPH):
        rosgraph.xdot.DotWindow.__init__(self, (1400, 512)) 
        self.dotcode = self.new_dotcode = None
        # 1hz update loop
        self.timeout_id = gobject.timeout_add(1000, self._update_dot_display, )
        # set the rotation handler
        self.widget.set_on_rotate(self.on_rotate)

        self.orientation = orientations[0]
        self.dirty = True
        #self.graph_mode = NODE_TOPIC_ALL_GRAPH
        self.graph_mode = mode
        self.output_file = output_file
        self.quiet = quiet

    def on_rotate(self, action):
        next_idx = (orientations.index(self.orientation) + 1) % len(orientations)
        self.orientation = orientations[next_idx]
        self.repaint_graph()
        
    ## render the next dotcode if available
    def repaint_graph(self):
        self.dirty = True
        self._update_dot_display()
        
    ## render the next dotcode if available
    def _update_dot_display(self):
        if not self.dirty:
            # True for gobject timeout keep alive
            return True
        
        self.dirty = False
        new_dotcode = generate_dotcode(self.graph_mode, self.quiet)
        if 0:
            print "---------------------------"
            print "NEXT", next_dotcode
            print "---------------------------"
        
        if new_dotcode:
            self.set_dotcode(new_dotcode%self.orientation)
            self.dotcode = self.new_dotcode
            if self.output_file:
                with open(self.output_file, 'w') as f:
                    f.write(new_dotcode%self.orientation)
                    print "output new graph to %s"%self.output_file

        return True # keep going

_graph = rosgraph.graph.Graph() #singleton

## singleton accessor for ROS graph
def get_graph():
    return _graph

## encode the name for dotcode symbol-safe syntax
def safe_dotcode_name(name):
    # not terribly efficient or sophisticated 
    ret = name.replace('/', '_')
    ret = ret.replace(' ', '_')
    ret = ret.replace('-', '_')
    ret = ret.replace('.', '_')            
    return ret

def _edge_to_dot(e):
    if e.label:
        return '    %s->%s [label="%s"]'%(safe_dotcode_name(e.start), safe_dotcode_name(e.end), e.label)
    else:
        return '    %s->%s'%(safe_dotcode_name(e.start), safe_dotcode_name(e.end))

def _generate_node_dotcode(node, g, quiet):
    if node in g.bad_nodes:
        bn = g.bad_nodes[node]
        if bn.type == rosgraph.graph.BadNode.DEAD:
            return '  %s [color="red", shape="doublecircle", label="%s"];'%(
                safe_dotcode_name(node), node)
        else:
            return '  %s [color="orange", shape="doublecircle", label="%s"];'%(
                safe_dotcode_name(node), node)
    else:
        return '  %s [label="%s"];'%(safe_dotcode_name(node), node)
    
def _quiet_filter(name):
    # ignore viewers
    for n in ['/pr2_dashboard', '/rviz', '/rosout', '/cpu_monitor', '/monitor','/hd_monitor', '/rxloggerlevel']:
        if n in name:
            return False
    return True
def _quiet_filter_edge(edge):
    return _quiet_filter(edge.start) and _quiet_filter(edge.end)

## @param graph_mode str: NODE_NODE_GRAPH | NODE_TOPIC_GRAPH | NODE_TOPIC_ALL_GRAPH
## @return str: dotcode generated from graph singleton
def generate_dotcode(graph_mode, quiet=False):
    #print "generate_dotcode", graph_mode
    g = get_graph()
    
    # create the node definitions
    if graph_mode == NODE_NODE_GRAPH:
        nodes = g.nn_nodes
        if quiet:
            nodes = filter(_quiet_filter, nodes)
        if nodes:
            nodes_str = '\n'.join([_generate_node_dotcode(n, g, quiet) for n in nodes])
        else:
            nodes_str = '  empty;'
    elif graph_mode == NODE_TOPIC_GRAPH or \
             graph_mode == NODE_TOPIC_ALL_GRAPH:
        nn_nodes = g.nn_nodes
        nt_nodes = g.nt_nodes        
        if quiet:
            nn_nodes = filter(_quiet_filter, nn_nodes)
            nt_nodes = filter(_quiet_filter, nt_nodes)
        if nn_nodes or nt_nodes:
            nodes_str = '\n'.join([_generate_node_dotcode(n, g, quiet) for n in nn_nodes])
            nodes_str += '\n'.join(['  %s [shape=box,label="%s"];'%(
                safe_dotcode_name(n), rosgraph.graph.node_topic(n)) for n in nt_nodes]) 
        else:
            nodes_str = '  empty;'

    # create the edge definitions
    if graph_mode == NODE_NODE_GRAPH:
        edges = g.nn_edges
    elif graph_mode == NODE_TOPIC_GRAPH:
        edges = g.nt_edges
    else:
        edges = g.nt_all_edges
    if quiet:
        edges = filter(_quiet_filter_edge, edges)
    edges_str = '\n'.join([_edge_to_dot(e) for e in edges])
    return "digraph G {\n  rankdir=%%s;\n%(nodes_str)s\n%(edges_str)s}\n"%vars()

class DotUpdate(threading.Thread):
    def __init__(self, callback, quiet=False):
        threading.Thread.__init__(self, name="DotUpdate")
        self.callback = callback
        self.quiet = quiet
    
    def run(self):
        callback = self.callback
        g = get_graph()
        g.set_master_stale(5.0)
        g.set_node_stale(5.0)

        try:
            while not is_shutdown():
                if g.update():
                    callback()
                    #information still changing, smaller yield
                    time.sleep(0.5)
                else:
                    # no new information, let bad nodes update
                    g.bad_update()
                    time.sleep(0.5)                    
        except:
            traceback.print_exc()
        
_is_shutdown = False
def is_shutdown():
    return _is_shutdown

def set_shutdown(is_shutdown):
    global _is_shutdown 
    _is_shutdown = is_shutdown
    
def rxgraph_main():
    parser = OptionParser(usage="usage: rxgraph [options]")
    parser.add_option("-o", "--dot",
                      dest="output_file", default=None,
                      help="ouput graph as graphviz dot file", metavar="DOTFILE")
    parser.add_option("-q", "--quiet",
                      dest="quiet", default=False, action="store_true",
                      help="filter out common viewers")
    parser.add_option("-t", "--topics",
                      dest="topics", default=False, action="store_true",
                      help="show all topics")

    options, args = parser.parse_args()
    if args:
        parser.error("invalid arguments")

    roslib.roslogging.configure_logging('rosviz', logging.DEBUG, additional=['rospy', 'roslib'])
    init_dotcode = """
digraph G { initializing [label="initializing..."]; }
"""
    try:
        # make gtk play nice with Python threads
        gtk.gdk.threads_init()

        if options.topics:
            mode = NODE_TOPIC_ALL_GRAPH
        else:
            mode = NODE_NODE_GRAPH
        window = RosDotWindow(options.output_file, options.quiet, mode)
        window.set_dotcode(init_dotcode)
        window.connect('destroy', gtk.main_quit)

        # Two threads of execution: the GTK stuff and our ROS-polling thread
        DotUpdate(window.repaint_graph, quiet=options.quiet).start()
        gtk.main()

    except KeyboardInterrupt:
        pass
    finally:
        set_shutdown(True)
        
