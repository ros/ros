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

import sys
import time
import threading
import traceback
import logging

import gobject
import gtk
import gtk.gdk

import xdot
import rosstats

import roslib.roslogging
roslib.roslogging.configure_logging('rosviz', logging.DEBUG, additional=['rospy', 'roslib'])


orientations = ['LR', 'TB', 'RL', 'BT']

# node/node connectivity
NODE_NODE_GRAPH = "node_node"
# node/topic connections where an actual network connection exists
NODE_TOPIC_GRAPH = "node_topic"
# all node/topic connections, even if no actual network connection
NODE_TOPIC_ALL_GRAPH = "node_topic_all" 
class RosDotWindow(xdot.DotWindow):
    
    def __init__(self):
        xdot.DotWindow.__init__(self, (1400, 512)) 
        self.dotcode = self.new_dotcode = None
        # 1hz update loop
        self.timeout_id = gobject.timeout_add(1000, self._update_dot_display, )
        # set the rotation handler
        self.widget.set_on_rotate(self.on_rotate)

        self.orientation = orientations[0]
        self.dirty = True
        self.graph_mode = NODE_TOPIC_ALL_GRAPH

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
        new_dotcode = generate_dotcode(self.graph_mode)
        if 0:
            print "---------------------------"
            print "NEXT", next_dotcode
            print "---------------------------"
        
        if new_dotcode:
            self.set_dotcode(new_dotcode%self.orientation)
            self.dotcode = self.new_dotcode
        return True # keep going

_rosstats = rosstats.RosStats() #singleton

## singleton accessor for ROS stats
def get_rosstats():
    return _rosstats

## encode the name for dotcode symbol-safe syntax
def safe_dotcode_name(name):
    # not terribly efficient or sophisticated 
    ret = name.replace('/', '_')
    ret = ret.replace(' ', '_')
    ret = ret.replace('-', '_')
    ret = ret.replace('.', '_')            
    return ret

def _edge_to_dot(e):
    return "    %s->%s"%(safe_dotcode_name(e.start), safe_dotcode_name(e.end))

def _generate_node_dotcode(node, stats):
    if node in stats.bad_nodes:
        bn = stats.bad_nodes[node]
        if bn.type == rosstats.BadNode.DEAD:
            return '  %s [color="red", shape="doublecircle", label="%s"];'%(
                safe_dotcode_name(node), node)
        else:
            return '  %s [color="orange", shape="doublecircle", label="%s"];'%(
                safe_dotcode_name(node), node)
    else:
        return '  %s [label="%s"];'%(safe_dotcode_name(node), node)
    
## @param graph_mode str: NODE_NODE_GRAPH | NODE_TOPIC_GRAPH | NODE_TOPIC_ALL_GRAPH
## @return str: dotcode generated from rosstats singleton
def generate_dotcode(graph_mode):
    print "generate_dotcode", graph_mode
    stats = get_rosstats()
    
    # create the node definitions
    if graph_mode == NODE_NODE_GRAPH:
        nodes = stats.nn_nodes        
        if nodes:
            nodes_str = '\n'.join([_generate_node_dotcode(n, stats) for n in nodes])
        else:
            nodes_str = '  empty;'
    elif graph_mode == NODE_TOPIC_GRAPH or \
             graph_mode == NODE_TOPIC_ALL_GRAPH:
        nn_nodes = stats.nn_nodes
        nt_nodes = stats.nt_nodes        
        if nn_nodes or nt_nodes:
            nodes_str = '\n'.join([_generate_node_dotcode(n, stats) for n in nn_nodes])
            nodes_str += '\n'.join(['  %s [shape=box,label="%s"];'%(
                safe_dotcode_name(n), rosstats.node_topic(n)) for n in nt_nodes]) 
        else:
            nodes_str = '  empty;'

    # create the edge definitions
    if graph_mode == NODE_NODE_GRAPH:
        edges = stats.nn_edges
    elif graph_mode == NODE_TOPIC_GRAPH:
        edges = stats.nt_edges
    else:
        edges = stats.nt_all_edges 
    edges_str = '\n'.join([_edge_to_dot(e) for e in edges])
    return "digraph G {\n  rankdir=%%s;\n%(nodes_str)s\n%(edges_str)s}\n"%vars()

class DotUpdate(threading.Thread):
    def __init__(self, callback):
        threading.Thread.__init__(self, name="DotUpdate")
        self.callback = callback
    
    def run(self):
        callback = self.callback
        stats = get_rosstats()
        stats.set_master_stale(5.0)
        stats.set_node_stale(5.0)

        try:
            while not is_shutdown():
                if stats.update():
                    callback()
                    #information still changing, smaller yield
                    time.sleep(0.5)
                else:
                    # no new information, let bad nodes update
                    stats.bad_update()
                    time.sleep(0.5)                    
        except:
            traceback.print_exc()
        
init_dotcode = """
digraph G { initializing [label="initializing..."]; }
"""

_is_shutdown = False
def is_shutdown():
    return _is_shutdown

def set_shutdown(is_shutdown):
    global _is_shutdown 
    _is_shutdown = is_shutdown
    
def main():
    try:
        try:
            # make gtk play nice with Python threads
            gtk.gdk.threads_init()     
            window = RosDotWindow()
            window.set_dotcode(init_dotcode)
            window.connect('destroy', gtk.main_quit)

            # Two threads of execution: the GTK stuff and our ROS-polling thread
            DotUpdate(window.repaint_graph).start()
            gtk.main()

        except KeyboardInterrupt:
            pass
    finally:
        set_shutdown(True)
        

if __name__ == '__main__':
    main()
    
