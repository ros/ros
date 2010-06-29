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

import rosgraph.impl.graph
import rxgraph.dotcode
from rxgraph.dotcode import generate_dotcode, generate_namespaces, NODE_NODE_GRAPH, NODE_TOPIC_GRAPH
from rxgraph.viewer import RxGraphViewerFrame

import roslib.scriptutil
import rostopic
import rosnode

# have to import wx last due to xdot playing with the wx version
import wx

def get_info_text(selection_url):
    if selection_url is None:
        return ''
    if selection_url.startswith('node:'):
        try:
            node_name = selection_url[5:]
            master = roslib.scriptutil.get_master()
            node_api = rosnode.get_api_uri(master, node_name)
            return rosnode.get_node_info_description(node_name) + rosnode.get_node_connection_info_description(node_api)
        except rosnode.ROSNodeException, e:
            return "ERROR: %s"%str(e)

    elif selection_url.startswith('topic:'):
        try:
            return rostopic.get_info_text(selection_url[6:])
        except rostopic.ROSTopicException, e:
            return "ERROR: %s"%str(e)
    
class DotUpdate(threading.Thread):
    """Thread to control update of dot file"""

    def __init__(self, graph, viewer, output_file=None):
        threading.Thread.__init__(self, name="DotUpdate")
        self.viewer = viewer
        self.graph = graph
        self.output_file = output_file
        self.selection_url = None
        self.selection_update = False

    def select_callback(self, target, event):
        try:
            url = target.url
            if url:
                self.selection_url = url
                self.selection_update = True
        except:
            pass

    def run(self):
        viewer = self.viewer
        current_ns_filter = viewer.ns_filter
        g = self.graph
        output_file = self.output_file
        last_graph_mode = NODE_NODE_GRAPH
        quiet = False

        orientation = rxgraph.dotcode.ORIENTATIONS[0]
        info_text = ''

        g.set_master_stale(5.0)
        g.set_node_stale(5.0)

        GUPDATE_INTERVAL = 0.5 
        INFO_UPDATE_INTERVAL = 10.
        
        last_gupdate = time.time() - GUPDATE_INTERVAL
        last_info_update = time.time() - GUPDATE_INTERVAL   
        try:
            while not is_shutdown():

                # #2839 by default, changes zoom, but we can cancel this for certain events
                zoom = True

                # throttle calls to g.update(). we want fast refresh
                # on changes to the viewer's ns_filter, less so on the
                # graph polling.
                now = time.time()
                if now - last_gupdate >= GUPDATE_INTERVAL:
                    changed = g.update()
                    last_gupdate = now

                if now - last_info_update >= INFO_UPDATE_INTERVAL or self.selection_update:
                    last_info_update = now
                    if self.selection_url is not None:
                        info_text = get_info_text(self.selection_url)
                    
                graph_mode = NODE_TOPIC_GRAPH if viewer.topic_boxes else NODE_NODE_GRAPH

                changed |= viewer.ns_filter != current_ns_filter
                changed |= quiet != viewer.quiet
                changed |= graph_mode != last_graph_mode
                if self.selection_update:
                    self.selection_update = False
                    # only alter the zoom flag if this is the sole change reason
                    if not changed:
                        changed = True
                        zoom = False
                
                quiet = viewer.quiet
                last_graph_mode = graph_mode

                if changed:
                    current_ns_filter = viewer.ns_filter
                    dotcode = generate_dotcode(g, current_ns_filter, graph_mode, orientation, quiet)

                    #compute path-combo box
                    namespaces = generate_namespaces(g, graph_mode, quiet)
                    viewer.update_namespaces(namespaces)
                    
                    viewer.set_dotcode(dotcode, zoom=zoom)
                    viewer.set_info_text(info_text)

                    # store dotcode if requested
                    if output_file:
                        with file(output_file, 'w') as f:
                            f.write(dotcode)

                    #information still changing, shorter yield
                    time.sleep(0.1)
                else:
                    # no new information, let bad nodes update
                    g.bad_update()
                    time.sleep(0.1)

                changed = False
                    
        except wx.PyDeadObjectError:
            # shutdown
            pass
        except:
            traceback.print_exc()
        
_is_shutdown = False
def is_shutdown():
    return _is_shutdown

def set_shutdown(is_shutdown):
    global _is_shutdown 
    _is_shutdown = is_shutdown
    
def init_frame():
    frame = RxGraphViewerFrame()
    frame.set_dotcode(rxgraph.dotcode.INIT_DOTCODE)
    return frame

def init_updater(frame, node_ns=None, topic_ns=None, output_file=None):
    graph = rosgraph.impl.graph.Graph(node_ns, topic_ns) 
    updater = DotUpdate(graph, frame, output_file=output_file)
    frame.register_select_cb(updater.select_callback)
    return updater
