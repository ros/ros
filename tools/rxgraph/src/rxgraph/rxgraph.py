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
# Revision $Id: rxgraph.py 8782 2010-03-22 21:44:43Z kwc $

from __future__ import with_statement

import roslib; roslib.load_manifest('rxgraph')

import sys
import time
import threading
import traceback
import logging
from optparse import OptionParser

import rosgraph.graph
import rxgraph.dotcode
from rxgraph.dotcode import generate_dotcode, generate_namespaces, NODE_NODE_GRAPH, NODE_TOPIC_GRAPH
from rxgraph.viewer import RxGraphViewerFrame

import roslib.roslogging

# have to import later than others due to xdot calling wxversion
import wx

class DotUpdate(threading.Thread):
    """Thread to control update of dot file"""

    def __init__(self, graph, viewer, output_file=None):
        threading.Thread.__init__(self, name="DotUpdate")
        self.viewer = viewer
        self.graph = graph
        self.output_file = None
    
    def run(self):
        viewer = self.viewer
        current_ns_filter = viewer.ns_filter
        g = self.graph
        output_file = self.output_file
        last_graph_mode = NODE_NODE_GRAPH
        quiet = False

        orientation = rxgraph.dotcode.ORIENTATIONS[0]

        g.set_master_stale(5.0)
        g.set_node_stale(5.0)

        GUPDATE_INTERVAL = 0.5 
        last_gupdate = time.time() - GUPDATE_INTERVAL
        try:
            while not is_shutdown():

                # throttle calls to g.update(). we want fast refresh
                # on changes to the viewer's ns_filter, less so on the
                # graph polling.
                now = time.time()
                if now - last_gupdate >= GUPDATE_INTERVAL:
                    changed = g.update()
                    last_gupdate = now
                    
                graph_mode = NODE_TOPIC_GRAPH if viewer.topic_boxes else NODE_NODE_GRAPH

                changed |= viewer.ns_filter != current_ns_filter
                changed |= quiet != viewer.quiet
                changed |= graph_mode != last_graph_mode
                
                quiet = viewer.quiet
                last_graph_mode = graph_mode

                if changed:
                    current_ns_filter = viewer.ns_filter
                    dotcode = generate_dotcode(g, current_ns_filter, graph_mode, orientation, quiet)

                    #compute path-combo box
                    namespaces = generate_namespaces(g, graph_mode, quiet)
                    viewer.update_namespaces(namespaces)
                    
                    viewer.set_dotcode(dotcode)

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
    
def rxgraph_main():
    parser = OptionParser(usage="usage: rxgraph [options]")
    parser.add_option("-o", "--dot",
                      dest="output_file", default=None,
                      help="ouput graph as graphviz dot file", metavar="DOTFILE")
    parser.add_option("--nodens",
                      dest="node_ns", default=None,
                      help="only show nodes in specified namespace")
    parser.add_option("--topicns",
                      dest="topic_ns", default=None,
                      help="only show topics in specified namespace")
    
    options, args = parser.parse_args()
    if args:
        parser.error("invalid arguments")


    import subprocess
    try:
        subprocess.check_call(['dot', '-V'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except:
        print >> sys.stderr, "Graphviz does not appear to be installed on your system. Please run:\n\n\trosdep install rosgraph\n\nto install the necessary dependencies on your system"
        sys.exit(1)

        
    roslib.roslogging.configure_logging('rxgraph', logging.DEBUG, additional=['rospy', 'roslib', 'rosgraph'])
    try:
        # make gtk play nice with Python threads
        #gtk.gdk.threads_init()

        graph = rosgraph.graph.Graph(options.node_ns, options.topic_ns) 

        app = wx.App()    
        frame = RxGraphViewerFrame()
        frame.set_dotcode(rxgraph.dotcode.INIT_DOTCODE)

        DotUpdate(graph, frame, output_file=options.output_file).start()
        
        frame.Show()
        app.MainLoop()

    except KeyboardInterrupt:
        pass
    finally:
        set_shutdown(True)
        

if __name__ == '__main__':
    rxgraph_main()
