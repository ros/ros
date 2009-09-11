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

import time
import roslib.roslogging
import rosgraph.graph

def rosgraph_main():
    roslib.roslogging.configure_logging('rosgraph')
    g = rosgraph.graph.Graph()
    try:
        while 1:
          g.update()

          if not g.nn_nodes and not g.srvs:
              print "empty"
          else:
              print '\n'
          if g.nn_nodes:
              print 'Nodes:'
              for n in g.nn_nodes:
                  print '  ' + n + ' :'
                  print '    Inbound:'
                  if n in g.nn_edges.edges_by_end:
                      for c in g.nt_all_edges.edges_by_end[n]:
                          print '      ' + c.start
                  print '    Outbound:'
                  if n in g.nn_edges.edges_by_start:
                      for c in g.nt_all_edges.edges_by_start[n]:
                          print '      ' + c.end
          if g.srvs:
              print 'Services:'
              for s in g.srvs:
                  print '  ' + s

          time.sleep(1.0)
    except KeyboardInterrupt:
        pass

