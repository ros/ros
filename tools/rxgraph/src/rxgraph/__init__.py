# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

import roslib; roslib.load_manifest('rxgraph')

import rxgraph.impl
from optparse import OptionParser

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

        
    # initialize logging libraries
    import logging
    import roslib.roslogging
    roslib.roslogging.configure_logging('rxgraph', logging.INFO, additional=['rospy', 'roslib', 'rosgraph'])
    try:
        import wx
        app = wx.App()    

        frame = rxgraph.impl.init_frame()
        updater = rxgraph.impl.init_updater(frame, node_ns=options.node_ns, topic_ns=options.topic_ns, output_file=options.output_file)
        updater.start()
        
        frame.Show()
        app.MainLoop()

    except KeyboardInterrupt:
        pass
    finally:
        rxgraph.impl.set_shutdown(True)

        
if __name__ == '__main__':
    rxgraph_main()
