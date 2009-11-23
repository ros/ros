#!/usr/bin/env python
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

PKG = 'rxplay'
import roslib; roslib.load_manifest(PKG)
import rospy

import optparse
import sys
import threading
import time

import wxversion
WXVER = '2.8'
if wxversion.checkInstalled(WXVER):
    wxversion.select(WXVER)
else:
    print >> sys.stderr, "This application requires wxPython version %s" % (WXVER)
    sys.exit(1)
import wx

import base_frame
import timeline

class RxPlayApp(wx.App):
    def __init__(self, input_files, options):
        wx.App.__init__(self)

        frame          = base_frame.BaseFrame(None, 'rxplay', 'Timeline', title='rxplay - ' + input_files[0])
        timeline_panel = timeline.TimelinePanel(input_files, options, frame, -1)
        #frame.CreateStatusBar()
        frame.Show()

def connect_to_ros(node_name, init_timeout):
    # Attempt to connect to master node
    class InitNodeThread(threading.Thread):
        def __init__(self):
            threading.Thread.__init__(self)
            self.daemon = True
            self.inited = False
            
        def run(self):
            rospy.loginfo('Attempting to connect to master node...')
            rospy.init_node(node_name, anonymous=True, disable_signals=True)
            self.inited = True

    try:
        # Check whether ros master is running
        master = rospy.get_master()
        master.getPid()
        
        # If so, attempt to initialize node
        init_thread = InitNodeThread()
        init_thread.start()
        time.sleep(init_timeout)

        if init_thread.inited:
            rospy.core.register_signals()
            rospy.loginfo('Connected to master node.')
        else:
            rospy.logerr('Giving up. Couldn\'t connect to master node.')
    except:
        rospy.loginfo('Master not found.')
  
def rxplay_main():
    # Parse command line for input files and options
    parser = optparse.OptionParser()
    parser.add_option('-i', '--index',           action='store_true', default=False, help='don\'t launch GUI; just generate bag file indexes')   
    parser.add_option('-t', '--init-timeout',    action='store',      default=0.5,   help='timeout in secs for connecting to master node')   
    parser.add_option('-k', '--show-thumbnails', action='store_true', default=False, help='display thumbnails for sensor_msgs/Image messages')   
    options, args = parser.parse_args(sys.argv[1:])
    if len(args) == 0:
        parser.print_help()
        return
    input_files = args[:]
    
    if options.index:
        # TODO: should not launch GUI, just create bag indexes
        pass
    else:
        connect_to_ros('rxplay', options.init_timeout)
    
        # Start application
        app = RxPlayApp(input_files, options)
        app.MainLoop()
     
        rospy.signal_shutdown('GUI shutdown')    
