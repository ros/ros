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

PKG = 'rxbag'
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
    print >> sys.stderr, 'This application requires wxPython version %s' % WXVER
    sys.exit(1)
import wx

import util.base_frame
import bag_index
import timeline

class RxBagApp(wx.App):
    def __init__(self, input_files, options):
        self.input_files = [input_files[0]]
        self.options     = options

        wx.App.__init__(self)
    
    def OnInit(self):
        try:
            if len(self.input_files) == 1:
                frame_title = 'rxbag - ' + self.input_files[0]
            else:
                frame_title = 'rxbag - [%d bags]' % len(self.input_files)
    
            frame = util.base_frame.BaseFrame(None, 'rxbag', 'Timeline', title=frame_title)
            timeline_panel = timeline.TimelinePanel(self.input_files, self.options, frame, -1)
            frame.Show()
            self.SetTopWindow(frame)
        except:
            return False

        return True

def connect_to_ros(node_name, init_timeout):
    # Attempt to connect to master node
    class InitNodeThread(threading.Thread):
        def __init__(self):
            threading.Thread.__init__(self)
            self.setDaemon(True)
            self.inited = False
            
        def run(self):
            rospy.loginfo('Attempting to connect to master node...')
            rospy.init_node(node_name, anonymous=True, disable_signals=True)
            self.inited = True

    try:
        # Check whether ROS master is running
        master = rospy.get_master()
        master.getPid()
        
        # If so (i.e. no exception), attempt to initialize node
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

def rxbag_main():
    # Parse command line for input files and options
    usage = "usage: %prog [options] BAG_FILE.bag"
    parser = optparse.OptionParser(usage=usage)
    #parser.add_option('-t', '--init-timeout', action='store', default=0.5, help='timeout in secs for connecting to master node')   
    options, args = parser.parse_args(sys.argv[1:])
    if len(args) == 0:
        parser.print_help()
        return
    input_files = args[:]

    #connect_to_ros('rxbag', options.init_timeout)

    # Start application
    app = RxBagApp(input_files, options)
    app.MainLoop()

    rospy.signal_shutdown('GUI shutdown')
