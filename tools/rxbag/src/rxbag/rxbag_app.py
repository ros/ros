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

"""
Defines the rxbag wx.App.
"""

from __future__ import with_statement

PKG = 'rxbag'
import roslib; roslib.load_manifest(PKG)
import rospy

import sys
import threading
import time

import wx

import rosbag

from util.base_frame import BaseFrame
from timeline        import Timeline

class RxBagApp(wx.App):
    def __init__(self, options, args):
        self.options = options
        self.args    = args
        
        self.connected_to_ros = False

        wx.App.__init__(self)
    
    def OnInit(self):
        try:
            if self.options.record:
                # Connect to ROS master
                if not self.connect_to_ros():
                    raise Exception('recording requires connection to master')

                # Get filename to record to
                record_filename = time.strftime('%Y-%m-%d-%H-%M-%S.bag', time.localtime(time.time()))
                if self.options.name:
                    record_filename = self.options.name
                    if not record_filename.endswith('.bag'):
                        record_filename += '.bag'
                elif self.options.prefix:
                    record_filename = '%s_%s' % (self.options.prefix, record_filename)

                rospy.loginfo('Recording to %s.' % record_filename)

            # Create main timeline frame
            self.frame = BaseFrame(None, 'rxbag', 'Timeline')
            self.frame.BackgroundColour = wx.WHITE
            self.frame.Bind(wx.EVT_CLOSE, lambda e: wx.Exit())

            scroll = wx.ScrolledWindow(self.frame, -1)
            scroll.BackgroundColour = wx.WHITE
            
            timeline = Timeline(scroll, -1)
            timeline.Size = (100, 100)
            
            self.frame.Show()
            self.SetTopWindow(self.frame)

            timeline.SetFocus()

            if self.options.record:
                timeline.record_bag(record_filename, all=self.options.all, topics=self.args, regex=self.options.regex, limit=self.options.limit)
            else:
                RxBagInitThread(self, timeline)

        except Exception, ex:
            print >> sys.stderr, 'Error initializing application:', ex
            #import traceback
            #traceback.print_exc()
            return False

        return True

    def connect_to_ros(self, init_timeout=3):
        if self.connected_to_ros:
            return True

        # Attempt to connect to master node                                                                                                                                
        class InitNodeThread(threading.Thread):
            def __init__(self):
                threading.Thread.__init__(self)
                self.setDaemon(True)
                self.inited = False
                self.init_cv = threading.Condition()
    
            def run(self):
                rospy.loginfo('Master found.  Connecting...')
                rospy.init_node('rxbag', anonymous=True, disable_signals=True)
                with self.init_cv:
                    self.inited = True
                    self.init_cv.notify_all()
    
        try:
            # Check whether ROS master is running                                                                                                                          
            master = rospy.get_master()
            master.getPid()
    
            # If so (i.e. no exception), attempt to initialize node                                                                                                        
            init_thread = InitNodeThread()
            with init_thread.init_cv:
                init_thread.start()
                init_thread.init_cv.wait(init_timeout)
    
            if init_thread.inited:
                rospy.core.register_signals()
                rospy.loginfo('Connected to ROS master.')
                self.connected_to_ros = True
                return True
            else:
                rospy.logerr('Couldn\'t connect to ROS master.')
        except Exception:
            rospy.loginfo('ROS master not found.')
        
        return False

class RxBagInitThread(threading.Thread):
    def __init__(self, app, timeline):
        threading.Thread.__init__(self)
        
        self.app      = app
        self.timeline = timeline
        
        self.start()
        
    def run(self):       
        for input_file in self.app.args:
            self.timeline.loading_filename = input_file               

            try:
                self.timeline.add_bag(rosbag.Bag(input_file))
            except Exception, ex:
                print >> sys.stderr, 'Error loading [%s]: %s' % (input_file, str(ex))

            self.timeline.loading_filename = None

        if self.app.options.start:
            playhead = self.timeline.start_stamp + rospy.Duration.from_sec(self.app.options.start)
            if playhead > self.timeline.end_stamp:
                playhead = self.timeline.end_stamp
            elif playhead < self.timeline.start_stamp:
                playhead = self.timeline.start_stamp
            
            self.timeline.playhead = playhead
