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

import sys
import threading
import time

import wx

import rosbag

import util.base_frame
import timeline_panel

class RxBagApp(wx.App):
    def __init__(self, options, args):
        self.options = options
        self.args    = args
        
        self.connected_to_ros = False

        wx.App.__init__(self)
    
    def OnInit(self):
        try:
            # Get filename to record to
            if self.options.record:
                if not self.connect_to_ros():
                    raise Exception('recording requires connection to master')

                # Get filename to record to
                if len(self.args) > 0:
                    record_filename = self.args[0]
                else:
                    record_filename = 'rxbag.bag'            

            # Title bar
            if self.options.record:
                frame_title = 'rxbag - %s [recording]' % record_filename
            elif len(self.args) == 1:
                frame_title = 'rxbag - ' + self.args[0]
            else:
                frame_title = 'rxbag - [%d bags]' % len(self.args)

            # Create main timeline frame
            frame = util.base_frame.BaseFrame(None, 'rxbag', 'Timeline', title=frame_title)
            panel = timeline_panel.TimelinePanel(frame, -1)
            panel.app = self
            frame.Show()
            self.SetTopWindow(frame)

            if self.options.record:
                panel.timeline.record_bag(record_filename)
            else:
                RxBagInitThread(self, panel.timeline)
            
        except Exception, ex:
            print >> sys.stderr, 'Error initializing application:', ex
            raise
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
                self.init_cv.acquire()
                self.inited = True
                self.init_cv.notify_all()
                self.init_cv.release()
    
        try:
            # Check whether ROS master is running                                                                                                                          
            master = rospy.get_master()
            master.getPid()
    
            # If so (i.e. no exception), attempt to initialize node                                                                                                        
            init_thread = InitNodeThread()
            init_thread.init_cv.acquire()
            init_thread.start()
            init_thread.init_cv.wait(init_timeout)
    
            if init_thread.inited:
                rospy.core.register_signals()
                rospy.loginfo('Connected to ROS master.')
                self.connected_to_ros = True
                return True
            else:
                rospy.logerr('Couldn\'t connect to ROS master.')
        except:
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
            try:
                bag = rosbag.Bag(input_file)
                self.timeline.add_bag(bag)
            except Exception, ex:
                print >> sys.stderr, 'Error loading [%s]: %s' % (input_file, str(ex))
