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

PKG = 'rxbag'
import roslib; roslib.load_manifest(PKG)
import rospy

import bisect
import time

import wx

from rxbag.util.layer import Layer

class MessageView(Layer):
    """
    A message details renderer. When registered with rxbag, a MessageView is called
    whenever the timeline playhead moves.
    """
    name = 'Untitled'
    
    def __init__(self, timeline, parent, title, x, y, width, height):
        Layer.__init__(self, parent, title, x, y, width, height)
        
        self.timeline = timeline
        self.border   = False
    
    def message_viewed(self, bag, topic, msg, t):
        """
        View the message.
        
        @param bag: the bag file the message is contained in
        @type  bag: rosbag.Bag
        @param topic: the message topic
        @type  topic: str
        @param msg: the message
        @type  msg: roslib.Message
        @param t: the message timestamp
        @type  t: rospy.Time
        """
        pass

    def message_cleared(self):
        """
        Clear the currently viewed message (if any).
        """
        pass
    
    def close(self):
        """
        Close the message view, releasing any resources.
        """
        pass

class TopicMessageView(MessageView):
    """
    A message view with a toolbar for navigating messages.
    """
    def __init__(self, timeline, parent, title, x, y, width, height):
        MessageView.__init__(self, timeline, parent, title, x, y, width, height)

        self.topic = None
        self.stamp = None
        self.entry = None

        self._create_toolbar()

    def message_viewed(self, bag, msg_details):
        self.topic, _, self.stamp = msg_details

    def on_close(self, event):
        self.timeline.remove_view(self.topic, self)

    def navigate_first(self):
        if not self.topic:
            return

        for entry in self.timeline.get_entries(self.topic, self.timeline.start_stamp, self.timeline.end_stamp):
            self.timeline.set_playhead(entry.time)
            break

    def navigate_previous(self):
        if not self.topic:
            return

        last_entry = None
        for entry in self.timeline.get_entries(self.topic, self.timeline.start_stamp, self.timeline.playhead):
            if entry.time < self.timeline.playhead:
                last_entry = entry
            
        if last_entry:
            self.timeline.set_playhead(last_entry.time)

    def navigate_next(self):
        if not self.topic:
            return

        for entry in self.timeline.get_entries(self.topic, self.timeline.playhead, self.timeline.end_stamp):
            if entry.time > self.timeline.playhead:
                self.timeline.set_playhead(entry.time)
                break

    def navigate_last(self):
        if not self.topic:
            return

        last_entry = None
        for entry in self.timeline.get_entries(self.topic, self.timeline.start_stamp, self.timeline.end_stamp):
            last_entry = entry

        if last_entry:
            self.timeline.set_playhead(last_entry.time)

    @property
    def frame(self):
        return self.parent.GetParent()

    def _create_toolbar(self):
        icons_dir = roslib.packages.get_pkg_dir(PKG) + '/icons/'

        tb = self.frame.CreateToolBar()
        tb.Bind(wx.EVT_TOOL, lambda e: self.navigate_first(),    tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'navigate_first.png')))
        tb.Bind(wx.EVT_TOOL, lambda e: self.navigate_previous(), tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'navigate_previous.png')))
        tb.Bind(wx.EVT_TOOL, lambda e: self.navigate_next(),     tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'navigate_next.png')))
        tb.Bind(wx.EVT_TOOL, lambda e: self.navigate_last(),     tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'navigate_last.png')))
        tb.Realize()
