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
import time

import wx

import layer

## A widget that can render an interval of time of a topic as a rectangle on the timeline
class TimelineRenderer:
    def __init__(self, timeline):
        self.timeline = timeline

    def get_segment_height(self, topic):
        return None

    def draw_timeline_segment(self, dc, topic, stamp_start, stamp_end, x, y, width, height):
        return False

## A widget that can display message details 
class MsgView(layer.Layer):
    name = 'Untitled'
    
    def __init__(self, timeline, parent, title, x, y, width, height, max_repaint=None):
        layer.Layer.__init__(self, parent, title, x, y, width, height, max_repaint)
        
        self.timeline = timeline
        self.border   = False
        
    def message_viewed(self, bag_file, bag_index, topic, stamp, datatype, msg_index, msg):
        pass
    
    def message_cleared(self):
        pass

    def paint(self, dc):
        pass

class TopicMsgView(MsgView):
    def __init__(self, timeline, parent, title, x, y, width, height, max_repaint=None):
        MsgView.__init__(self, timeline, parent, title, x, y, width, height, max_repaint)
        
        self.topic     = None
        self.msg_index = None
    
        self._create_toolbar()

    def message_viewed(self, bag_file, bag_index, topic, stamp, datatype, msg_index, msg):
        self.topic     = topic
        self.msg_index = msg_index
    
    def message_cleared(self):
        self.msg_index = None
        
    def navigate_first(self):
        if self.topic and self.msg_index is not None:
            topic_positions = self.timeline.bag_index.msg_positions[self.topic]
            if len(topic_positions) > 0:
                self.timeline.set_playhead(topic_positions[0][0])

    def navigate_previous(self):
        if self.topic and self.msg_index is not None:
            new_msg_index = self.msg_index - 1
            if new_msg_index > 0:
                self.timeline.set_playhead(self.timeline.bag_index.msg_positions[self.topic][new_msg_index][0])

    def navigate_next(self):
        if self.topic and self.msg_index is not None:
            new_msg_index = self.msg_index + 1
            topic_positions = self.timeline.bag_index.msg_positions[self.topic]
            if new_msg_index < len(topic_positions):
                self.timeline.set_playhead(topic_positions[new_msg_index][0])

    def navigate_last(self):
        if self.topic and self.msg_index is not None:
            topic_positions = self.timeline.bag_index.msg_positions[self.topic]
            if len(topic_positions) > 0:
                self.timeline.set_playhead(topic_positions[-1][0])

    def _create_toolbar(self):
        frame = self.parent.GetParent()

        icons_dir = roslib.packages.get_pkg_dir(PKG) + '/icons/'

        tb = frame.CreateToolBar()
        tb.Bind(wx.EVT_TOOL, lambda e: self.navigate_first(),    tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'resultset_first.png')))
        tb.Bind(wx.EVT_TOOL, lambda e: self.navigate_previous(), tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'resultset_previous.png')))
        tb.Bind(wx.EVT_TOOL, lambda e: self.navigate_next(),     tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'resultset_next.png')))
        tb.Bind(wx.EVT_TOOL, lambda e: self.navigate_last(),     tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'resultset_last.png')))
        tb.Realize()
