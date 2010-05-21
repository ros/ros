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
import bisect
import time

import wx

from util.layer import Layer

class TimelineRenderer(object):
    """
    A widget that can render an interval of time of a topic as a rectangle on the timeline.
    @param msg_combine_px: don't draw discrete messages if they're less than this many pixels separated [optional]
    @type  msg_combine_px: float
    """
    def __init__(self, timeline, msg_combine_px=1.5):
        self.timeline       = timeline
        self.msg_combine_px = msg_combine_px 

    def get_segment_height(self, topic):
        return None

    def draw_timeline_segment(self, dc, topic, stamp_start, stamp_end, x, y, width, height):
        return False

    def close(self):
        pass

class MessageView(Layer):
    """
    A widget that can display message details.
    """
    
    name = 'Untitled'
    
    def __init__(self, timeline, parent, title, x, y, width, height):
        Layer.__init__(self, parent, title, x, y, width, height)
        
        self.timeline = timeline
        self.border   = False
    
    def message_viewed(self, bag, topic, msg, t):
        pass
    
    def message_cleared(self):
        pass

    def paint(self, dc):
        pass

    def close(self):
        pass

class TopicMessageView(MessageView):
    def __init__(self, timeline, parent, title, x, y, width, height):
        MessageView.__init__(self, timeline, parent, title, x, y, width, height)

        self.topic = None
        self.stamp = None
        self.entry = None

        self._create_toolbar()

    def message_viewed(self, bag, msg_details):
        topic, msg, t = msg_details

        self.topic = topic
        self.stamp = t

    def message_cleared(self):
        pass
        
    def on_close(self, event):
        self.timeline.remove_view(self.topic, self)

    def navigate_first(self):
        if not self.topic:
            return

        for entry in self.timeline.bag_file._get_entries(self.timeline.bag_file._get_connections(self.topic)):
            self.timeline.set_playhead(entry.time.to_sec())
            break

#    def navigate_previous(self):
#        if not self.topic:
#            return
#        
#        index = bisect.bisect_right(self.timeline.message_history_cache[self.topic], self.stamp.to_sec()) - 1
#        if index > 0:
#            self.stamp = self.timeline.message_history_cache[self.topic][index - 1]
#            self.timeline.set_playhead(self.stamp)
#
#    def navigate_next(self):
#        if not self.topic:
#            return
#
#        index = bisect.bisect_right(self.timeline.message_history_cache[self.topic], self.stamp.to_sec()) - 1
#        if index < len(self.timeline.message_history_cache[self.topic]) - 1:
#            self.stamp = self.timeline.message_history_cache[self.topic][index + 1]
#            self.timeline.set_playhead(self.stamp)

    def navigate_last(self):
        if not self.topic:
            return
        
        for entry in self.timeline.bag_file._get_entries_reverse(self.timeline.bag_file._get_connections(self.topic)):
            self.timeline.set_playhead(entry.time.to_sec())
            break

    @property
    def frame(self):
        return self.parent.GetParent()

    def _create_toolbar(self):
        icons_dir = roslib.packages.get_pkg_dir(PKG) + '/icons/'

        tb = self.frame.CreateToolBar()
        tb.Bind(wx.EVT_TOOL, lambda e: self.navigate_first(),    tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'resultset_first.png')))
        #tb.Bind(wx.EVT_TOOL, lambda e: self.navigate_previous(), tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'resultset_previous.png')))
        #tb.Bind(wx.EVT_TOOL, lambda e: self.navigate_next(),     tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'resultset_next.png')))
        tb.Bind(wx.EVT_TOOL, lambda e: self.navigate_last(),     tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'resultset_last.png')))
        tb.Realize()
