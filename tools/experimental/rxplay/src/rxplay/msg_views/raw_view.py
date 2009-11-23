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

from rxplay import msg_view

class RawView(msg_view.TopicMsgView):
    name = 'Raw'
    
    def __init__(self, timeline, parent, title, x, y, width, height, max_repaint=0.1):
        msg_view.TopicMsgView.__init__(self, timeline, parent, title, x, y, width, height, max_repaint)
        
        self.title_font       = wx.Font(9, wx.FONTFAMILY_SCRIPT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.text_font        = wx.Font(9, wx.FONTFAMILY_MODERN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.title_color      = wx.BLUE
        self.text_color       = wx.BLACK       
        self.border_pen       = wx.BLACK_PEN
        self.background_brush = wx.WHITE_BRUSH       
        self.margin           = (6, 6)

        self.title = None
        self.text  = None

    def message_viewed(self, bag_file, bag_index, topic, stamp, datatype, msg_index, msg):
        msg_view.TopicMsgView.message_viewed(self, bag_file, bag_index, topic, stamp, datatype, msg_index, msg)

        self.title = '%s #%d' % (time.asctime(time.localtime(stamp.to_sec())), msg_index)
        self.text  = str(msg)
        
        self.invalidate()
        
    def message_cleared(self):
        msg_view.TopicMsgView.message_cleared(self)
        
        self.title = None
        self.text  = None

    def paint(self, dc):
        dc.SetBrush(self.background_brush)
        if self.border:
            dc.SetPen(self.border_pen)
            dc.DrawRectangle(0, 0, self.width, self.height)
        else:
            dc.Clear()

        y = self.margin[1]
        if self.title:
            dc.SetFont(self.title_font)
            dc.SetTextForeground(self.title_color)
            dc.DrawText(self.title, self.margin[0], y)

            (w, h) = dc.GetTextExtent(self.title)
            y += h + self.margin[1]
            
            dc.DrawLine(0, y, self.width, y)
            y += self.margin[1]

        if self.text:
            dc.SetFont(self.text_font)
            dc.SetTextForeground(self.text_color)
            try:
                dc.DrawText(self.text, self.margin[0], y)
            except UnicodeDecodeError, e:
                pass

    def on_size(self, event):
        self.resize(*self.parent.GetClientSize())

    def clear(self):
        self.title = None
        self.text  = None
        self.invalidate()
