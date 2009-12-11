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
import wx

from layer import Layer
from bag_index import BagIndex

class StatusLayer(Layer):
    def __init__(self, parent, title, timeline, x, y, width, height, max_repaint=None):
        Layer.__init__(self, parent, title, x, y, width, height, max_repaint)
        
        self.timeline = timeline

        self.font       = wx.Font(10, wx.FONTFAMILY_MODERN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)
        self.font_color = wx.Colour(0, 0, 0)

    def paint(self, dc):
        dc.SetBackground(self.timeline.background_brush)
        dc.Clear()

        if not self.timeline.playhead:
            return

        dc.SetFont(self.font)
        dc.SetTextForeground(self.font_color)

        s = BagIndex.stamp_to_str(self.timeline.playhead)
        
        spd = self.timeline.play_speed
        spd_str = None
        if spd != 0.0:
            if spd > 1.0:
                spd_str = '>> %.0fx' % spd
            elif spd == 1.0:
                spd_str = '>'
            elif spd > 0.0:
                spd_str = '> 1/%.0fx' % (1.0 / spd)
            elif spd > -1.0:
                spd_str = '< 1/%.0fx' % (1.0 / -spd)
            elif spd == 1.0:
                spd_str = '<'
            else:
                spd_str = '<< %.0fx' % -spd
        if spd_str:
            s += ' ' + spd_str

        x = self.timeline.margin_left
        y = self.timeline.history_top - dc.GetTextExtent(s)[1] - 4

        dc.DrawText(s, x, y)
