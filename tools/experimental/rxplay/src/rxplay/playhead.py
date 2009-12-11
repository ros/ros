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
from util.layer import TransparentLayer

class PlayheadLayer(TransparentLayer):
    def __init__(self, parent, title, timeline, x, y, width, height, max_repaint=None):
        TransparentLayer.__init__(self, parent, title, x, y, width, height, max_repaint)
        
        self.timeline = timeline
        
        self.pen          = wx.RED_PEN
        self.pointer_size = (6, 6)
        
        self.timeline_rect = self._get_timeline_rect()

    def check_dirty(self):
        timeline_rect = self._get_timeline_rect()
        if self.timeline_rect != timeline_rect:
            self.timeline_rect = timeline_rect
            self.update_position()
        elif not self.timeline.bag_index.loaded:
            self.invalidate()

    def paint(self, dc):
        TransparentLayer.paint(self, dc)
        
        px, pw, ph = self.width / 2, self.pointer_size[0], self.pointer_size[1]

        # Draw line
        dc.SetPen(self.pen)
        dc.DrawLine(px, self.timeline.history_top, px, self.timeline.history_bottom + 3)

        # Draw upper triangle
        py = self.timeline.history_top - (ph + 1)
        dc.DrawLineList([(px,      py + ph, px + pw, py),
                         (px + pw, py,      px - pw, py),
                         (px - pw, py,      px ,     py + ph)])

        # Draw lower triangle
        py = self.timeline.history_bottom + 1
        dc.DrawLineList([(px,      py,      px + pw, py + ph),
                         (px + pw, py + ph, px - pw, py + ph),
                         (px - pw, py + ph, px,      py)])
        
    def on_size(self, event):
        self.resize(self.width, self.timeline.height)
        
    def on_mouse_move(self, event):
        TransparentLayer.on_mouse_move(self, event)

        self.update_position()

    def update_position(self):
        if not self.timeline.playhead:
            return

        playhead_x = self.timeline.map_stamp_to_x(self.timeline.playhead)

        self.move(self.timeline.x + playhead_x - self.width / 2, self.timeline.y)

    def _get_timeline_rect(self):
        return (self.timeline.history_left, self.timeline.history_top, self.timeline.history_width, self.timeline.history_height)
