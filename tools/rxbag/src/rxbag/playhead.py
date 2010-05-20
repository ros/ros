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

import wx

from util.layer import Layer

class PlayheadLayer(Layer):
    def __init__(self, parent, title, timeline, x, y, width, height):
        Layer.__init__(self, parent, title, x, y, width, height)
        
        self.timeline = timeline
        
        self.pointer_size = (6, 6)
        
        self.timeline_rect = self._get_timeline_rect()

    def check_dirty(self):
        timeline_rect = self._get_timeline_rect()
        if self.timeline_rect != timeline_rect:
            self.timeline_rect = timeline_rect
            self.update_position()

    def paint(self, dc):
        Layer.paint(self, dc)
        
        if not self.timeline.playhead:
            return
        
        px, pw, ph = self.width / 2, self.pointer_size[0], self.pointer_size[1]

        # Line
        dc.set_line_width(2)
        dc.set_source_rgba(1, 0, 0, 0.75)
        dc.move_to(px, self.timeline.history_top - 1)
        dc.line_to(px, self.timeline.history_bottom + 2)
        dc.stroke()

        dc.set_source_rgb(1, 0, 0)

        # Upper triangle
        py = self.timeline.history_top - ph
        dc.move_to(px,      py + ph)
        dc.line_to(px + pw, py)
        dc.line_to(px - pw, py)
        dc.line_to(px ,     py + ph)
        dc.fill()

        # Lower triangle
        py = self.timeline.history_bottom + 1
        dc.move_to(px,      py)
        dc.line_to(px + pw, py + ph)
        dc.line_to(px - pw, py + ph)
        dc.line_to(px,      py)
        dc.fill()

    def on_size(self, event):
        self.resize(self.width, self.timeline.height)

    def update_position(self):
        if not self.timeline.playhead:
            return

        playhead_x = self.timeline.map_stamp_to_x(self.timeline.playhead.to_sec())

        self.move(self.timeline.x + playhead_x - self.width / 2, self.timeline.y)

    def _get_timeline_rect(self):
        return (self.timeline.history_left, self.timeline.history_top, self.timeline.history_width, self.timeline.history_height)
