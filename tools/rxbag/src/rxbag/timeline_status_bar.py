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

import wx

import bag_helper

PlayheadChangedEvent, EVT_PLAYHEAD_CHANGED = wx.lib.newevent.NewEvent()

class TimelineStatusBar(wx.StatusBar):
    def __init__(self, parent, timeline):
        wx.StatusBar.__init__(self, parent, -1)

        self.timeline = timeline

        self.progress_field       = 1
        self.timestamp_field      = 2
        self.human_readable_field = 3
        self.elapsed_field        = 4
        self.playspeed_field      = 5

        self.progress_width       = 150
        self.timestamp_width      = 125
        self.human_readable_width = 180
        self.elapsed_width        = 110
        self.playspeed_width      =  80

        self.SetFieldsCount(6)

        self.gauge = wx.Gauge(self, range=100)
        self.gauge.Hide()
        
        self._reposition_progress()

        parent.Bind(EVT_PLAYHEAD_CHANGED, lambda e: self._update())
        parent.Bind(wx.EVT_SIZE,          self.on_size)

        self._update()

    # property: progress

    def _get_progress(self):
        return self.gauge.Value

    def _set_progress(self, value):
        self.gauge.Value = value

    progress = property(_get_progress, _set_progress)

    def on_size(self, event):
        main_width = self.Size[0] - (self.progress_width + self.timestamp_width + self.human_readable_width + self.elapsed_width + self.playspeed_width)
        self.SetStatusWidths([main_width, self.progress_width, self.timestamp_width, self.human_readable_width, self.elapsed_width, self.playspeed_width])

        self._reposition_progress()
        
        event.Skip()

    def _reposition_progress(self):
        rect = self.GetFieldRect(self.progress_field)

        self.gauge.SetPosition((rect.x + 1, rect.y + 1))
        self.gauge.SetSize((rect.width - 2, rect.height - 2))

    def _update(self):
        if self.timeline.playhead is None or self.timeline.start_stamp is None:
            return

        # Raw timestamp
        self.SetStatusText('%d.%s' % (self.timeline.playhead.secs, str(self.timeline.playhead.nsecs)[:3]), self.timestamp_field)

        # Human-readable time
        self.SetStatusText(bag_helper.stamp_to_str(self.timeline.playhead), self.human_readable_field)

        # Elapsed time (in seconds)
        self.SetStatusText('%.3fs' % (self.timeline.playhead - self.timeline.start_stamp).to_sec(), self.elapsed_field)

        # Play speed
        spd = self.timeline.play_speed
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

            self.SetStatusText(spd_str, self.playspeed_field)
        else:
            self.SetStatusText('', self.playspeed_field)
