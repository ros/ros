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

class TimelineToolBar(wx.ToolBar):
    def __init__(self, parent, timeline):
        wx.ToolBar.__init__(self, parent, -1, style=wx.TB_TEXT)

        self.timeline = timeline

        self._state = None

        self._setup()

    def _setup(self):
        # If state of toolbar hasn't changed, don't recreate the tools
        new_state = TimelineToolBarState(self.timeline)
        if new_state == self._state:
            return
        
        self._state = new_state

        self.ClearTools()
        
        icons_dir = roslib.packages.get_pkg_dir(PKG) + '/icons/'
        
        start_icon       = wx.Bitmap(icons_dir + 'start.png')
        rewind_icon      = wx.Bitmap(icons_dir + 'rewind.png')
        play_icon        = wx.Bitmap(icons_dir + 'play.png')
        fastforward_icon = wx.Bitmap(icons_dir + 'fastforward.png')
        end_icon         = wx.Bitmap(icons_dir + 'end.png')
        stop_icon        = wx.Bitmap(icons_dir + 'stop.png')

        if self.timeline._play_speed > 1.0:       
            fastforward_icon = wx.Bitmap(icons_dir + 'fastforward_active.png')
        elif self.timeline._play_speed > 0.0:       
            play_icon        = wx.Bitmap(icons_dir + 'play_active.png')
        elif self.timeline._play_speed == 0.0:
            stop_icon        = wx.Bitmap(icons_dir + 'stop_active.png')
        elif self.timeline._play_speed < 0.0:
            rewind_icon      = wx.Bitmap(icons_dir + 'rewind_active.png')

        idle_renderers = len(self.timeline._rendered_topics) < len(self.timeline.topics)
        if idle_renderers:
            thumbnails_icon = wx.Bitmap(icons_dir + 'thumbnails.png')
        else:
            thumbnails_icon = wx.Bitmap(icons_dir + 'thumbnails_off.png')           

        # Create the tools

        start_tool       = self.AddLabelTool(wx.ID_ANY, '', start_icon,       shortHelp='Start (Home)',           longHelp='Move playhead to start of timeline')
        rewind_tool      = self.AddLabelTool(wx.ID_ANY, '', rewind_icon,      shortHelp='Rewind (Numpad -)',      longHelp='Rewind')
        play_tool        = self.AddLabelTool(wx.ID_ANY, '', play_icon,        shortHelp='Play (Space)',           longHelp='Play')
        fastforward_tool = self.AddLabelTool(wx.ID_ANY, '', fastforward_icon, shortHelp='Fastforward (Numpad +)', longHelp='Fastforward')
        end_tool         = self.AddLabelTool(wx.ID_ANY, '', end_icon,         shortHelp='End (End)',              longHelp='Move playhead to end of timeline')
        stop_tool        = self.AddLabelTool(wx.ID_ANY, '', stop_icon,        shortHelp='Stop (Space)',           longHelp='Stop')
        if self.timeline._recorder:
            if self.timeline._recorder.paused:
                record_icon = 'record_inactive.png'
            else:
                record_icon = 'record.png'
            record_tool = self.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + record_icon), shortHelp='Record (Pause)', longHelp='Toggle recording')
        self.AddSeparator()
        zoom_in_tool     = self.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'zoom_in.png'),  shortHelp='Zoom in (Page Up)',    longHelp='Zoom in timeline')
        zoom_out_tool    = self.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'zoom_out.png'), shortHelp='Zoom out (Page Down)', longHelp='Zoom out timeline')
        zoom_tool        = self.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'zoom.png'),     shortHelp='Reset zoom', longHelp='View entire timeline')
        self.AddSeparator()
        thumbnails_tool  = self.AddLabelTool(wx.ID_ANY, '', thumbnails_icon, shortHelp='Thumbnails', longHelp='Toggle thumbnails')

        # Bind tools to events

        self.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_start(),       start_tool)
        self.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_rewind(),      rewind_tool)
        self.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_play(),        play_tool)       
        self.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_fastforward(), fastforward_tool)       
        self.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_end(),         end_tool)
        self.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_stop(),        stop_tool)
        if self.timeline._recorder:
            self.Bind(wx.EVT_TOOL, lambda e: self.timeline.toggle_recording(), record_tool)
        self.Bind(wx.EVT_TOOL, lambda e: self.timeline.zoom_in(),              zoom_in_tool)
        self.Bind(wx.EVT_TOOL, lambda e: self.timeline.zoom_out(),             zoom_out_tool)
        self.Bind(wx.EVT_TOOL, lambda e: self.timeline.reset_zoom(),           zoom_tool)
        self.Bind(wx.EVT_TOOL, lambda e: self.timeline.toggle_renderers(),     thumbnails_tool)

        # Set the enabled flag

        if self.timeline._stamp_left is None:
            for tool in [start_tool, rewind_tool, play_tool, fastforward_tool, end_tool, stop_tool, zoom_in_tool, zoom_out_tool, zoom_tool, thumbnails_tool]:
                self.EnableTool(tool.Id, False)
        else:
            self.EnableTool(zoom_in_tool.Id,  self.timeline.can_zoom_in())
            self.EnableTool(zoom_out_tool.Id, self.timeline.can_zoom_out())

        # Realize the toolbar

        self.Realize()

class TimelineToolBarState(object):
    def __init__(self, timeline):
        self._fastforward_active = timeline._play_speed > 1.0
        self._play_active        = timeline._play_speed > 0.0
        self._stop_active        = timeline._play_speed == 0.0
        self._rewind_active      = timeline._play_speed < 0.0
        self._idle_renderers     = len(timeline._rendered_topics) < len(timeline.topics)
        self._recorder_visible   = timeline._recorder is not None
        self._recorder_paused    = timeline._recorder is not None and timeline._recorder.paused
        self._timeline_visible   = timeline._stamp_left is not None
        self._can_zoom_in        = timeline._stamp_left is not None and timeline.can_zoom_in()
        self._can_zoom_out       = timeline._stamp_left is not None and timeline.can_zoom_out()

    def __eq__(self, other):
        if other is None:
            return False
        
        return self._fastforward_active == other._fastforward_active \
           and self._play_active        == other._play_active        \
           and self._stop_active        == other._stop_active        \
           and self._rewind_active      == other._rewind_active      \
           and self._idle_renderers     == other._idle_renderers     \
           and self._recorder_visible   == other._recorder_visible   \
           and self._recorder_paused    == other._recorder_paused    \
           and self._timeline_visible   == other._timeline_visible   \
           and self._can_zoom_in        == other._can_zoom_in        \
           and self._can_zoom_out       == other._can_zoom_out
