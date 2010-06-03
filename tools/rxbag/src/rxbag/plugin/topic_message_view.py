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

from rxbag import bag_helper
from message_view import MessageView

class TopicMessageView(MessageView):
    """
    A message view with a toolbar for navigating messages in a single topic.
    """
    def __init__(self, timeline, parent):
        MessageView.__init__(self, timeline)
        
        self._parent = parent
        self._topic  = None
        self._stamp  = None

        self._toolbar = self.parent.CreateToolBar()
        self._setup_toolbar()

        self.parent.StatusBar = TopicMessageViewStatusBar(self.parent, self)
        
        self.parent.Bind(wx.EVT_CLOSE, self._on_close)

    @property
    def parent(self): return self._parent

    @property
    def topic(self): return self._topic

    @property
    def stamp(self): return self._stamp

    ## MessageView implementation

    def message_viewed(self, bag, msg_details):
        self._topic, _, self._stamp = msg_details[:3]
        
        wx.CallAfter(self.parent.StatusBar.update)

    ## Events

    def _on_close(self, event):
        # @todo: needs to handle closing when a message hasn't been viewed yet
        if self._topic:
            self.timeline.remove_view(self._topic, self)

        event.Skip()

    def navigate_first(self):
        if not self.topic:
            return

        for entry in self.timeline.get_entries(self._topic, *self.timeline.play_region):
            self.timeline.playhead = entry.time
            break

    def navigate_previous(self):
        if not self.topic:
            return

        last_entry = None
        for entry in self.timeline.get_entries(self._topic, self.timeline.start_stamp, self.timeline.playhead):
            if entry.time < self.timeline.playhead:
                last_entry = entry
            
        if last_entry:
            self.timeline.playhead = last_entry.time

    def navigate_next(self):
        if not self.topic:
            return

        for entry in self.timeline.get_entries(self._topic, self.timeline.playhead, self.timeline.end_stamp):
            if entry.time > self.timeline.playhead:
                self.timeline.playhead = entry.time
                break

    def navigate_last(self):
        if not self.topic:
            return

        last_entry = None
        for entry in self.timeline.get_entries(self._topic, *self.timeline.play_region):
            last_entry = entry

        if last_entry:
            self.timeline.playhead = last_entry.time

    def _setup_toolbar(self):
        self._toolbar.ClearTools()
        
        icons_dir = roslib.packages.get_pkg_dir(PKG) + '/icons/'

        navigate_first_tool    = self._toolbar.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'navigate_first.png'),    shortHelp='First message',    longHelp='Move playhead to first message on topic')
        navigate_previous_tool = self._toolbar.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'navigate_previous.png'), shortHelp='Previous message', longHelp='Move playhead to previous message on topic')
        navigate_next_tool     = self._toolbar.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'navigate_next.png'),     shortHelp='Next message',     longHelp='Move playhead to next message on topic')
        navigate_last_tool     = self._toolbar.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'navigate_last.png'),     shortHelp='Last message',     longHelp='Move playhead to last message on topic')

        self._toolbar.Bind(wx.EVT_TOOL, lambda e: self.navigate_first(),    navigate_first_tool)
        self._toolbar.Bind(wx.EVT_TOOL, lambda e: self.navigate_previous(), navigate_previous_tool)
        self._toolbar.Bind(wx.EVT_TOOL, lambda e: self.navigate_next(),     navigate_next_tool)
        self._toolbar.Bind(wx.EVT_TOOL, lambda e: self.navigate_last(),     navigate_last_tool)

        self._toolbar.Realize()

class TopicMessageViewStatusBar(wx.StatusBar):
    def __init__(self, parent, message_view):
        wx.StatusBar.__init__(self, parent, -1)

        self.message_view = message_view

        self.timestamp_field      = 1
        self.human_readable_field = 2
        self.elapsed_field        = 3

        self.timestamp_width      = 125
        self.human_readable_width = 180
        self.elapsed_width        = 110

        self.SetFieldsCount(4)

        parent.Bind(wx.EVT_SIZE, self.on_size)

        self.update()

    def on_size(self, event):
        main_width = max(10, self.Size[0] - (self.timestamp_width + self.human_readable_width + self.elapsed_width))
        self.SetStatusWidths([main_width, self.timestamp_width, self.human_readable_width, self.elapsed_width])
        event.Skip()

    def update(self):
        if self.message_view.stamp is None or self.message_view.timeline.start_stamp is None:
            return

        # Raw timestamp
        self.SetStatusText('%d.%s' % (self.message_view.stamp.secs, str(self.message_view.stamp.nsecs)[:3]), self.timestamp_field)

        # Human-readable time
        self.SetStatusText(bag_helper.stamp_to_str(self.message_view.stamp), self.human_readable_field)

        # Elapsed time (in seconds)
        self.SetStatusText('%.3fs' % (self.message_view.stamp - self.message_view.timeline.start_stamp).to_sec(), self.elapsed_field)
