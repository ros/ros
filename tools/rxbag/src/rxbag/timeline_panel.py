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

from playhead        import PlayheadLayer
from status          import StatusLayer
from timeline        import Timeline
from util.base_frame import BaseFrame
from util.layer      import LayerPanel

class TimelinePanel(LayerPanel):
    def __init__(self, *args, **kwargs):
        LayerPanel.__init__(self, *args, **kwargs)

        self._create_controls()
        self._create_toolbar()

    def display_popup(self, pos):
        self.PopupMenu(TimelinePopupMenu(self, self.timeline), pos)

    def _create_controls(self):
        (width, height) = self.GetParent().GetClientSize()

        self.timeline = Timeline     (self, 'Timeline', 5, 19, width - 5, height - 19)
        self.status   = StatusLayer  (self, 'Status',   self.timeline, self.timeline.x, 4, 300, 16)
        self.playhead = PlayheadLayer(self, 'Playhead', self.timeline, 0, 0, 12, self.timeline.height)

        self.layers = [self.timeline, self.status, self.playhead]

    def _create_toolbar(self):
        icons_dir = roslib.packages.get_pkg_dir(PKG) + '/icons/'

        tb = self.GetParent().CreateToolBar()

        start_tool       = tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'control_start_blue.png'))
        rewind_tool      = tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'control_rewind_blue.png'))
        play_tool        = tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'control_play_blue.png'))
        fastforward_tool = tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'control_fastforward_blue.png'))
        end_tool         = tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'control_end_blue.png'))
        stop_tool        = tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'control_stop_blue.png'))
        tb.AddSeparator()
        zoom_in_tool     = tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'zoom_in.png'))
        zoom_out_tool    = tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'zoom_out.png'))
        zoom_tool        = tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'zoom.png'))
        tb.AddSeparator()
        thumbnails_tool  = tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'pictures.png'))
        #tb.AddSeparator()
        #save_layout_tool = tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'application_put.png'))
        #load_layout_tool = tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'application_get.png'))

        tb.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_start(),       start_tool)
        tb.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_rewind(),      rewind_tool)
        tb.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_play(),        play_tool)       
        tb.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_fastforward(), fastforward_tool)       
        tb.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_end(),         end_tool)
        tb.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_stop(),        stop_tool)
        tb.Bind(wx.EVT_TOOL, lambda e: self.timeline.zoom_in(),              zoom_in_tool)
        tb.Bind(wx.EVT_TOOL, lambda e: self.timeline.zoom_out(),             zoom_out_tool)
        tb.Bind(wx.EVT_TOOL, lambda e: self.timeline.reset_zoom(),           zoom_tool)
        tb.Bind(wx.EVT_TOOL, lambda e: self.timeline.toggle_renderers(),     thumbnails_tool)
        #tb.Bind(wx.EVT_TOOL, lambda e: self.timeline.save_layout(),          save_layout_tool)
        #tb.Bind(wx.EVT_TOOL, lambda e: self.timeline.load_layout(),          load_layout_tool)

        tb.Realize()

class TimelinePopupMenu(wx.Menu):
    """
    Timeline popup menu.  Allows user to manipulate the timeline view, and open new message views.
    """
    def __init__(self, parent, timeline):
        wx.Menu.__init__(self)

        self.parent   = parent
        self.timeline = timeline

        # Reset timeline
        self.reset_timeline_menu = wx.MenuItem(self, wx.NewId(), 'Reset Timeline')
        self.AppendItem(self.reset_timeline_menu)
        self.Bind(wx.EVT_MENU, lambda e: self.timeline.reset_timeline(), id=self.reset_timeline_menu.GetId())

        # Play all messages
        self.play_all_menu = wx.MenuItem(self, wx.NewId(), 'Play All Messages', kind=wx.ITEM_CHECK)
        self.AppendItem(self.play_all_menu)
        self.play_all_menu.Check(self.timeline.play_all)
        self.Bind(wx.EVT_MENU, lambda e: self.timeline.toggle_play_all(), id=self.play_all_menu.GetId())

        # ---
        self.AppendSeparator()

        renderers = self.timeline.get_renderers()

        # Thumbnails...
        if len(renderers) > 0:
            self.thumbnail_menu = wx.Menu()
            self.AppendSubMenu(self.thumbnail_menu, 'Thumbnails...', 'View message thumbnails')

            # Thumbnails... / Show All
            self.show_thumbnails_menu = wx.MenuItem(self.thumbnail_menu, wx.NewId(), 'Show All')
            self.thumbnail_menu.AppendItem(self.show_thumbnails_menu)
            self.thumbnail_menu.Bind(wx.EVT_MENU, lambda e: self.timeline.set_renderers_active(True), id=self.show_thumbnails_menu.GetId())
            
            # Thumbnails... / Hide All
            self.hide_thumbnails_menu = wx.MenuItem(self.thumbnail_menu, wx.NewId(), 'Hide All')
            self.thumbnail_menu.AppendItem(self.hide_thumbnails_menu)
            self.thumbnail_menu.Bind(wx.EVT_MENU, lambda e: self.timeline.set_renderers_active(False), id=self.hide_thumbnails_menu.GetId())
            
            # ---
            self.thumbnail_menu.AppendSeparator()
            
            # Thumbnails... / topic
            for topic, renderer in renderers:
                renderer_item = self.TimelineRendererMenuItem(self.thumbnail_menu, wx.NewId(), topic.lstrip('/'), topic, renderer, self.timeline)
                self.thumbnail_menu.AppendItem(renderer_item)

                renderer_item.Check(topic in self.timeline.rendered_topics)

        # View (by topic)...
        self.view_topic_menu = wx.Menu()
        self.AppendSubMenu(self.view_topic_menu, 'View (by Topic)...', 'View message detail')
        
        for topic in self.timeline.topics:
            datatype = self.timeline.get_datatype(topic)

            # View... / topic
            topic_menu = wx.Menu()
            self.view_topic_menu.AppendSubMenu(topic_menu, topic.lstrip('/'), topic)

            viewer_types = self.timeline.get_viewer_types(datatype)

            # View... / topic / Viewer
            for viewer_type in viewer_types:
                topic_menu.AppendItem(self.TopicViewMenuItem(topic_menu, wx.NewId(), viewer_type.name, topic, viewer_type, self.timeline))

        # View (by datatype)...
        self.view_datatype_menu = wx.Menu()
        self.AppendSubMenu(self.view_datatype_menu, 'View (by Type)...', 'View message detail')

        topics_by_datatype = self.timeline.topics_by_datatype
        
        for datatype in sorted(topics_by_datatype):
            # View... / datatype
            datatype_menu = wx.Menu()
            self.view_datatype_menu.AppendSubMenu(datatype_menu, datatype, datatype)
            
            topics = topics_by_datatype[datatype]
            
            viewer_types = self.timeline.get_viewer_types(datatype)
            
            for topic in [t for t in self.timeline.topics if t in topics]:   # use timeline ordering
                topic_menu = wx.Menu()
                datatype_menu.AppendSubMenu(topic_menu, topic.lstrip('/'), topic)
    
                # View... / datatype / topic / Viewer
                for viewer_type in viewer_types:
                    topic_menu.AppendItem(self.TopicViewMenuItem(topic_menu, wx.NewId(), viewer_type.name, topic, viewer_type, self.timeline))

        # ---
        self.AppendSeparator()

        # Publish...
        self.publish_menu = wx.Menu()
        self.AppendSubMenu(self.publish_menu, 'Publish...', 'Publish Messages')
        
        # Publish... / Publish All
        self.publish_menu.AppendItem(self.PublishAllMenuItem(self.publish_menu, wx.NewId(), 'Publish All',  True,  self.timeline))

        # Publish... / Publish None
        self.publish_menu.AppendItem(self.PublishAllMenuItem(self.publish_menu, wx.NewId(), 'Publish None', False, self.timeline))

        # ---
        self.publish_menu.AppendSeparator()

        for topic in self.timeline.topics:
            # Publish... / topic
            publish_topic_menu = self.PublishTopicMenuItem(self.publish_menu, wx.NewId(), topic, self.timeline)
            self.publish_menu.AppendItem(publish_topic_menu)
            publish_topic_menu.Check(self.timeline.is_publishing(topic))

    class TimelineRendererMenuItem(wx.MenuItem):
        def __init__(self, parent, id, label, topic, renderer, timeline):
            wx.MenuItem.__init__(self, parent, id, label, kind=wx.ITEM_CHECK)
            
            self.topic    = topic
            self.renderer = renderer
            self.timeline = timeline

            parent.Bind(wx.EVT_MENU, self.on_menu, id=self.GetId())

        def on_menu(self, event):
            self.timeline.set_renderer_active(self.topic, not self.timeline.is_renderer_active(self.topic))

    class TopicViewMenuItem(wx.MenuItem):
        def __init__(self, parent, id, label, topic, viewer_type, timeline):
            wx.MenuItem.__init__(self, parent, id, label)
            
            self.topic       = topic
            self.viewer_type = viewer_type
            self.timeline    = timeline
    
            parent.Bind(wx.EVT_MENU, self.on_menu, id=self.GetId())

        def on_menu(self, event):
            frame = BaseFrame(None, 'rxbag', self.topic, title='rxbag - %s [%s]' % (self.topic.lstrip('/'), self.viewer_type.name), pos=(4, 4), size=(640, 480))
            panel = LayerPanel(frame, -1)
            view  = self.viewer_type(self.timeline, panel, self.topic, 0, 0, *frame.GetClientSize())
            panel.layers = [view]
            frame.Show()
            
            self.timeline.add_view(self.topic, view)

    class PlayAllMenuItem(wx.MenuItem):
        def __init__(self, parent, id, label, timeline):
            wx.MenuItem.__init__(self, parent, id, label, kind=wx.ITEM_CHECK)
            
            self.timeline = timeline

            parent.Bind(wx.EVT_MENU, self.on_menu, id=self.GetId())

        def on_menu(self, event):
            self.timeline.play_all = not self.timeline.play_all

    class PublishTopicMenuItem(wx.MenuItem):
        def __init__(self, parent, id, topic, timeline):
            wx.MenuItem.__init__(self, parent, id, topic.lstrip('/'), kind=wx.ITEM_CHECK)
            
            self.topic    = topic
            self.timeline = timeline

            parent.Bind(wx.EVT_MENU, self.on_menu, id=self.GetId())

        def on_menu(self, event):
            if self.timeline.is_publishing(self.topic):
                self.timeline.stop_publishing(self.topic)
            else:
                self.timeline.start_publishing(self.topic)

    class PublishAllMenuItem(wx.MenuItem):
        def __init__(self, parent, id, label, start, timeline):
            wx.MenuItem.__init__(self, parent, id, label)
            
            self.timeline = timeline
            self.start    = start

            parent.Bind(wx.EVT_MENU, self.on_menu, id=self.GetId())

        def on_menu(self, event):
            if self.start:
                for topic in self.timeline.topics:
                    self.timeline.start_publishing(topic)
            else:
                for topic in self.timeline.topics:
                    self.timeline.stop_publishing(topic)
