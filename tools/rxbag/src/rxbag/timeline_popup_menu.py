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

from util.base_frame import BaseFrame

class TimelinePopupMenu(wx.Menu):
    """
    Timeline popup menu.  Allows user to manipulate the timeline view, and open new message views.
    """
    def __init__(self, timeline):
        wx.Menu.__init__(self)

        self.parent   = timeline
        self.timeline = timeline

        # Reset Timeline
        self.reset_timeline_menu = wx.MenuItem(self, wx.NewId(), 'Reset Timeline')
        self.AppendItem(self.reset_timeline_menu)
        self.Bind(wx.EVT_MENU, lambda e: self.timeline.reset_timeline(), id=self.reset_timeline_menu.GetId())

        # Play All Messages
        self.play_all_menu = wx.MenuItem(self, wx.NewId(), 'Play All Messages', kind=wx.ITEM_CHECK)
        self.AppendItem(self.play_all_menu)
        self.play_all_menu.Check(self.timeline.play_all)
        self.Bind(wx.EVT_MENU, lambda e: self.timeline.toggle_play_all(), id=self.play_all_menu.GetId())

        topics = self.timeline.topics

        if len(topics) == 0:
            return
            
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

            # Thumbnails... / ---
            self.thumbnail_menu.AppendSeparator()

            # Thumbnails... / topic
            for topic, renderer in renderers:
                renderer_item = self.TimelineRendererMenuItem(self.thumbnail_menu, wx.NewId(), topic.lstrip('/'), topic, renderer, self.timeline)
                self.thumbnail_menu.AppendItem(renderer_item)

                renderer_item.Check(self.timeline.is_renderer_active(topic))

        # View (by topic)...
        self.view_topic_menu = wx.Menu()
        self.AppendSubMenu(self.view_topic_menu, 'View (by Topic)...', 'View message detail')
        
        for topic in topics:
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
            
            datatype_topics = topics_by_datatype[datatype]
            
            viewer_types = self.timeline.get_viewer_types(datatype)
            
            for topic in [t for t in topics if t in datatype_topics]:   # use timeline ordering
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

        # Publish... / ---
        self.publish_menu.AppendSeparator()

        for topic in topics:
            # Publish... / topic
            publish_topic_menu = self.PublishTopicMenuItem(self.publish_menu, wx.NewId(), topic, self.timeline)
            self.publish_menu.AppendItem(publish_topic_menu)
            publish_topic_menu.Check(self.timeline.is_publishing(topic))

        # Copy region to bag...
        self.copy_region_menu = wx.MenuItem(self, wx.NewId(), 'Copy Region To Bag...')
        self.AppendItem(self.copy_region_menu)
        self.Bind(wx.EVT_MENU, lambda e: self.timeline.copy_region_to_bag(), id=self.copy_region_menu.GetId())
        if self.timeline.selected_left is None or self.timeline.selected_right is None:
            self.copy_region_menu.Enable(False)

    class PlayAllMenuItem(wx.MenuItem):
        def __init__(self, parent, id, label, timeline):
            wx.MenuItem.__init__(self, parent, id, label, kind=wx.ITEM_CHECK)
            
            self.timeline = timeline

            parent.Bind(wx.EVT_MENU, self.on_menu, id=self.GetId())

        def on_menu(self, event):
            self.timeline.play_all = not self.timeline.play_all

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
            view = self.viewer_type(self.timeline, frame)
            frame.Show()

            self.timeline.add_view(self.topic, view)

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
                    if not self.timeline.start_publishing(topic):
                        break
            else:
                for topic in self.timeline.topics:
                    self.timeline.stop_publishing(topic)
