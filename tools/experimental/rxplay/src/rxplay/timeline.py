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

from __future__ import with_statement

PKG = 'rxplay'
import roslib; roslib.load_manifest(PKG)
import rospy

import bisect
import collections
import math
import threading
import time

import wx

import sensor_msgs.msg

from bag_file import BagFile
import bag_index
import base_frame
import layer
import playhead
import status
import msg_views.image_view
import msg_views.raw_view
import msg_views.plot_view

class TimelinePanel(layer.LayerPanel):
    def __init__(self, input_files, options, *args, **kwargs):
        layer.LayerPanel.__init__(self, *args, **kwargs)

        bag_path   = input_files[0]
        index_path = bag_path + '.index'

        self._init_bag_files(bag_path, index_path)
        self._create_controls(options)
        self._create_toolbar()

    def _init_bag_files(self, bag_path, index_path):
        self.bag_file = BagFile(bag_path)
        
        self.bag_index = bag_index.BagIndexPickler(index_path).load()
        
        if self.bag_index:
            self.bag_file.read_datatype_defs(self.bag_index)
            return

        rospy.loginfo('Index not found - indexing...')
        
        self.bag_index_factory = bag_index.BagIndexFactory(bag_path)
        self.bag_index         = self.bag_index_factory.index

        # Background thread to generate, then save the index
        class BagIndexFactoryThread(threading.Thread):
            def __init__(self, bag_index_factory):
                threading.Thread.__init__(self)
                self.bag_index_factory = bag_index_factory

            def run(self):
                if self.bag_index_factory.load():
                    bag_index.BagIndexPickler(self.bag_index_factory.bag_path + '.index').save(self.bag_index_factory.index)

        self.bag_index_factory_thread = BagIndexFactoryThread(self.bag_index_factory)
        self.bag_index_factory_thread.start()

    def _create_controls(self, options):
        (width, height) = self.GetParent().GetClientSize()

        self.timeline = Timeline(self, 'Timeline', 5, 22, width, height, show_thumbnails=options.show_thumbnails, thumbnail_height=64, max_repaint=1.0)
        self.timeline.set_bag_file(self.bag_file, self.bag_index)

        self.status = status.StatusLayer(self, 'Status', self.timeline, self.timeline.x, 0, width, 20)

        self.playhead = playhead.PlayheadLayer(self, 'Playhead', self.timeline, 0, 0, 12, self.timeline.height)

        self.layers = [self.timeline, self.status, self.playhead]

    def _create_toolbar(self):
        icons_dir = roslib.packages.get_pkg_dir(PKG) + '/icons/'

        start_bitmap       = wx.Bitmap(icons_dir + 'control_start_blue.png')
        rewind_bitmap      = wx.Bitmap(icons_dir + 'control_rewind_blue.png')
        play_bitmap        = wx.Bitmap(icons_dir + 'control_play_blue.png')
        fastforward_bitmap = wx.Bitmap(icons_dir + 'control_fastforward_blue.png')
        end_bitmap         = wx.Bitmap(icons_dir + 'control_end_blue.png')
        stop_bitmap        = wx.Bitmap(icons_dir + 'control_stop_blue.png')
        zoom_in_bitmap     = wx.Bitmap(icons_dir + 'zoom_in.png')
        zoom_out_bitmap    = wx.Bitmap(icons_dir + 'zoom_out.png')
        zoom_bitmap        = wx.Bitmap(icons_dir + 'zoom.png')

        toolbar = self.GetParent().CreateToolBar()
        
        start_tool       = toolbar.AddLabelTool(wx.ID_ANY, '', start_bitmap)
        rewind_tool      = toolbar.AddLabelTool(wx.ID_ANY, '', rewind_bitmap)
        play_tool        = toolbar.AddLabelTool(wx.ID_ANY, '', play_bitmap)
        fastforward_tool = toolbar.AddLabelTool(wx.ID_ANY, '', fastforward_bitmap)
        end_tool         = toolbar.AddLabelTool(wx.ID_ANY, '', end_bitmap)
        stop_tool        = toolbar.AddLabelTool(wx.ID_ANY, '', stop_bitmap)
        toolbar.AddSeparator()
        zoom_in_tool     = toolbar.AddLabelTool(wx.ID_ANY, '', zoom_in_bitmap)
        zoom_out_tool    = toolbar.AddLabelTool(wx.ID_ANY, '', zoom_out_bitmap)
        zoom_tool        = toolbar.AddLabelTool(wx.ID_ANY, '', zoom_bitmap)

        toolbar.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_start(),       start_tool)       
        toolbar.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_rewind(),      rewind_tool)
        toolbar.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_play(),        play_tool)       
        toolbar.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_fastforward(), fastforward_tool)       
        toolbar.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_end(),         end_tool)
        toolbar.Bind(wx.EVT_TOOL, lambda e: self.timeline.navigate_stop(),        stop_tool)
        toolbar.Bind(wx.EVT_TOOL, lambda e: self.timeline.zoom_in(),              zoom_in_tool)
        toolbar.Bind(wx.EVT_TOOL, lambda e: self.timeline.zoom_out(),             zoom_out_tool)
        toolbar.Bind(wx.EVT_TOOL, lambda e: self.timeline.reset_zoom(),           zoom_tool)

        toolbar.Realize()

class Timeline(layer.Layer):
    name = 'Timeline'
    
    def __init__(self, parent, title, x, y, width, height, show_thumbnails=False, thumbnail_height=48, max_repaint=None):
        layer.Layer.__init__(self, parent, title, x, y, width, height, max_repaint)

        self.bag_file  = None
        self.bag_index = None

        ## Rendering parameters

        self.background_brush   = wx.Brush('white')
        self.topic_divider_pen  = wx.Pen('#dddddd', 1)
        self.topic_font_color   = wx.Colour(0, 0, 0)
        self.time_font_color    = wx.Colour(0, 0, 0)
        self.time_label_spacing = 6
        self.time_major_pen     = wx.Pen('#222222', 1, wx.SHORT_DASH)
        self.time_minor_pen     = wx.Pen('#aaaaaa', 1, wx.LONG_DASH)
        self.time_tick_pen      = wx.Pen('#888888', 1, wx.SHORT_DASH)
        self.time_tick_height   = 3

        self.sec_divisions = [0.001, 0.005, 0.01, 0.05, 0.1, 0.5,                                 # 1ms, 5ms, 10ms, 50ms, 100ms, 500ms
                              1, 5, 15, 30,                                                       # 1s, 5s, 15s, 30s
                              1 * 60, 2 * 60, 5 * 60, 10 * 60, 15 * 60, 30 * 60,                  # 1m, 2m, 5m, 10m, 15m, 30m
                              1 * 60 * 60, 2 * 60 * 60, 3 * 60 * 60, 6 * 60 * 60, 12 * 60 * 60,   # 1h, 2h, 3h, 6h, 12h
                              1 * 60 * 60 * 24, 7 * 60 * 60 * 24]                                 # 1d, 7d
        self.minor_spacing = 15
        self.major_spacing = 50

        self.topic_font        = wx.Font(9, wx.FONTFAMILY_SCRIPT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.topic_font_height = None
        self.topic_name_sizes  = None
        self.margin_left       = 2
        self.margin_right      = 15

        self.time_font        = wx.Font(9, wx.FONTFAMILY_SCRIPT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.time_font_height = None
        
        self.history_top        = 26
        self.history_border_pen = wx.Pen('black', 1)

        self.bag_end_width = 4
        self.bag_end_pen   = wx.Pen('black', 1)
        
        self.default_datatype_color = wx.Colour(0, 0, 0)
        self.datatype_colors = {
            'sensor_msgs/CameraInfo':            wx.Colour(  0,   0, 150),
            'sensor_msgs/Image':                 wx.Colour(  0,  80,  80),
            'sensor_msgs/LaserScan':             wx.Colour(150,   0,   0),
            'pr2_msgs/LaserScannerSignal':       wx.Colour(150,   0,   0),
            'pr2_mechanism_msgs/MechanismState': wx.Colour(  0, 150,   0),
            'tf/tfMessage':                      wx.Colour(  0, 150,   0),
        }
        self.msg_combine_px = 1.5   # don't draw discrete messages if they're less than this many pixels separated 

        self.zoom_sensitivity = 0.005
        self.min_zoom_speed   = 0.5
        self.max_zoom_speed   = 2.0
        
        self.viewer_types = {
            'sensor_msgs/Image':      [msg_views.image_view.ImageView],
            'sensor_msgs/JointState': [msg_views.plot_view.PlotView],
            'tf/tfMessage':           [msg_views.plot_view.PlotView],
        }

        self.image_timeline_renderer = msg_views.image_view.ImageTimelineRenderer(self, show_thumbnails, thumbnail_height)

        self.timeline_renderers = {
            'sensor_msgs/Image': self.image_timeline_renderer,
        }

        ##

        self.clicked_pos = None

        self.history_left   =  0
        self.history_height =  0
        self.history_width  =  0
        self.history_bounds = {}

        self.stamp_left    = None
        self.stamp_right   = None
        self.playhead      = None
        self.playhead_lock = threading.Lock()
        
        self.listeners   = {}

        self.playhead_indices = None

        class PlayThread(threading.Thread):
            def __init__(self, timeline):
                threading.Thread.__init__(self)
                
                self.setDaemon(True)
                self.timeline = timeline
                
            def run(self):
                last_frame, last_playhead = None, None
                
                while True:
                    if self.timeline.play_speed == 0.0:
                        last_frame    = None
                        last_playhead = None
                        
                        time.sleep(0.5)
                        continue
                        
                    now = time.time()
                    if last_frame and self.timeline.playhead == last_playhead:
                        new_playhead = self.timeline.playhead + (now - last_frame) * self.timeline.play_speed

                        start_stamp = self.timeline.bag_index._data.get_start_stamp()
                        end_stamp   = self.timeline.bag_index._data.get_end_stamp()
                        if new_playhead > end_stamp:
                            new_playhead = start_stamp
                        elif new_playhead < start_stamp:
                            new_playhead = end_stamp

                        self.timeline.set_playhead(new_playhead)

                    last_frame    = now
                    last_playhead = self.timeline.playhead
                    
                    time.sleep(0.08)

        self.play_speed = 0.0
        self.play_thread = PlayThread(self)
        self.play_thread.start()
        
    def set_bag_file(self, bag_file, bag_index):
        self.bag_file  = bag_file
        self.bag_index = bag_index        

    def add_listener(self, topic, listener):
        self.listeners.setdefault(topic, []).append(listener)

        self.update_message_view()

    @property
    def topics(self):
        return sorted(self.bag_index.topics)

    @property
    def history_bottom(self):
        return self.history_top + self.history_height
    
    @property
    def history_right(self):
        return self.history_left + self.history_width

    def _set_status(self, status):
        self.parent.GetParent().SetStatusText(str(status))

    ## Navigation

    def navigate_play(self): self.play_speed = 1.0
    def navigate_stop(self): self.play_speed = 0.0

    def navigate_rewind(self):
        if self.play_speed <= -1.0:
            self.play_speed *= 2.0
        elif self.play_speed < 0.25:
            self.play_speed = -1.0
        else:
            self.play_speed *= 0.5
        
    def navigate_fastforward(self):
        if self.play_speed >= 1.0:
            self.play_speed *= 2.0
        elif self.play_speed > -0.25:
            self.play_speed = 1.0
        else:
            self.play_speed *= 0.5

    def navigate_start(self): self.set_playhead(self.bag_index._data.get_start_stamp())
    def navigate_end(self):   self.set_playhead(self.bag_index._data.get_end_stamp())

    ## View port

    def reset_timeline(self):
        self.reset_zoom() 
        self.set_playhead(self.stamp_left)
        
    def set_timeline_view(self, stamp_left, stamp_right):
        self.stamp_left  = stamp_left
        self.stamp_right = stamp_right

        self.force_repaint()
    
    def translate_timeline(self, dx):
        dstamp = self.map_dx_to_dstamp(dx)

        self.stamp_left  -= dstamp
        self.stamp_right -= dstamp

        self.force_repaint()

    def reset_zoom(self):
        self.set_timeline_view(self.bag_index._data.get_start_stamp(), self.bag_index._data.get_end_stamp())

    def zoom_in(self):  self.zoom_timeline(0.5)
    def zoom_out(self): self.zoom_timeline(2.0)

    def zoom_timeline(self, zoom):
        new_stamp_width   = zoom * (self.stamp_right - self.stamp_left)
        playhead_fraction = (self.playhead - self.stamp_left) / (self.stamp_right - self.stamp_left)
        self.stamp_left   = self.playhead - playhead_fraction * new_stamp_width
        self.stamp_right  = self.stamp_left + new_stamp_width 

        self._layout()

        self.force_repaint()
    
    def set_playhead(self, playhead):
        with self.playhead_lock:
            self.playhead = playhead  
            
            if self.playhead > self.stamp_right:
                end_stamp = self.bag_index._data.get_end_stamp()
    
                dstamp = self.playhead - self.stamp_right + (self.stamp_right - self.stamp_left) * 0.75
                dstamp = min(dstamp, end_stamp - self.stamp_right)
                
                self.stamp_left  += dstamp
                self.stamp_right += dstamp
                
                self.invalidate()
            elif self.playhead < self.stamp_left:
                start_stamp = self.bag_index._data.get_start_stamp()
    
                dstamp = self.stamp_left - self.playhead + (self.stamp_right - self.stamp_left) * 0.75
                dstamp = min(dstamp, self.stamp_left - start_stamp)
    
                self.stamp_left  -= dstamp
                self.stamp_right -= dstamp
                self.invalidate()
    
            # Get the indices in each topic msg_positions corresponding to the timestamp
            self.playhead_indices = dict([(topic, self.bag_index._data.find_stamp_index(topic, self.playhead)) for topic in self.topics])
    
            self.update_message_view()
            
            self.parent.status.invalidate()
            self.parent.playhead.update_position()

    def toggle_thumbnails(self):
        self.image_timeline_renderer.show_thumbnails = not self.image_timeline_renderer.show_thumbnails

        self.force_repaint()
        self.parent.playhead.force_repaint()

    ### Rendering

    def on_size(self, event):
        self.resize(*self.parent.GetClientSize())

    def check_dirty(self):
        if not self.bag_index.loaded:
            self.invalidate()

    def paint(self, dc):
        dc.SetBackground(self.background_brush)
        dc.Clear()

        if len(self.topics) == 0:
            return

        if not self.stamp_left:
            if self.bag_index.loaded:
                self.reset_timeline()
            else:
                start_stamp = self.bag_index._data.get_start_stamp()
                
                self.set_timeline_view(start_stamp, start_stamp + (30 * 60))  # default to showing 30 mins if index not created yet
                self.set_playhead(self.stamp_left)

        if not self.stamp_left or not self.stamp_right:
            return

        self._calc_font_sizes(dc)
        self._layout()

        self._draw_topic_dividers(dc)
        self._draw_time_indicators(dc)
        self._draw_message_history(dc)
        self._draw_bag_ends(dc)
        self._draw_topic_names(dc)
        self._draw_history_border(dc)

    def _calc_font_sizes(self, dc):
        dc.SetFont(self.topic_font)
        self.topic_name_sizes = dict([(topic, dc.GetTextExtent(topic)) for topic in self.topics])

        dc.SetFont(self.time_font)
        self.time_font_height = dc.GetTextExtent('12:34')[1]

    def _layout(self):
        max_topic_name_width   = max([w for (w, h) in self.topic_name_sizes.values()])
        self.topic_font_height = max([h for (w, h) in self.topic_name_sizes.values()])

        self.history_left  = self.margin_left + max_topic_name_width + 6
        self.history_width = self.width - self.history_left - self.margin_right

        self.history_bounds = {}
        y = self.history_top
        for topic in self.topics:
            datatype = self.bag_index.get_datatype(topic)
            
            topic_height = None
            if datatype in self.timeline_renderers:
                topic_height = self.timeline_renderers[datatype].get_segment_height(topic)
            if not topic_height:
                topic_height = self.topic_font_height
            
            self.history_bounds[topic] = (self.history_left, y, self.history_width, topic_height)

            y += topic_height - 2

        self.history_bottom = max([y + h for (x, y, w, h) in self.history_bounds.values()])

    def _draw_topic_dividers(self, dc):
        clip_left  = self.history_left
        clip_right = self.history_left + self.history_width

        dc.SetPen(self.topic_divider_pen)
        dc.DrawLineList([(max(clip_left, x), y, min(clip_right, x + w), y) for (x, y, w, h) in self.history_bounds.values()])

    ## Draws time indicators on the timeline 
    def _draw_time_indicators(self, dc):
        x_per_sec = self.map_dstamp_to_dx(1.0)

        major_divisions = [s for s in self.sec_divisions if x_per_sec * s >= self.major_spacing]
        if len(major_divisions) == 0:
            major_division = max(self.sec_divisions)
        else:
            major_division = min(major_divisions)

        minor_divisions = [s for s in self.sec_divisions if x_per_sec * s >= self.minor_spacing and major_division % s == 0]
        if len(minor_divisions) > 0:
            minor_division = min(minor_divisions)
        else:
            minor_division = None

        start_stamp = self.bag_index._data.get_start_stamp()

        major_stamps = list(self._get_stamps(start_stamp, major_division))
        self._draw_major_divisions(dc, major_stamps, start_stamp, major_division)
        if minor_division:
            minor_stamps = [s for s in self._get_stamps(start_stamp, minor_division) if s not in major_stamps]
            self._draw_minor_divisions(dc, minor_stamps, start_stamp, minor_division)

    def _draw_major_divisions(self, dc, stamps, start_stamp, division):
        dc.SetPen(self.time_major_pen)
        dc.SetFont(self.time_font)
        dc.SetTextForeground(self.time_font_color)

        for stamp in stamps:
            x = self.map_stamp_to_x(stamp)
            label_y = self.history_top - self.time_font_height - self.time_label_spacing
            dc.DrawText(self._get_label(division, stamp - start_stamp), x + 3, label_y)

            dc.DrawLine(x, label_y + 1, x, self.history_bottom)

    def _draw_minor_divisions(self, dc, stamps, start_stamp, division):
        xs = [self.map_stamp_to_x(stamp) for stamp in stamps]

        dc.SetPen(self.time_tick_pen)
        dc.DrawLineList([(x, self.history_top - self.time_tick_height, x, self.history_top) for x in xs])

        dc.SetPen(self.time_minor_pen)
        dc.DrawLineList([(x, self.history_top, x, self.history_bottom) for x in xs])

    ## Returns visible stamps every stamp_step 
    def _get_stamps(self, start_stamp, stamp_step):
        stamp = start_stamp
        while True:
            if stamp >= self.stamp_left:
                if stamp > self.stamp_right:
                    break

                yield stamp

            stamp += stamp_step

    def _get_label(self, division, elapsed):
        secs  = int(elapsed) % 60

        mins  = int(elapsed) / 60
        hrs   = mins / 60
        days  = hrs / 24
        weeks = days / 7
        
        if division >= 7 * 24 * 60 * 60:    # >1wk divisions: show weeks
            return '%dw' % weeks
        elif division >= 24 * 60 * 60:      # >24h divisions: show days
            return '%dd' % days
        elif division >= 60 * 60:           # >1h divisions: show hours 
            return '%dh' % hrs
        elif division >= 5 * 60:            # >5m divisions: show minutes
            return '%dm' % mins
        elif division >= 1:                 # >1s divisions: show minutes:seconds
            return '%d:%02d' % (mins, secs)
        elif division >= 0.1:               # >0.1s divisions: show seconds.0
            return '%d.%s' % (secs, str(int(10.0 * (elapsed - int(elapsed)))))
        elif division >= 0.01:              # >0.1s divisions: show seconds.0
            return '%d.%02d' % (secs, int(100.0 * (elapsed - int(elapsed))))
        else:                               # show seconds.00
            return '%d.%03d' % (secs, int(1000.0 * (elapsed - int(elapsed))))

    ## Draw boxes to show messages regions in timelines
    def _draw_message_history(self, dc):
        msg_combine_interval = self.map_dx_to_dstamp(self.msg_combine_px)

        for topic, (x, y, w, h) in self.history_bounds.items():
            msg_y      = y + 1
            msg_height = h - 3
            
            datatype = self.bag_index.get_datatype(topic)
            
            # Set pen based on datatype
            if datatype in self.datatype_colors:
                datatype_color = self.datatype_colors[datatype]
            else:
                datatype_color = self.default_datatype_color
            dc.SetPen(wx.Pen(datatype_color))
            dc.SetBrush(wx.Brush(datatype_color))

            # Get custom timeline renderer (if any)
            if datatype in self.timeline_renderers:
                renderer = self.timeline_renderers[datatype]
            else:
                renderer = None
            
            # Find the index of the earliest visible stamp
            stamp_left_index  = self.bag_index._data.find_stamp_index(topic, self.stamp_left)
            stamp_right_index = self.bag_index._data.find_stamp_index(topic, self.stamp_right)

            # Iterate through regions of connected messages
            for (stamp_start, stamp_end) in self._find_regions((stamp for (stamp, pos) in self.bag_index._data.msg_positions[topic][stamp_left_index:stamp_right_index]), msg_combine_interval):
                # Calculate region rectangle bounds 
                region_x_start = self.map_stamp_to_x(stamp_start)
                region_x_end   = self.map_stamp_to_x(stamp_end)
                region_width   = max(1, region_x_end - region_x_start) 
                region_rect    = (region_x_start, msg_y, region_width, msg_height)

                # Use custom datatype renderer
                if renderer and renderer.draw_timeline_segment(dc, topic, stamp_start, stamp_end, *region_rect):
                    continue

                # Use default renderer
                dc.DrawRectangle(*region_rect)

    ## Groups timestamps into regions connected by timestamps less than max_interval secs apart
    def _find_regions(self, stamps, max_interval):
        region_start, prev_stamp = None, None
        for stamp in stamps:
            if prev_stamp:
                if stamp - prev_stamp > max_interval:
                    region_end = prev_stamp
                    yield (region_start, region_end)
                    region_start = stamp
            else:
                region_start = stamp

            prev_stamp = stamp

        if region_start and prev_stamp:
            yield (region_start, prev_stamp)

    ## Draws centered topic names
    def _draw_topic_names(self, dc):
        topics = self.history_bounds.keys()
        coords = [(self.margin_left, y + (h / 2) - (self.topic_font_height / 2)) for (x, y, w, h) in self.history_bounds.values()]

        dc.SetFont(self.topic_font)
        dc.SetTextForeground(self.topic_font_color)
        dc.DrawTextList(topics, coords)

    ## Draw markers to indicate the extent of the bag file
    def _draw_bag_ends(self, dc):
        dc.SetPen(self.bag_end_pen)

        marker_top, marker_bottom = self.history_top - 2, self.history_bottom + 2

        # Draw start marker
        start_stamp = self.bag_index._data.get_start_stamp()
        if start_stamp > self.stamp_left and start_stamp < self.stamp_right:
            x = self.map_stamp_to_x(start_stamp)
            dc.DrawLineList([(x - i, marker_top, x - i, marker_bottom) for i in range(1, self.bag_end_width)])

        # Draw end marker
        end_stamp = self.bag_index._data.get_end_stamp()
        if end_stamp > self.stamp_left and end_stamp < self.stamp_right:
            x = self.map_stamp_to_x(end_stamp)
            dc.DrawLineList([(x + i, marker_top, x + i, marker_bottom) for i in range(1, self.bag_end_width)])

    def _draw_history_border(self, dc):
        dc.SetPen(self.history_border_pen)
        dc.SetBrush(wx.Brush('white', wx.TRANSPARENT))
        bounds_width = min(self.history_width, self.parent.width - self.x)
        dc.DrawRectangle(self.history_left, self.history_top, bounds_width, self.history_bottom - self.history_top)

    def map_dx_to_dstamp(self, dx):
        return float(dx) * (self.stamp_right - self.stamp_left) / self.history_width

    def map_x_to_stamp(self, x):
        fraction = float(x - self.history_left) / self.history_width

        if fraction <= 0.0:
            return self.stamp_left
        elif fraction >= 1.0:
            return self.stamp_right

        return self.stamp_left + fraction * (self.stamp_right - self.stamp_left)

    def map_dstamp_to_dx(self, dstamp):
        return (float(dstamp) * self.history_width) / (self.stamp_right - self.stamp_left)

    def map_stamp_to_x(self, stamp, clamp_to_visible=True):
        fraction = (stamp - self.stamp_left) / (self.stamp_right - self.stamp_left)

        if clamp_to_visible:
            fraction = min(1.0, max(0.0, fraction))

        return self.history_left + fraction * self.history_width

    ### Control

    # Mouse events

    def on_left_down(self, event):
        self.clicked_pos = event.GetPosition()
        if not self.contains(*self.clicked_pos):
            return

        if event.ShiftDown():
            return

        x, y = self.clicked_pos[0] - self.x, self.clicked_pos[1] - self.y
        if x < self.history_left or x > self.history_left + self.history_width:
            return

        self.set_playhead(self.map_x_to_stamp(x))  

    def on_middle_down(self, event):
        self.clicked_pos = event.GetPosition()

    def on_right_down(self, event):
        self.clicked_pos = event.GetPosition()
        if not self.contains(*self.clicked_pos):
            return

        self.parent.PopupMenu(TimelinePopupMenu(self.parent, self), self.clicked_pos)

    def on_mousewheel(self, event):
        dz = event.GetWheelRotation() / event.GetWheelDelta()
        self.zoom_timeline(1.0 - dz * 0.2)
        
    def on_mouse_move(self, event):
        mouse_pos = event.GetPosition()

        if not event.Dragging():
            return

        left, middle, right = event.LeftIsDown(), event.MiddleIsDown(), event.RightIsDown()

        if middle or event.ShiftDown():
            x, y = mouse_pos

            dx, dy = x - self.clicked_pos[0], y - self.clicked_pos[1]

            if dx != 0:
                self.translate_timeline(dx)
            if dy != 0:
                zoom = min(self.max_zoom_speed, max(self.min_zoom_speed, 1.0 + self.zoom_sensitivity * dy))
                self.zoom_timeline(zoom)

            self.clicked_pos = mouse_pos

            self.parent.playhead.update_position()

        elif left:
            x, y = mouse_pos[0] - self.x, mouse_pos[1] - self.y
            
            self.set_playhead(self.map_x_to_stamp(x))

            self.clicked_pos = mouse_pos
            
    ##
    
    def load_msg_by_stamp(self, topic, stamp):
        index = topic.find_stamp_index(stamp)
        if not index:
            return None

        pos = topic.msg_positions[index][1]

    def update_message_view(self):
        if not self.playhead_indices:
            return

        msgs = {}
        for topic in self.playhead_indices:
            if topic in self.listeners:
                playhead_index = self.playhead_indices[topic]
                if playhead_index is not None:
                    # Load the message
                    pos = self.bag_index.msg_positions[topic][playhead_index][1]
                    (datatype, msg, stamp) = self.bag_file.load_message(pos, self.bag_index)
                    
                    msgs[topic] = (stamp, datatype, playhead_index, msg)
                    continue
                
            msgs[topic] = None

        # Inform the listeners
        for topic, msg_data in msgs.items():
            topic_listeners = self.listeners.get(topic)
            if not topic_listeners:
                continue
            
            if msg_data:
                for listener in topic_listeners:
                    listener.message_viewed(self.bag_file, self.bag_index, topic, *msg_data)
            else:
                for listener in topic_listeners:
                    listener.message_cleared()

    @staticmethod
    def stamp_to_str(secs):
        secs_frac     = secs - int(secs) 
        secs_frac_str = ('%.2f' % secs_frac)[1:]

        return time.strftime('%b %d %Y %H:%M:%S', time.localtime(secs)) + secs_frac_str

## Timeline popup menu.  Allows user to manipulate the timeline view, and open new message views.
class TimelinePopupMenu(wx.Menu):
    def __init__(self, parent, timeline):
        wx.Menu.__init__(self)

        self.parent   = parent
        self.timeline = timeline

        # Reset Timeline
        self.reset_timeline_menu = wx.MenuItem(self, wx.NewId(), 'Reset Timeline')
        self.AppendItem(self.reset_timeline_menu)
        self.Bind(wx.EVT_MENU, self.on_reset_timeline_menu, id=self.reset_timeline_menu.GetId())

        # Show Thumbnails
        self.show_thumbnails_menu = wx.MenuItem(self, wx.NewId(), 'Show Thumbnails', kind=wx.ITEM_CHECK)
        self.AppendItem(self.show_thumbnails_menu)
        self.Bind(wx.EVT_MENU, self.on_show_thumbnails_menu, id=self.show_thumbnails_menu.GetId())
        self.show_thumbnails_menu.Check(self.timeline.image_timeline_renderer.show_thumbnails)

        # View
        self.view_menu = wx.Menu()
        self.AppendSubMenu(self.view_menu, 'View...', 'View message detail')
        
        for topic in self.timeline.topics:
            topic_menu = wx.Menu()
            self.view_menu.AppendSubMenu(topic_menu, topic, topic)

            datatype = self.parent.bag_index.get_datatype(topic)

            viewer_types = [msg_views.raw_view.RawView]
            if datatype in self.timeline.viewer_types:
                viewer_types.extend(self.timeline.viewer_types[datatype])

            for viewer_type in viewer_types:
                topic_menu.AppendItem(self.TopicViewMenuItem(topic_menu, wx.NewId(), viewer_type.name, topic, viewer_type, self.timeline))

    def on_reset_timeline_menu(self, event):  self.timeline.reset_timeline()
    def on_show_thumbnails_menu(self, event): self.timeline.toggle_thumbnails()

    class TopicViewMenuItem(wx.MenuItem):
        def __init__(self, parent, id, label, topic, viewer_type, timeline):
            wx.MenuItem.__init__(self, parent, id, label)
            
            self.topic       = topic
            self.viewer_type = viewer_type
            self.timeline    = timeline
    
            parent.Bind(wx.EVT_MENU, self.on_menu, id=self.GetId())

        def on_menu(self, event):
            frame = base_frame.BaseFrame(None, 'rxplay', self.topic, title='rxplay - %s [%s]' % (self.topic, self.viewer_type.name), pos=(4, 4), size=(640, 480))
            panel = layer.LayerPanel(frame, -1)
            size = frame.GetClientSize()
            view = self.viewer_type(self.timeline, panel, self.topic, 0, 0, size[0], size[1])
            panel.layers = [view]
            frame.Show()

            self.timeline.add_listener(self.topic, view)
