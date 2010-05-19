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

PKG = 'rxbag'
import roslib; roslib.load_manifest(PKG)
import rospy

import rosbag
import rosgraph.masterapi

import bisect
import collections
import math
import os
import sys
import threading
import time

import cairo
import wx

import bag_helper
import playhead
import plugins
from raw_view   import RawView
from recorder   import Recorder
from util.layer import Layer

class IndexCacheThread(threading.Thread):
    def __init__(self, timeline, period=0.5):
        threading.Thread.__init__(self)

        self.setDaemon(True)
        
        self.timeline  = timeline
        self.period    = period
        self.stop_flag = False
        
        self.start()

    def run(self):
        while not self.stop_flag:
            topics = self.timeline.topics
            if len(topics) == 0:
                time.sleep(1.0)
                continue
            
            with self.timeline.index_cache_lock:
                for topic in topics:
                    self.timeline._update_index_cache(topic)

            wx.CallAfter(self.timeline.invalidate)

            time.sleep(self.period)

    def stop(self): self.stop_flag = True

class PlayThread(threading.Thread):
    def __init__(self, timeline):
        threading.Thread.__init__(self)

        self.setDaemon(True)

        self.timeline  = timeline
        self.stop_flag = False
        
        self.fixed_step_mode = False
        
        self.start()

    def run(self):
        self.last_frame    = None
        self.last_playhead = None

        while not self.stop_flag:
            if self.fixed_step_mode:
                wx.CallAfter(self.step_fixed)
                time.sleep(0.04)  # 25 Hz
            else:
                self.step_next()

    def step_fixed(self):
        if self.timeline.play_speed == 0.0:
            self.last_frame    = None
            self.last_playhead = None
            return

        now = time.time()
        if self.last_frame and self.timeline.playhead == self.last_playhead:
            new_playhead = self.timeline.playhead + (now - self.last_frame) * self.timeline.play_speed

            start_stamp = self.timeline.start_stamp
            end_stamp   = self.timeline.end_stamp
            if new_playhead > end_stamp:
                new_playhead = start_stamp
            elif new_playhead < start_stamp:
                new_playhead = end_stamp

            self.timeline.set_playhead(new_playhead)

        self.last_frame    = now
        self.last_playhead = self.timeline.playhead

    def step_next(self):
        if self.timeline.play_speed <= 0.0:
            self.last_frame    = None
            self.last_playhead = None
            return
        
        now = time.time()
        if self.last_frame and self.timeline.playhead == self.last_playhead:
            next_message_time = self.get_next_message_time()
            if next_message_time is not None:
                if self.last_playhead is None:
                    new_playhead = next_message_time
                else:
                    if next_message_time > self.last_playhead:
                        delta = (next_message_time - self.last_playhead) * self.timeline.play_speed
                        if delta > 0.05:
                            delta = 0.05
                        time.sleep(delta)
    
                        new_playhead = self.last_playhead + delta
                    else:
                        delta = (self.timeline.end_stamp - self.last_playhead) * self.timeline.play_speed
                        if delta > 0.05:
                            delta = 0.05
                            time.sleep(delta)
                            new_playhead = self.last_playhead + delta
                        else:
                            time.sleep(delta)
                            new_playhead = next_message_time

                self.timeline.set_playhead(new_playhead)
            else:
                time.sleep(0.5)

        self.last_frame    = now
        self.last_playhead = self.timeline.playhead

    def get_next_message_time(self):
        if not self.timeline.playhead:
            return None

        # NOTE: add an epsilon due to imperfect conversion from roslib.Time <-> float
        playhead_time = roslib.rostime.Time.from_sec(self.timeline.playhead + 0.000001)

        _, entry = self.timeline.get_entry_after(playhead_time)
        if entry is None:
            return self.timeline.start_stamp

        return entry.time.to_sec()

    def stop(self): self.stop_flag = True

class Timeline(Layer):
    name = 'Timeline'
    
    def __init__(self, parent, title, x, y, width, height):
        Layer.__init__(self, parent, title, x, y, width, height)

        self._bag_lock = threading.RLock()
        self.bags = []

        ## Rendering parameters

        self.sec_divisions = [0.001, 0.005, 0.01, 0.05, 0.1, 0.5,                                 # 1ms, 5ms, 10ms, 50ms, 100ms, 500ms
                              1, 5, 15, 30,                                                       # 1s, 5s, 15s, 30s
                              1 * 60, 2 * 60, 5 * 60, 10 * 60, 15 * 60, 30 * 60,                  # 1m, 2m, 5m, 10m, 15m, 30m
                              1 * 60 * 60, 2 * 60 * 60, 3 * 60 * 60, 6 * 60 * 60, 12 * 60 * 60,   # 1h, 2h, 3h, 6h, 12h
                              1 * 60 * 60 * 24, 7 * 60 * 60 * 24]                                 # 1d, 7d
        self.minor_spacing = 15
        self.major_spacing = 50

        self.time_font_height  = None
        self.topic_font_height = None
        self.topic_name_sizes  = None
        self.margin_left       = 0
        self.margin_right      = 20
        self.history_top       = 24
        self.bag_end_width     = 3
        self.time_tick_height  = 3
        
        self.minor_divisions_color_tick = (0.5, 0.5, 0.5, 0.3)
        self.minor_divisions_color      = (0.6, 0.6, 0.6, 0.3)

        self.default_datatype_color = (0.0, 0.0, 0.0)
        self.datatype_colors = {
            'sensor_msgs/CameraInfo':            (  0,   0, 0.6),
            'sensor_msgs/Image':                 (  0, 0.3, 0.3),
            'sensor_msgs/LaserScan':             (0.6,   0,   0),
            'pr2_msgs/LaserScannerSignal':       (0.6,   0,   0),
            'pr2_mechanism_msgs/MechanismState': (  0, 0.6,   0),
            'tf/tfMessage':                      (  0, 0.6,   0),
        }
        self.default_msg_combine_px = 1.0

        self.zoom_sensitivity = 0.005
        self.min_zoom_speed   = 0.5
        self.max_zoom_speed   = 2.0
        self.min_zoom         = 0.0001      # max zoom out (in px/s)
        self.max_zoom         = 50000.0     # max zoom in  (in px/s)

        self.max_play_speed =  1024.0
        self.min_play_speed = -1024.0

        self.viewer_types       = {}
        self.timeline_renderers = {}
        self.rendered_topics    = set()

        self.last_invalidated = 0.0
        
        self.load_plugins()

        ##

        self.clicked_pos = None
        self.dragged_pos = None

        self.history_left   = 0
        self.history_height = 0
        self.history_width  = 0
        self.history_bounds = {}

        self.stamp_left    = None
        self.stamp_right   = None
        self.playhead      = None
        self.playhead_lock = threading.RLock()
        
        self.play_speed         = 0.0
        self.playhead_positions = None

        self.views = []

        self.listeners = {}

        self.index_cache_lock   = threading.RLock()
        self.index_cache        = {}
        self.invalidated_caches = set()
        
        self.recorder = None

        ##

        self.play_thread        = PlayThread(self)
        self.index_cache_thread = IndexCacheThread(self)
        
    ###

    def invalidate(self):
        Layer.invalidate(self)
        self.last_invalidated = time.time()

    def message_recorded(self, topic, msg, t):
        # Invalidate the topic
        self.invalidated_caches.add(topic)

        # Periodically invalidate the timeline
        now = time.time()
        if now - self.last_invalidated > 2.0:
            wx.CallAfter(self.invalidate)

    def record_bag(self, filename):
        try:
            self.recorder = Recorder(filename, bag_lock=self._bag_lock)
        except Exception, ex:
            print >> sys.stderr, 'Error opening bag for recording [%s]: %s' % (self._filename, str(ex))
            return

        self.recorder.add_listener(self.message_recorded)
        
        self.bags.append(self.recorder.bag)
        
        self.recorder.start()

    def add_bag(self, bag):
        self.bags.append(bag)

        # Invalidate entire index cache for all topics in this bag
        with self.index_cache_lock:
            for topic in bag_helper.get_topics(bag):
                if topic in self.index_cache:
                    del self.index_cache[topic]
                    self._update_index_cache(topic)

    @property
    def start_stamp(self):
        with self._bag_lock:
            start_stamp = None
            for bag in self.bags:               
                bag_start_stamp = bag_helper.get_start_stamp(bag)
                if bag_start_stamp is None:
                    continue
                if start_stamp is None:
                    start_stamp = bag_start_stamp
                else:
                    start_stamp = min(start_stamp, bag_start_stamp)
            return start_stamp

    @property
    def end_stamp(self):
        with self._bag_lock:
            end_stamp = 0.0
            for bag in self.bags:
                bag_end_stamp = bag_helper.get_end_stamp(bag)
                if bag_end_stamp is None:
                    continue
                if end_stamp is None:
                    end_stamp = bag_end_stamp
                else:
                    end_stamp = max(end_stamp, bag_end_stamp)
            return end_stamp

    @property
    def topics(self):
        with self._bag_lock:
            topics = set()
            for bag in self.bags:
                for topic in bag_helper.get_topics(bag):
                    topics.add(topic)
            return sorted(topics)

    @property
    def topics_by_datatype(self):
        with self._bag_lock:
            topics_by_datatype = {}
            for bag in self.bags:
                for datatype, topics in bag_helper.get_topics_by_datatype(bag).items():
                    topics_by_datatype.setdefault(datatype, []).extend(topics)
            return topics_by_datatype

    def get_datatype(self, topic):
        with self._bag_lock:
            datatype = None
            for bag in self.bags:
                bag_datatype = bag_helper.get_datatype(bag, topic)
                if datatype and bag_datatype and (bag_datatype != datatype):
                    raise Exception('topic %s has multiple datatypes: %s and %s' % (topic, datatype, bag_datatype))
                datatype = bag_datatype
            return datatype

    def get_entries(self, topic, start_stamp, end_stamp):
        with self._bag_lock:
            from rosbag import bag
            
            start_time = start_stamp.to_sec()
            end_time   = end_stamp.to_sec()

            bag_entries = []
            for b in self.bags:
                bag_start_time = bag_helper.get_start_stamp(b)
                if bag_start_time > end_time:
                    continue
                
                bag_end_time = bag_helper.get_end_stamp(b)
                if bag_end_time < start_time:
                    continue
                
                connections = list(b._get_connections(topic))
                bag_entries.append(b._get_entries(connections, start_stamp, end_stamp))

            for entry, _ in bag._mergesort(bag_entries, key=lambda entry: entry.time):
                yield entry

    def get_entry(self, t, topic):
        with self._bag_lock:
            entry_bag, entry = None, None
            for bag in self.bags:
                bag_entry = bag._get_entry(t, bag._get_connections(topic))
                if bag_entry and (not entry or bag_entry.time > entry.time):
                    entry_bag, entry = bag, bag_entry

            return entry_bag, entry

    def get_entry_after(self, t):
        with self._bag_lock:
            entry_bag, entry = None, None
            for bag in self.bags:
                bag_entry = bag._get_entry_after(t, bag._get_connections())
                if bag_entry and (not entry or bag_entry.time < entry.time):
                    entry_bag, entry = bag, bag_entry

            return entry_bag, entry

    def read_message(self, bag, position):
        with self._bag_lock:
            return bag._read_message(position)

    ##

    def on_close(self, event):
        self.index_cache_thread.stop()
        self.play_thread.stop()

        if self.recorder:
            self.recorder.stop()
            self.recorder.join()

    ### Views / listeners

    def add_view(self, topic, view):
        self.views.append(view)
        self.add_listener(topic, view)

    def remove_view(self, topic, view):
        self.remove_listener(topic, view)
        self.views.remove(view)

    def add_listener(self, topic, listener):
        self.listeners.setdefault(topic, []).append(listener)

        self._update_message_view()
        self.invalidate()

    def remove_listener(self, topic, listener):
        topic_listeners = self.listeners.get(topic)
        if topic_listeners is not None and listener in topic_listeners:
            topic_listeners.remove(listener)
            if len(topic_listeners) == 0:
                del self.listeners[topic]
        self.invalidate()

    ### Plugins

    def get_viewer_types(self, datatype):
        return [RawView] + self.viewer_types.get('*', []) + self.viewer_types.get(datatype, [])
    
    def load_plugins(self):
        for view, timeline_renderer, msg_types in plugins.load_plugins():
            for msg_type in msg_types:
                self.viewer_types.setdefault(msg_type, []).append(view)
            if timeline_renderer is not None:
                self.timeline_renderers[msg_type] = timeline_renderer(self)

    ### Timeline renderers

    def get_renderers(self):
        renderers = []

        for topic in self.topics:
            datatype = self.get_datatype(topic)
            renderer = self.timeline_renderers.get(datatype)
            if renderer is not None:
                renderers.append((topic, renderer))
                
        return renderers

    def is_renderer_active(self, topic):
        return topic in self.rendered_topics

    def toggle_renderers(self):
        idle_renderers = len(self.rendered_topics) < len(self.topics)
        
        self.set_renderers_active(idle_renderers)

    def set_renderers_active(self, active):
        if active:
            for topic in self.topics:
                self.rendered_topics.add(topic)
        else:
            self.rendered_topics.clear()

        self.invalidate()
        self.parent.playhead.invalidate()

    def set_renderer_active(self, topic, active):
        if active:
            if topic in self.rendered_topics:
                return
            self.rendered_topics.add(topic)
        else:
            if not topic in self.rendered_topics:
                return
            self.rendered_topics.remove(topic)
        
        self.invalidate()
        self.parent.playhead.invalidate()

    ###

    @property
    def history_bottom(self):
        return self.history_top + self.history_height
    
    @property
    def history_right(self):
        return self.history_left + self.history_width

    def _set_status(self, status):
        self.parent.GetParent().SetStatusText(str(status))

    ### Navigation

    def navigate_play(self): self.play_speed = 1.0
    def navigate_stop(self): self.play_speed = 0.0

    def navigate_rewind(self):
        if self.play_speed <= -1.0:
            self.play_speed *= 2.0
        elif self.play_speed < 0.001:
            self.play_speed = -1.0
        else:
            self.play_speed *= 0.5
            
        self.play_speed = max(self.min_play_speed, self.play_speed)
        
    def navigate_fastforward(self):
        if self.play_speed >= 1.0:
            self.play_speed *= 2.0
        elif self.play_speed > -0.001:
            self.play_speed = 1.0
        else:
            self.play_speed *= 0.5

        self.play_speed = min(self.max_play_speed, self.play_speed)

    def navigate_start(self): self.set_playhead(self.start_stamp)
    def navigate_end(self):   self.set_playhead(self.end_stamp)

    ### View port

    def reset_timeline(self):
        self.reset_zoom() 
        self.set_playhead(self.stamp_left)
        
    def set_timeline_view(self, stamp_left, stamp_right):
        self.stamp_left  = stamp_left
        self.stamp_right = stamp_right

        self.invalidate()

    def translate_timeline(self, dx):
        dstamp = self.map_dx_to_dstamp(dx)

        self.stamp_left  -= dstamp
        self.stamp_right -= dstamp

        self.invalidate()

    def reset_zoom(self):
        start_stamp, end_stamp = self.start_stamp, self.end_stamp
        if end_stamp - start_stamp < 5.0:
            end_stamp = start_stamp + 5.0
        
        self.set_timeline_view(start_stamp, end_stamp)

    def zoom_in(self):  self.zoom_timeline(0.5)
    def zoom_out(self): self.zoom_timeline(2.0)

    def zoom_timeline(self, zoom):
        new_stamp_interval = zoom * (self.stamp_right - self.stamp_left)
        
        # Enforce zoom limits
        px_per_sec = self.history_width / new_stamp_interval
        if px_per_sec < self.min_zoom:
            new_stamp_interval = self.history_width / self.min_zoom
        elif px_per_sec > self.max_zoom:
            new_stamp_interval = self.history_width / self.max_zoom
        
        playhead_fraction = (self.playhead - self.stamp_left) / (self.stamp_right - self.stamp_left)
        self.stamp_left   = self.playhead - playhead_fraction * new_stamp_interval
        self.stamp_right  = self.stamp_left + new_stamp_interval

        self._layout()

        self.invalidate()
    
    def set_playhead(self, playhead):
        with self.playhead_lock:
            self.playhead = playhead
            
            if self.playhead > self.stamp_right:
                dstamp = self.playhead - self.stamp_right + (self.stamp_right - self.stamp_left) * 0.75
                dstamp = min(dstamp, self.end_stamp - self.stamp_right)
                
                self.stamp_left  += dstamp
                self.stamp_right += dstamp
                
                self.invalidate()
            elif self.playhead < self.stamp_left:
                dstamp = self.stamp_left - self.playhead + (self.stamp_right - self.stamp_left) * 0.75
                dstamp = min(dstamp, self.stamp_left - self.start_stamp)

                self.stamp_left  -= dstamp
                self.stamp_right -= dstamp
                self.invalidate()

            playhead_time = roslib.rostime.Time.from_sec(self.playhead)

            self.playhead_positions = {}
            for topic in self.topics:
                bag, entry = self.get_entry(playhead_time, topic)
                if entry:
                    self.playhead_positions[topic] = (bag, entry.position)

            self._update_message_view()

            self.parent.status.invalidate()
            self.parent.playhead.update_position()

    ### Rendering

    def on_size(self, event):
        (w, h) = self.parent.GetClientSize()
        
        self.resize(w - self.x, h - self.y)   # resize layer to fill client area

    def paint(self, dc):
        if len(self.bags) == 0 or len(self.topics) == 0:
            return

        if self.stamp_left is None:
            self.reset_timeline()

        dc.set_antialias(cairo.ANTIALIAS_NONE)

        dc.set_font_size(12.0)

        self._calc_font_sizes(dc)
        self._layout()

        self._draw_topic_dividers(dc)
        self._draw_time_indicators(dc)
        self._draw_topic_histories(dc)
        self._draw_bag_ends(dc)
        self._draw_topic_names(dc)
        self._draw_history_border(dc)

    def _calc_font_sizes(self, dc):
        (ascent, descent, height, max_x_advance, max_y_advance) = dc.font_extents()
        
        self.time_font_height = height

        self.topic_name_sizes = {}
        for topic in self.topics:
            x_bearing, y_bearing, width, height, x_advance, y_advance = dc.text_extents(topic)
            self.topic_name_sizes[topic] = (width, height)

    def _layout(self):
        max_topic_name_width   = max([w for (w, h) in self.topic_name_sizes.values()])
        self.topic_font_height = max([h for (w, h) in self.topic_name_sizes.values()])

        new_history_left  = self.margin_left + max_topic_name_width + 3
        new_history_width = self.width - self.history_left - self.margin_right
        updated_history = new_history_left != self.history_left or new_history_width != self.history_width
        if updated_history:
            self.history_left  = new_history_left
            self.history_width = new_history_width
            self.parent.playhead.update_position()

        self.history_bounds = {}
        y = self.history_top
        for topic in self.topics:
            datatype = self.get_datatype(topic)
            
            topic_height = None
            if topic in self.rendered_topics:
                renderer = self.timeline_renderers.get(datatype)
                if renderer:
                    topic_height = renderer.get_segment_height(topic)
            if not topic_height:
                topic_height = self.topic_font_height + 2

            self.history_bounds[topic] = (self.history_left, y, self.history_width, topic_height)

            y += topic_height

        self.history_bottom = max([y + h for (x, y, w, h) in self.history_bounds.values()]) - 1

    def _draw_topic_dividers(self, dc):
        clip_left  = self.history_left
        clip_right = self.history_left + self.history_width

        dc.set_line_width(1)
        row = 0
        for topic in self.topics:
            (x, y, w, h) = self.history_bounds[topic]
            
            left = max(clip_left, x)
            rect = (left, y, min(clip_right - left, w), h)
            if row % 2 == 0:
                dc.set_source_rgba(0.7, 0.7, 0.7, 0.1)
                dc.rectangle(*rect)
                dc.fill()
            dc.set_source_rgba(0.8, 0.8, 0.8, 0.4)
            dc.rectangle(*rect)
            dc.stroke()
            row += 1

    def _draw_time_indicators(self, dc):
        """
        Draw vertical grid-lines showing major and minor time divisions.
        """

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

        start_stamp = self.start_stamp

        major_stamps = list(self._get_stamps(start_stamp, major_division))
        self._draw_major_divisions(dc, major_stamps, start_stamp, major_division)
        if minor_division:
            minor_stamps = [s for s in self._get_stamps(start_stamp, minor_division) if s not in major_stamps]
            self._draw_minor_divisions(dc, minor_stamps, start_stamp, minor_division)

    def _draw_major_divisions(self, dc, stamps, start_stamp, division):
        dc.set_line_width(1.0)

        for stamp in stamps:
            x = self.map_stamp_to_x(stamp, False)

            label        = self._get_label(division, stamp - start_stamp)
            label_x      = x + 3
            label_y      = self.history_top - self.parent.playhead.pointer_size[1] - 2
            label_extent = dc.text_extents(label)
            if label_x + label_extent[2] < self.width:
                dc.set_source_rgb(0, 0, 0)
                dc.move_to(label_x, label_y)
                dc.show_text(label)

            dc.set_source_rgba(0.25, 0.25, 0.25, 0.3)
            dc.move_to(x, label_y + 1)
            dc.line_to(x, self.history_bottom)
        
        dc.stroke()

    def _draw_minor_divisions(self, dc, stamps, start_stamp, division):
        xs = [self.map_stamp_to_x(stamp) for stamp in stamps]

        dc.set_source_rgba(*self.minor_divisions_color_tick)
        for x in xs:
            dc.move_to(x, self.history_top - self.time_tick_height)
            dc.line_to(x, self.history_top)
        dc.stroke()

        dc.set_source_rgba(*self.minor_divisions_color)
        for x in xs:
            dc.move_to(x, self.history_top)
            dc.line_to(x, self.history_bottom)
        dc.stroke()

    def _get_stamps(self, start_stamp, stamp_step):
        """Generate visible stamps every stamp_step"""
        if start_stamp >= self.stamp_left:
            stamp = start_stamp
        else:
            stamp = start_stamp + int((self.stamp_left - start_stamp) / stamp_step) * stamp_step + stamp_step

        while stamp < self.stamp_right:
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

    def _draw_topic_histories(self, dc):
        for topic in sorted(self.history_bounds.keys()):
            self._draw_topic_history(dc, topic)

    def _update_index_cache(self, topic):
        if topic not in self.index_cache:
            # Don't have any cache of messages in this topic
            start_time = roslib.rostime.Time.from_sec(max(0.0, self.start_stamp))
            topic_cache = []
            self.index_cache[topic] = topic_cache
        else:
            topic_cache = self.index_cache[topic]               

            # Check if the cache has been invalidated
            if topic not in self.invalidated_caches:
                return topic_cache
            
            start_time = roslib.rostime.Time.from_sec(max(0.0, topic_cache[-1]))

        end_time = roslib.rostime.Time.from_sec(max(0.0, self.end_stamp))

        for entry in self.get_entries(topic, start_time, end_time):
            topic_cache.append(entry.time.to_sec())

        if topic in self.invalidated_caches:
            self.invalidated_caches.remove(topic)
            
        return topic_cache

    def _draw_topic_history(self, dc, topic):
        """
        Draw boxes to show message regions on timelines.
        """
        x, y, w, h = self.history_bounds[topic]
        
        msg_y      = y + 1
        msg_height = h - 1

        datatype = self.get_datatype(topic)

        # Get the renderer and the message combine interval
        renderer = None
        msg_combine_interval = None
        if topic in self.rendered_topics:
            renderer = self.timeline_renderers.get(datatype)
            if not renderer is None:
                msg_combine_interval = self.map_dx_to_dstamp(renderer.msg_combine_px)
        if msg_combine_interval is None:
            msg_combine_interval = self.map_dx_to_dstamp(self.default_msg_combine_px)

        # Get the cache
        with self.index_cache_lock:
            if topic not in self.index_cache:
                return
            all_stamps = self.index_cache[topic]

        start_index = bisect.bisect_left(all_stamps, self.stamp_left)
        end_index   = bisect.bisect_left(all_stamps, self.stamp_right)

        # Set pen based on datatype
        datatype_color = self.datatype_colors.get(datatype, self.default_datatype_color)

        # Iterate through regions of connected messages
        width_interval = self.history_width / (self.stamp_right - self.stamp_left)

        dc.set_line_width(1)
        dc.set_source_rgb(*datatype_color) 
        for (stamp_start, stamp_end) in self._find_regions(all_stamps[start_index:end_index], self.map_dx_to_dstamp(self.default_msg_combine_px)):
            region_x_start = self.history_left + (stamp_start - self.stamp_left) * width_interval
            region_x_end   = self.history_left + (stamp_end   - self.stamp_left) * width_interval
            region_width   = max(1, region_x_end - region_x_start)

            dc.rectangle(region_x_start, msg_y, region_width, msg_height)

        dc.fill()

        # Draw active message
        if topic in self.listeners or True:
            dc.set_line_width(3)
            playhead_stamp = None
            playhead_index = bisect.bisect_right(all_stamps, self.playhead) - 1
            if playhead_index >= 0:
                playhead_stamp = all_stamps[playhead_index]
                if playhead_stamp > self.stamp_left and playhead_stamp < self.stamp_right:
                    playhead_x = self.history_left + (all_stamps[playhead_index] - self.stamp_left) * width_interval
                    dc.move_to(playhead_x, msg_y)
                    dc.line_to(playhead_x, msg_y + msg_height)
                    dc.stroke()

        # Custom renderer
        if renderer:
            # Iterate through regions of connected messages
            for (stamp_start, stamp_end) in self._find_regions(all_stamps[start_index:end_index], msg_combine_interval):
                region_x_start = self.history_left + (stamp_start - self.stamp_left) * width_interval
                region_x_end   = self.history_left + (stamp_end   - self.stamp_left) * width_interval
                region_width   = max(1, region_x_end - region_x_start)

                renderer.draw_timeline_segment(dc, topic, stamp_start, stamp_end, region_x_start, msg_y, region_width, msg_height)

    def _find_regions(self, stamps, max_interval):
        """Group timestamps into regions connected by timestamps less than max_interval secs apart"""
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

    ## Draws topic names
    def _draw_topic_names(self, dc):
        topics = self.history_bounds.keys()
        coords = [(self.margin_left, y + (h / 2) + (self.topic_font_height / 2)) for (x, y, w, h) in self.history_bounds.values()]

        dc.set_font_size(12)
        dc.set_source_rgb(0, 0, 0)
        for text, coords in zip([t.lstrip('/') for t in topics], coords):
            dc.move_to(*coords)
            dc.show_text(text)

    ## Draw markers to indicate the extent of the bag file
    def _draw_bag_ends(self, dc):
        x_start, x_end = self.map_stamp_to_x(self.start_stamp), self.map_stamp_to_x(self.end_stamp)
        dc.set_source_rgba(0, 0, 0, 0.1)
        dc.rectangle(self.history_left, self.history_top, x_start - self.history_left,                    self.history_bottom - self.history_top)
        dc.rectangle(x_end,             self.history_top, self.history_left + self.history_width - x_end, self.history_bottom - self.history_top)
        dc.fill()

    def _draw_history_border(self, dc):
        bounds_width = min(self.history_width, self.parent.width - self.x)

        x, y, w, h = self.history_left, self.history_top, bounds_width, self.history_bottom - self.history_top

        dc.set_source_rgb(0.1, 0.1, 0.1)
        dc.set_line_width(1)
        dc.rectangle(x, y, w, h)
        dc.stroke()

        dc.set_source_rgba(0.6, 0.6, 0.6, 0.3)
        dc.move_to(x + 2,     y + h + 1)
        dc.line_to(x + w + 1, y + h + 1)
        dc.line_to(x + w + 1, y + 2)
        dc.stroke()

    ### Pixel location <-> timestamp

    def map_x_to_stamp(self, x, clamp_to_visible=True):
        fraction = float(x - self.history_left) / self.history_width

        if clamp_to_visible:
            if fraction <= 0.0:
                return self.stamp_left
            elif fraction >= 1.0:
                return self.stamp_right

        return max(0.0, self.stamp_left + fraction * (self.stamp_right - self.stamp_left))

    def map_dx_to_dstamp(self, dx):
        return float(dx) * (self.stamp_right - self.stamp_left) / self.history_width

    def map_stamp_to_x(self, stamp, clamp_to_visible=True):
        fraction = (stamp - self.stamp_left) / (self.stamp_right - self.stamp_left)

        if clamp_to_visible:
            fraction = min(1.0, max(0.0, fraction))

        return self.history_left + fraction * self.history_width

    def map_dstamp_to_dx(self, dstamp):
        return (float(dstamp) * self.history_width) / (self.stamp_right - self.stamp_left)

    ### Mouse events

    def on_left_down(self, event):
        self.clicked_pos = self.dragged_pos = event.GetPosition()
        if not self.contains(*self.clicked_pos):
            return

        if event.ShiftDown():
            return

        x, y = self.clicked_pos[0] - self.x, self.clicked_pos[1] - self.y
        if x < self.history_left or x > self.history_left + self.history_width:
            return

        self.set_playhead(self.map_x_to_stamp(x))  

    def on_middle_down(self, event):
        self.clicked_pos = self.dragged_pos = event.GetPosition()

    def on_right_down(self, event):
        self.clicked_pos = self.dragged_pos = event.GetPosition()
        if not self.contains(*self.clicked_pos):
            return

        self.parent.display_popup(self.clicked_pos)

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

            dx_click, dy_click = x - self.clicked_pos[0], y - self.clicked_pos[1]
            dx_drag,  dy_drag  = x - self.dragged_pos[0], y - self.dragged_pos[1]
            
            if not self.history_left:  # @todo: need a better notion of initialized
                return

            if dx_drag != 0:
                self.translate_timeline(dx_drag)
            if (dx_drag == 0 and abs(dy_drag) > 0) or (dx_drag != 0 and abs(float(dy_drag) / dx_drag) > 0.2 and abs(dy_drag) > 1):
                zoom = min(self.max_zoom_speed, max(self.min_zoom_speed, 1.0 + self.zoom_sensitivity * dy_drag))
                self.zoom_timeline(zoom)

            self.parent.playhead.update_position()

        elif left:
            x, y = mouse_pos[0] - self.x, mouse_pos[1] - self.y
            
            if not self.history_left:
                return

            self.set_playhead(self.map_x_to_stamp(x))

        self.dragged_pos = mouse_pos

    ###

    def _update_message_view(self):
        if not self.playhead_positions:
            return

        msgs = {}
        for topic in self.playhead_positions:
            if topic in self.listeners:
                bag, playhead_position = self.playhead_positions[topic]
                if playhead_position is not None:
                    # Load the message
                    msgs[topic] = (bag, self.read_message(bag, playhead_position))
                    continue

        # Inform the listeners
        for topic in self.topics:
            topic_listeners = self.listeners.get(topic)
            if not topic_listeners:
                continue

            if topic in msgs:
                bag, msg_data = msgs[topic]
                for listener in topic_listeners:
                    try:
                        listener.message_viewed(bag, msg_data)
                    except wx.PyDeadObjectError:
                        self.remove_listener(topic, listener)
            else:
                for listener in topic_listeners:
                    try:
                        listener.message_cleared()
                    except wx.PyDeadObjectError:
                        self.remove_listener(topic, listener)

    ### Layout management

    def save_layout(self):
        user_config_dir = wx.StandardPaths.Get().GetUserConfigDir()
        config_dir = os.path.join(user_config_dir, '.rxbag')
        if not os.path.exists(config_dir):
            os.mkdir(config_dir)
        layout_file = os.path.join(config_dir, 'layout')
        config = wx.Config(localFilename=layout_file)
        
        # TODO
        #for i, view in enumerate(self.views):
        #    config.Write('/Views/View%d' % (i + 1), view.__class__.__name__)
        #config.Flush()

    def load_layout(self):
        user_config_dir = wx.StandardPaths.Get().GetUserConfigDir()
        config_dir = os.path.join(user_config_dir, '.rxbag')
        if not os.path.exists(config_dir):
            return
        layout_file = os.path.join(config_dir, 'layout')
        config = wx.Config(localFilename=layout_file)
        
        # TODO
