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

"""
The rxbag timeline control.
"""

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
import wx.lib.newevent

import bag_helper
import plugins
from player              import Player
from raw_view            import RawView
from recorder            import Recorder
from timeline_popup_menu import TimelinePopupMenu
from timeline_status_bar import TimelineStatusBar, PlayheadChangedEvent, EVT_PLAYHEAD_CHANGED
from timeline_toolbar    import TimelineToolBar

class _SelectionMode(object):
    NONE        = 0    # no region marked or started
    LEFT_MARKED = 1    # one end of the region has been marked
    MARKED      = 2    # region has been marked
    SHIFTING    = 3    # region is marked; now shifting
    MOVE_LEFT   = 4    # region is marked; changing the left mark
    MOVE_RIGHT  = 5    # region is marked; changing the right mark

class Timeline(wx.Window):
    def __init__(self, *args, **kwargs):
        wx.Window.__init__(self, *args, **kwargs)

        ## Rendering parameters

        self._sec_divisions = [0.001, 0.005, 0.01, 0.05, 0.1, 0.5,                                 # 1ms, 5ms, 10ms, 50ms, 100ms, 500ms
                               1, 5, 15, 30,                                                       # 1s, 5s, 15s, 30s
                               1 * 60, 2 * 60, 5 * 60, 10 * 60, 15 * 60, 30 * 60,                  # 1m, 2m, 5m, 10m, 15m, 30m
                               1 * 60 * 60, 2 * 60 * 60, 3 * 60 * 60, 6 * 60 * 60, 12 * 60 * 60,   # 1h, 2h, 3h, 6h, 12h
                               1 * 60 * 60 * 24, 7 * 60 * 60 * 24]                                 # 1d, 7d
        self._minor_spacing = 15
        self._major_spacing = 50

        self._time_font_size   = 12.0
        self._topic_font_size  = 12.0
        self._topic_font_color = (0, 0, 0)

        self._playhead_pointer_size = (6, 6)                # size of playhead pointer
        self._playhead_line_width   = 1                     # width of line of playhead
        self._playhead_color        = (1, 0, 0, 0.75)       # color of playhead

        self._time_font_height          = None
        self._topic_font_height         = None
        self._topic_name_sizes          = None
        self._topic_name_spacing        = 3                 # minimum pixels between end of topic name and start of history
        self._margin_left               = 4
        self._margin_right              = 8
        self._margin_bottom             = 6
        self._history_top               = 30
        self._topic_vertical_padding    = 4

        self._bag_end_color             = (0, 0, 0, 0.1)    # color of background of timeline before first message and after last
        
        self._time_tick_height          = 5

        self._minor_divisions_color_tick   = (0.5, 0.5, 0.5, 0.5)     # color of vertical line above timeline for minor division
        self._minor_divisions_color        = (0.6, 0.6, 0.6, 0.5)     # color of vertical line for minor division
        self._minor_divisions_dash         = [2, 2]                   # dash pattern for minor division
        self._major_divisions_label_indent = 3                        # padding in px between line and label
        self._major_divisions_label_color  = (0, 0, 0)                # color of label for major division
        self._major_divisions_color        = (0.25, 0.25, 0.25, 0.7)  # color of vertical line for major division
        self._major_divisions_dash         = [2, 2]                   # dash pattern for major division

        self._history_background_color_alternate = (0.7, 0.7, 0.7, 0.1)
        self._history_background_color           = (0.8, 0.8, 0.8, 0.4)

        self._selected_region_color              = (0.0, 0.7, 0.0, 0.08)
        self._selected_region_outline_top_color  = (0.0, 0.3, 0.0, 0.2)
        self._selected_region_outline_ends_color = (0.0, 0.3, 0.0, 0.4)

        # Time ticks

        self._default_datatype_color = (0.0, 0.0, 0.4)
        self._datatype_colors = {
            'sensor_msgs/CameraInfo':            (  0,   0, 0.6),
            'sensor_msgs/Image':                 (  0, 0.3, 0.3),
            'sensor_msgs/LaserScan':             (0.6,   0,   0),
            'pr2_msgs/LaserScannerSignal':       (0.6,   0,   0),
            'pr2_mechanism_msgs/MechanismState': (  0, 0.6,   0),
            'tf/tfMessage':                      (  0, 0.6,   0),
        }

        self._default_msg_combine_px    = 1.0
        self._active_message_line_width = 3
        
        ##

        # Zoom

        self._zoom_sensitivity = 0.005
        self._min_zoom_speed   = 0.5
        self._max_zoom_speed   = 2.0
        self._min_zoom         = 0.0001      # max zoom out (in px/s)
        self._max_zoom         = 50000.0     # max zoom in  (in px/s)

        # Playhead

        self._max_play_speed = 1024.0          # fastest X play speed
        self._min_play_speed = 1.0 / 1024.0    # slowest X play speed

        self._viewer_types       = {}
        self._timeline_renderers = {}
        self._rendered_topics    = set()

        self.load_plugins()

        ##

        self._clicked_pos = None
        self._dragged_pos = None

        self._history_left   = 0
        self._history_width  = 0
        self._history_bottom = 0
        self._history_bounds = {}
        
        self._selecting_mode = _SelectionMode.NONE
        self._selected_left  = None
        self._selected_right = None
        
        self._selection_handle_width = 3.0

        ##

        self._bag_lock = threading.RLock()
        self._bags     = []

        self._start_stamp        = None
        self._end_stamp          = None
        self._topics             = []
        self._topics_by_datatype = {}

        self._stamp_left  = None      # earliest visible timestamp on the timeline
        self._stamp_right = None      # latest visible timestamp on the timeline
        
        self._playhead_lock = threading.RLock()
        self._playhead      = None                   # timestamp of the playhead

        self._paused        = False
        self._play_speed    = 0.0
        self._play_all      = False
        
        self._playhead_positions_cvs   = {}
        self._playhead_positions       = {}                  # topic -> (bag, position)
        self._message_loaders          = {}
        self._messages_cvs             = {}
        self._messages                 = {}                  # topic -> (bag, msg_data)
        self._message_listener_threads = {}                  # listener -> MessageListenerThread

        self._views = []

        self._listeners = {}

        self.index_cache_cv     = threading.Condition()
        self.index_cache        = {}
        self.invalidated_caches = set()

        self._recorder = None
        self._player   = False

        ## Playing
        
        self.last_frame       = None
        self.last_playhead    = None
        self.desired_playhead = None
        self.wrap             = True       # should the playhead wrap when it reaches the end?
        self.stick_to_end     = False      # should the playhead stick to the end?

        ##

        self._index_cache_thread = IndexCacheThread(self)

        # Trap SIGINT and close wx app
        def sigint_handler(signum, frame):
            self._close()
            wx.GetApp().Exit()
        import signal
        signal.signal(signal.SIGINT, sigint_handler)

        self.frame.Bind(wx.EVT_CLOSE, self.on_close)

        self.Parent.Bind(wx.EVT_SIZE, self.on_size)

        self.Bind(wx.EVT_IDLE,        self.on_idle)
        self.Bind(wx.EVT_PAINT,       self.on_paint)
        self.Bind(wx.EVT_KEY_DOWN,    self.on_key_down)
        self.Bind(wx.EVT_LEFT_DOWN,   self.on_left_down)
        self.Bind(wx.EVT_MIDDLE_DOWN, self.on_middle_down)
        self.Bind(wx.EVT_RIGHT_DOWN,  self.on_right_down)
        self.Bind(wx.EVT_LEFT_UP,     self.on_left_up)
        self.Bind(wx.EVT_MIDDLE_UP,   self.on_middle_up)
        self.Bind(wx.EVT_RIGHT_UP,    self.on_right_up)
        self.Bind(wx.EVT_MOTION,      self.on_mouse_move)
        self.Bind(wx.EVT_MOUSEWHEEL,  self.on_mousewheel)
        ##
        
        self._update_title()

        self.frame.ToolBar   = TimelineToolBar  (self.frame, self)
        self.frame.StatusBar = TimelineStatusBar(self.frame, self)

    ###

    # property: play_all

    def _get_play_all(self): return self._play_all

    def _set_play_all(self, play_all):
        if play_all == self._play_all:
            return
    
        self._play_all = not self._play_all

        self.last_frame       = None
        self.last_playhead    = None
        self.desired_playhead = None

    play_all = property(_get_play_all, _set_play_all)

    def toggle_play_all(self):
        self.play_all = not self.play_all

    @property
    def has_selected_region(self): return self._selected_left is not None and self._selected_right is not None

    @property
    def play_region(self):
        if self.has_selected_region:
            return (rospy.Time.from_sec(self._selected_left), rospy.Time.from_sec(self._selected_right))
        else:
            return (self.start_stamp, self.end_stamp)

    @property
    def frame(self): return wx.GetApp().GetTopWindow()

    ## Visual

    @property
    def history_left(self): return self._history_left

    @property
    def history_top(self): return self._history_top

    @property
    def history_bottom(self): return self._history_bottom

    @property
    def history_height(self): return self._history_bottom - self._history_top

    @property
    def history_right(self): return self._history_left + self._history_width

    ##
    
    @property
    def selected_left(self): return self._selected_left

    @property
    def selected_right(self): return self._selected_right

    ##

    def _close(self):
        for renderer in self._timeline_renderers.values(): 
            renderer.close()

        self._index_cache_thread.stop()

        if self._player:
            self._player.stop()

        if self._recorder:
            self._recorder.stop()

    ### Bags

    @property
    def bags(self): return self._bags

    def add_bag(self, bag):
        self._bags.append(bag)
        
        bag_topics = bag_helper.get_topics(bag)
        
        new_topics = set(bag_topics) - set(self._topics)
        for topic in new_topics:
            self._playhead_positions_cvs[topic] = threading.Condition()
            self._messages_cvs[topic]           = threading.Condition()
            self._message_loaders[topic]        = MessageLoader(self, topic)

        self._start_stamp        = self._get_start_stamp()
        self._end_stamp          = self._get_end_stamp()
        self._topics             = self._get_topics()
        self._topics_by_datatype = self._get_topics_by_datatype()

        # If this is the first bag, reset the timeline
        if self._stamp_left is None:
            self.reset_timeline()

        # Invalidate entire index cache for all topics in this bag
        with self.index_cache_cv:
            for topic in bag_topics:
                self.invalidated_caches.add(topic)
                if topic in self.index_cache:
                    del self.index_cache[topic]

            self.index_cache_cv.notify()

        wx.CallAfter(self.frame.ToolBar._setup)
        wx.CallAfter(self._update_title)
        wx.CallAfter(self.Refresh)

    ### Recording

    def record_bag(self, filename, all=True, topics=[], regex=False, limit=0):
        try:
            self._recorder = Recorder(filename, bag_lock=self._bag_lock, all=all, topics=topics, regex=regex, limit=limit)
        except Exception, ex:
            print >> sys.stderr, 'Error opening bag for recording [%s]: %s' % (filename, str(ex))
            return

        self._recorder.add_listener(self._message_recorded)

        self.add_bag(self._recorder.bag)

        self._recorder.start()

        self.wrap = False
        self._index_cache_thread.period = 0.1

        wx.CallAfter(self.frame.ToolBar._setup)  # record button has been added
        self._update_title()

    def toggle_recording(self):
        if self._recorder:
            self._recorder.toggle_paused()
            wx.CallAfter(self.frame.ToolBar._setup)
            self._update_title()

    ### Export

    def copy_region_to_bag(self):
        dialog = wx.FileDialog(self, 'Copy messages to...', wildcard='Bag files (*.bag)|*.bag', style=wx.FD_SAVE)
        if dialog.ShowModal() == wx.ID_OK:
            self._export_region(dialog.Path, self.topics, *self.play_region)
        dialog.Destroy()
        
    def _export_region(self, path, topics, start_stamp, end_stamp):
        bag_entries = list(self.get_entries_with_bags(topics, start_stamp, end_stamp))

        # Get the total number of messages to copy
        total_messages = len(bag_entries)
        
        # If no messages, prompt the user and return
        if total_messages == 0:
            wx.MessageDialog(None, 'No messages found', 'rxbag', wx.OK | wx.ICON_EXCLAMATION).ShowModal()
            return
        
        # Open the path for writing
        try:
            export_bag = rosbag.Bag(path, 'w')
        except Exception, ex:
            print >> sys.stderr, 'Error opening bag file [%s] for writing' % path

        # Create a dialog to track progress 
        self._export_progress = wx.ProgressDialog('Exporting messages...', 'Saving %d messages to %s...' % (total_messages, path),
                                                  maximum=total_messages,
                                                  parent=self,
                                                  style=wx.PD_CAN_ABORT | wx.PD_APP_MODAL | wx.PD_ELAPSED_TIME | wx.PD_ESTIMATED_TIME | wx.PD_REMAINING_TIME | wx.PD_SMOOTH | wx.PD_AUTO_HIDE)
        
        # Run copying in a background thread
        self._export_thread = threading.Thread(target=self._run_export_region, args=(export_bag, topics, start_stamp, end_stamp, bag_entries))
        self._export_thread.start()

    def _run_export_region(self, export_bag, topics, start_stamp, end_stamp, bag_entries):
        self._keep_exporting = True

        total_messages = len(bag_entries)
        
        update_step = max(1, total_messages / 100)

        message_num = 1
        
        for bag, entry in bag_entries:
            try:
                topic, msg, t = self.read_message(bag, entry.position)
                export_bag.write(topic, msg, t)
            except Exception, ex:
                print >> sys.stderr, 'Error exporting message at position %d: %s' % (entry.position, str(ex))

            if message_num % update_step == 0 or message_num == total_messages:
                wx.CallAfter(self._update_export_progress, message_num)
                if not self._keep_exporting:
                    wx.CallAfter(self._export_progress.Destroy)
                    break
        
            message_num += 1

        try:
            export_bag.close()
        except Exception, ex:
            print >> sys.stderr, 'Error closing bag file [%s]: %s' % (export_bag.filename, str(ex))

    def _update_export_progress(self, message_num):
        self._keep_exporting, _ = self._export_progress.Update(message_num)

    ### Publishing

    def is_publishing(self, topic):
        return self._player and self._player.is_publishing(topic)

    def start_publishing(self, topic):
        if not self._player and not self._create_player():
            return False
        
        self._player.start_publishing(topic)
        return True

    def stop_publishing(self, topic):
        if not self._player:
            return False
        
        self._player.stop_publishing(topic)
        return True
    
    def _create_player(self):
        if not self._player:
            try:
                self._player = Player(self)
            except Exception, ex:
                print >> sys.stderr, 'Error starting player; aborting publish: %s' % str(ex)
                return False
            
        return True

    ### Recording

    def _message_recorded(self, topic, msg, t):
        if self._start_stamp is None:
            self._start_stamp = t
            self._end_stamp   = t
            self._playhead    = t
        elif self._end_stamp is None or t > self._end_stamp:
            self._end_stamp = t

        if not self._topics or topic not in self._topics:
            self._topics             = self._get_topics()
            self._topics_by_datatype = self._get_topics_by_datatype()

            self._playhead_positions_cvs[topic] = threading.Condition()
            self._messages_cvs[topic]           = threading.Condition()
            self._message_loaders[topic]        = MessageLoader(self, topic)

        if self._stamp_left is None:
            self.reset_zoom()

        # Notify the index caching thread that it has work to do
        with self.index_cache_cv:
            self.invalidated_caches.add(topic)
            self.index_cache_cv.notify()

        if topic in self._listeners:
            for listener in self._listeners[topic]:
                try:
                    listener.timeline_changed()
                except wx.PyDeadObjectError:
                    self.remove_listener(topic, listener)

    ## Timeline info

    @property
    def start_stamp(self): return self._start_stamp

    @property
    def end_stamp(self): return self._end_stamp

    @property
    def topics(self): return self._topics

    @property
    def topics_by_datatype(self): return self._topics_by_datatype
    
    def _get_start_stamp(self):
        with self._bag_lock:
            start_stamp = None
            for bag in self._bags:
                bag_start_stamp = bag_helper.get_start_stamp(bag)
                if bag_start_stamp is not None and (start_stamp is None or bag_start_stamp < start_stamp):
                    start_stamp = bag_start_stamp
            return start_stamp
    
    def _get_end_stamp(self):
        with self._bag_lock:
            end_stamp = None
            for bag in self._bags:
                bag_end_stamp = bag_helper.get_end_stamp(bag)
                if bag_end_stamp is not None and (end_stamp is None or bag_end_stamp > end_stamp):
                    end_stamp = bag_end_stamp
            return end_stamp
    
    def _get_topics(self):
        with self._bag_lock:
            topics = set()
            for bag in self._bags:
                for topic in bag_helper.get_topics(bag):
                    topics.add(topic)
            return sorted(topics)

    def _get_topics_by_datatype(self):
        with self._bag_lock:
            topics_by_datatype = {}
            for bag in self._bags:
                for datatype, topics in bag_helper.get_topics_by_datatype(bag).items():
                    topics_by_datatype.setdefault(datatype, []).extend(topics)
            return topics_by_datatype

    def get_datatype(self, topic):
        with self._bag_lock:
            datatype = None
            for bag in self._bags:
                bag_datatype = bag_helper.get_datatype(bag, topic)
                if datatype and bag_datatype and (bag_datatype != datatype):
                    raise Exception('topic %s has multiple datatypes: %s and %s' % (topic, datatype, bag_datatype))
                datatype = bag_datatype
            return datatype

    def get_entries(self, topics, start_stamp, end_stamp):
        with self._bag_lock:
            from rosbag import bag

            bag_entries = []
            for b in self._bags:
                bag_start_time = bag_helper.get_start_stamp(b)
                if bag_start_time is not None and bag_start_time > end_stamp:
                    continue

                bag_end_time = bag_helper.get_end_stamp(b)
                if bag_end_time is not None and bag_end_time < start_stamp:
                    continue

                connections = list(b._get_connections(topics))
                bag_entries.append(b._get_entries(connections, start_stamp, end_stamp))

            for entry, _ in bag._mergesort(bag_entries, key=lambda entry: entry.time):
                yield entry

    def get_entries_with_bags(self, topic, start_stamp, end_stamp):
        with self._bag_lock:
            from rosbag import bag

            bag_entries = []
            bag_by_iter = {}
            for b in self._bags:
                bag_start_time = bag_helper.get_start_stamp(b)
                if bag_start_time is not None and bag_start_time > end_stamp:
                    continue

                bag_end_time = bag_helper.get_end_stamp(b)
                if bag_end_time is not None and bag_end_time < start_stamp:
                    continue

                connections = list(b._get_connections(topic))                
                it = iter(b._get_entries(connections, start_stamp, end_stamp))
                bag_by_iter[it] = b
                bag_entries.append(it)

            for entry, it in bag._mergesort(bag_entries, key=lambda entry: entry.time):
                yield bag_by_iter[it], entry

    def get_entry(self, t, topic):
        with self._bag_lock:
            entry_bag, entry = None, None
            for bag in self._bags:
                bag_entry = bag._get_entry(t, bag._get_connections(topic))
                if bag_entry and (not entry or bag_entry.time > entry.time):
                    entry_bag, entry = bag, bag_entry

            return entry_bag, entry

    def get_entry_after(self, t):
        with self._bag_lock:
            entry_bag, entry = None, None
            for bag in self._bags:
                bag_entry = bag._get_entry_after(t, bag._get_connections())
                if bag_entry and (not entry or bag_entry.time < entry.time):
                    entry_bag, entry = bag, bag_entry

            return entry_bag, entry

    def get_next_message_time(self):
        if self.playhead is None:
            return None

        _, entry = self.get_entry_after(self.playhead)
        if entry is None:
            return self.start_stamp

        return entry.time

    def read_message(self, bag, position):
        with self._bag_lock:
            return bag._read_message(position)

    ### Views / listeners

    def add_view(self, topic, view):
        self._views.append(view)
        self.add_listener(topic, view)

    def remove_view(self, topic, view):
        self.remove_listener(topic, view)
        self._views.remove(view)

        wx.CallAfter(self.Refresh)
        
    def has_listeners(self, topic): return topic in self._listeners

    def add_listener(self, topic, listener):
        self._listeners.setdefault(topic, []).append(listener)
        
        self._message_listener_threads[(topic, listener)] = MessageListenerThread(self, topic, listener)

        # Notify the message listeners
        self._message_loaders[topic].reset()
        with self._playhead_positions_cvs[topic]:
            self._playhead_positions_cvs[topic].notify_all()

        wx.CallAfter(self.Refresh)

    def remove_listener(self, topic, listener):
        topic_listeners = self._listeners.get(topic)
        if topic_listeners is not None and listener in topic_listeners:
            topic_listeners.remove(listener)

            if len(topic_listeners) == 0:
                del self._listeners[topic]

            # Stop the message listener thread
            if (topic, listener) in self._message_listener_threads:
                self._message_listener_threads[(topic, listener)].stop()
                del self._message_listener_threads[(topic, listener)]

            wx.CallAfter(self.Refresh)

    ### Plugins

    def get_viewer_types(self, datatype):
        return [RawView] + self._viewer_types.get('*', []) + self._viewer_types.get(datatype, [])
    
    def load_plugins(self):
        for view, timeline_renderer, msg_types in plugins.load_plugins():
            for msg_type in msg_types:
                self._viewer_types.setdefault(msg_type, []).append(view)
                if timeline_renderer:
                    self._timeline_renderers[msg_type] = timeline_renderer(self)

    ### Timeline renderers

    def get_renderers(self):
        renderers = []

        for topic in self.topics:
            datatype = self.get_datatype(topic)
            renderer = self._timeline_renderers.get(datatype)
            if renderer is not None:
                renderers.append((topic, renderer))
                
        return renderers

    def is_renderer_active(self, topic):
        return topic in self._rendered_topics

    def toggle_renderers(self):
        idle_renderers = len(self._rendered_topics) < len(self.topics)
        
        self.set_renderers_active(idle_renderers)

    def set_renderers_active(self, active):
        if active:
            for topic in self.topics:
                self._rendered_topics.add(topic)
        else:
            self._rendered_topics.clear()

        wx.CallAfter(self.frame.ToolBar._setup)
        wx.CallAfter(self.Refresh)

    def set_renderer_active(self, topic, active):
        if active:
            if topic in self._rendered_topics:
                return
            self._rendered_topics.add(topic)
        else:
            if not topic in self._rendered_topics:
                return
            self._rendered_topics.remove(topic)

        wx.CallAfter(self.frame.ToolBar._setup)
        wx.CallAfter(self.Refresh)

    ### Playhead

    # property: play_speed

    def _get_play_speed(self):
        if self._paused:
            return 0.0
        return self._play_speed

    def _set_play_speed(self, play_speed):
        if play_speed == self._play_speed:
            return

        if play_speed > 0.0:
            self._play_speed = min( self._max_play_speed, max( self._min_play_speed, play_speed))
        elif play_speed < 0.0:
            self._play_speed = max(-self._max_play_speed, min(-self._min_play_speed, play_speed))
        else:
            self._play_speed = play_speed

        if self._play_speed < 1.0:
            self.stick_to_end = False

        wx.CallAfter(self.frame.ToolBar._setup)

        wx.PostEvent(self.frame, PlayheadChangedEvent())

    play_speed = property(_get_play_speed, _set_play_speed)

    def toggle_play(self):
        if self._play_speed != 0.0:
            self.play_speed = 0.0
        else:
            self.play_speed = 1.0

    def navigate_play(self): self.play_speed = 1.0
    def navigate_stop(self): self.play_speed = 0.0

    def navigate_rewind(self):
        if self._play_speed < 0.0:
            new_play_speed = self._play_speed * 2.0
        elif self._play_speed == 0.0:
            new_play_speed = -1.0
        else:
            new_play_speed = self._play_speed * 0.5

        self.play_speed = new_play_speed
        
    def navigate_fastforward(self):
        if self._play_speed > 0.0:
            new_play_speed = self._play_speed * 2.0
        elif self._play_speed == 0.0:
            new_play_speed = 2.0
        else:
            new_play_speed = self._play_speed * 0.5

        self.play_speed = new_play_speed

    def navigate_start(self): self.playhead = self.play_region[0]
    def navigate_end(self):   self.playhead = self.play_region[1]

    ### View port

    def reset_timeline(self):
        self.reset_zoom()

        self._selected_left  = None
        self._selected_right = None

        if self._stamp_left is not None:
            self.playhead = rospy.Time.from_sec(self._stamp_left)

    def set_timeline_view(self, stamp_left, stamp_right):
        self._stamp_left  = stamp_left
        self._stamp_right = stamp_right

        wx.CallAfter(self.frame.ToolBar._setup)
        wx.CallAfter(self.Refresh)

    def translate_timeline(self, dstamp):
        self.set_timeline_view(self._stamp_left + dstamp, self._stamp_right + dstamp)

    def reset_zoom(self):
        start_stamp, end_stamp = self.start_stamp, self.end_stamp
        if start_stamp is None:
            return
        
        if (end_stamp - start_stamp) < rospy.Duration.from_sec(5.0):
            end_stamp = start_stamp + rospy.Duration.from_sec(5.0)

        self.set_timeline_view(start_stamp.to_sec(), end_stamp.to_sec())

    def zoom_in(self):  self.zoom_timeline(0.5)
    def zoom_out(self): self.zoom_timeline(2.0)

    def can_zoom_in(self):  return self.can_zoom(0.5)
    def can_zoom_out(self): return self.can_zoom(2.0)

    def can_zoom(self, desired_zoom):
        if not self._stamp_left or not self._playhead:
            return False

        new_interval = self.get_zoom_interval(desired_zoom)
        
        new_range   = new_interval[1] - new_interval[0]
        curr_range  = self._stamp_right - self._stamp_left
        actual_zoom = new_range / curr_range

        if desired_zoom < 1.0:
            return actual_zoom < 0.95
        else:
            return actual_zoom > 1.05

    def zoom_timeline(self, zoom):
        interval = self.get_zoom_interval(zoom)
        if not interval:
            return

        self._stamp_left, self._stamp_right = interval

        self._layout()

        wx.CallAfter(self.frame.ToolBar._setup)
        wx.CallAfter(self.Refresh)

    def get_zoom_interval(self, zoom):
        if self._stamp_left is None:
            return None
        
        stamp_interval     = self._stamp_right - self._stamp_left
        playhead_fraction  = (self._playhead.to_sec() - self._stamp_left) / stamp_interval
        
        new_stamp_interval = zoom * stamp_interval
        
        # Enforce zoom limits
        px_per_sec = self._history_width / new_stamp_interval
        if px_per_sec < self._min_zoom:
            new_stamp_interval = self._history_width / self._min_zoom
        elif px_per_sec > self._max_zoom:
            new_stamp_interval = self._history_width / self._max_zoom

        left  = self._playhead.to_sec() - playhead_fraction * new_stamp_interval
        right = left + new_stamp_interval

        return (left, right)

    # property: playhead

    def _get_playhead(self): return self._playhead

    def _set_playhead(self, playhead):
        with self._playhead_lock:
            if playhead == self._playhead:
                return

            self._playhead = playhead

            if self._playhead != self.end_stamp:
                self.stick_to_end = False

            playhead_secs = playhead.to_sec()

            if playhead_secs > self._stamp_right:
                dstamp = playhead_secs - self._stamp_right + (self._stamp_right - self._stamp_left) * 0.75
                if dstamp > self.end_stamp.to_sec() - self._stamp_right:
                    dstamp = self.end_stamp.to_sec() - self._stamp_right
                self.translate_timeline(dstamp)
                
            elif playhead_secs < self._stamp_left:
                dstamp = self._stamp_left - playhead_secs + (self._stamp_right - self._stamp_left) * 0.75
                if dstamp > self._stamp_left - self.start_stamp.to_sec():
                    dstamp = self._stamp_left - self.start_stamp.to_sec()
                self.translate_timeline(-dstamp)

            # Update the playhead positions
            for topic in self.topics:
                bag, entry = self.get_entry(self._playhead, topic)
                if entry:
                    if topic in self._playhead_positions and self._playhead_positions[topic] == (bag, entry.position):
                        continue
                    new_playhead_position = (bag, entry.position)
                else:
                    new_playhead_position = (None, None)

                with self._playhead_positions_cvs[topic]:
                    self._playhead_positions[topic] = new_playhead_position
                    self._playhead_positions_cvs[topic].notify_all()           # notify all message loaders that a new message needs to be loaded

            wx.PostEvent(self.frame, PlayheadChangedEvent())
            
            wx.CallAfter(self.Refresh)

    playhead = property(_get_playhead, _set_playhead)

    ### Rendering

    def _update_title(self):
        title = 'rxbag'
        if self._recorder:
            if self._recorder.paused:
                title += ' - %s [recording paused]' % self._bags[0].filename
            else:
                title += ' - %s [recording]' % self._bags[0].filename
        elif len(self.bags) > 0:
            if len(self.bags) == 1:
                title += ' - ' + self._bags[0].filename
            else:
                title += ' - [%d bags]' % len(self._bags)
        
        self.frame.Title = title

    def on_close(self, event):
        self._close()
        event.Skip()

    def on_idle(self, event):
        self._step_playhead()
        event.RequestMore()

    def on_size(self, event):
        self.Position = (0, 0)
        self.Size = (self.Parent.ClientSize[0], max(self.Parent.ClientSize[1], self._history_bottom + self._playhead_pointer_size[1] + self._margin_bottom))
        self.Refresh()

    def on_paint(self, event):
        pdc = wx.PaintDC(self)
        pdc.Background = wx.WHITE_BRUSH
        pdc.Clear()

        if len(self.bags) == 0 or len(self.topics) == 0:
            return

        dc = wx.lib.wxcairo.ContextFromDC(pdc)
        dc.set_antialias(cairo.ANTIALIAS_NONE)

        self._calc_font_sizes(dc)
        self._layout()

        self._draw_topic_dividers(dc)
        self._draw_selected_region(dc)
        self._draw_time_indicators(dc)
        self._draw_topic_histories(dc)
        self._draw_bag_ends(dc)
        self._draw_topic_names(dc)
        self._draw_history_border(dc)
        self._draw_playhead(dc)

    def _calc_font_sizes(self, dc):
        dc.set_font_size(self._time_font_size)
        self._time_font_height = dc.font_extents()[2]

        dc.set_font_size(self._topic_font_size)
        self._topic_name_sizes = dict([(topic, dc.text_extents(topic)[2:4]) for topic in self.topics])

    def _layout(self):
        self._topic_font_height = max([h for (w, h) in self._topic_name_sizes.values()])

        # Calculate history left and history width
        max_topic_name_width    = max([w for (w, h) in self._topic_name_sizes.values()])
        new_history_left  = self._margin_left + max_topic_name_width + self._topic_name_spacing
        new_history_width = self.Size[0] - new_history_left - self._margin_right
        updated_history = (new_history_left != self._history_left or new_history_width != self._history_width)
        if updated_history:
            self._history_left  = new_history_left
            self._history_width = new_history_width
            
            wx.CallAfter(self.frame.ToolBar._setup)    # zoom enabled may have changed

        # Calculate the bounds for each topic
        self._history_bounds = {}
        y = self._history_top
        for topic in self.topics:
            datatype = self.get_datatype(topic)
            
            topic_height = None
            if topic in self._rendered_topics:
                renderer = self._timeline_renderers.get(datatype)
                if renderer:
                    topic_height = renderer.get_segment_height(topic)
            if not topic_height:
                topic_height = self._topic_font_height + self._topic_vertical_padding

            self._history_bounds[topic] = (self._history_left, y, self._history_width, topic_height)

            y += topic_height

        new_history_bottom = max([y + h for (x, y, w, h) in self._history_bounds.values()]) - 1
        if new_history_bottom != self._history_bottom:
            self._history_bottom = new_history_bottom 

            # Resize the scroll bars
            scroll_window = self.Parent
            visible_height = int(self._history_bottom) + self._playhead_pointer_size[1] + self._margin_bottom
            scroll_window.SetScrollbars(0, 1, 0, visible_height, 0, scroll_window.Position[1])

            # Resize the frame to fit
            bar_height = 0
            if self.frame.ToolBar:
                bar_height += self.frame.ToolBar.Size[1]
            if self.frame.StatusBar:
                bar_height += self.frame.StatusBar.Size[1]

            frame = scroll_window.Parent
            frame.Size = (frame.Size[0], visible_height + bar_height)

    def _draw_topic_dividers(self, dc):
        clip_left  = self._history_left
        clip_right = self._history_left + self._history_width

        dc.set_line_width(1)
        row = 0
        for topic in self.topics:
            (x, y, w, h) = self._history_bounds[topic]
            
            left = max(clip_left, x)
            rect = (left, y, min(clip_right - left, w), h)
            if row % 2 == 0:
                dc.set_source_rgba(*self._history_background_color_alternate)
                dc.rectangle(*rect)
                dc.fill()
            dc.set_source_rgba(*self._history_background_color)
            dc.rectangle(*rect)
            dc.stroke()
            row += 1

    def _draw_selected_region(self, dc):
        if self._selected_left is None:
            return
        
        x_left  = self.map_stamp_to_x(self._selected_left)
        if self._selected_right is not None:
            x_right = self.map_stamp_to_x(self._selected_right)
        else:
            x_right = self.map_stamp_to_x(self._playhead.to_sec())

        left   = x_left
        top    = self._history_top - self._playhead_pointer_size[1] - 5 - self._time_font_size - 4
        width  = x_right - x_left
        height = self._history_top - top

        dc.set_source_rgba(*self._selected_region_color)
        dc.rectangle(left, top, width, height)
        dc.fill()

        dc.set_source_rgba(*self._selected_region_outline_ends_color)
        dc.set_line_width(2.0)
        dc.move_to(left, top)
        dc.line_to(left, top + height)
        dc.move_to(left + width, top)
        dc.line_to(left + width, top + height)
        dc.stroke()
        
        dc.set_source_rgba(*self._selected_region_outline_top_color)
        dc.set_line_width(1.0)
        dc.move_to(left,         top)
        dc.line_to(left + width, top)
        dc.stroke()
        dc.set_line_width(2.0)
        dc.move_to(left, self._history_top)
        dc.line_to(left, self._history_bottom)
        dc.move_to(left + width, self._history_top)
        dc.line_to(left + width, self._history_bottom)
        dc.stroke()

    def _draw_time_indicators(self, dc):
        """
        Draw vertical grid-lines showing major and minor time divisions.
        """
        x_per_sec = self.map_dstamp_to_dx(1.0)

        major_divisions = [s for s in self._sec_divisions if x_per_sec * s >= self._major_spacing]
        if len(major_divisions) == 0:
            major_division = max(self._sec_divisions)
        else:
            major_division = min(major_divisions)

        minor_divisions = [s for s in self._sec_divisions if x_per_sec * s >= self._minor_spacing and major_division % s == 0]
        if len(minor_divisions) > 0:
            minor_division = min(minor_divisions)
        else:
            minor_division = None

        start_stamp = self.start_stamp.to_sec()

        major_stamps = list(self._get_stamps(start_stamp, major_division))
        self._draw_major_divisions(dc, major_stamps, start_stamp, major_division)
        if minor_division:
            minor_stamps = [s for s in self._get_stamps(start_stamp, minor_division) if s not in major_stamps]
            self._draw_minor_divisions(dc, minor_stamps, start_stamp, minor_division)

    def _draw_major_divisions(self, dc, stamps, start_stamp, division):
        dc.set_line_width(1.0)
        dc.set_font_size(self._time_font_size)

        label_y = self._history_top - self._playhead_pointer_size[1] - 5

        for stamp in stamps:
            x = self.map_stamp_to_x(stamp, False)

            label        = self._get_label(division, stamp - start_stamp)
            label_x      = x + self._major_divisions_label_indent
            label_extent = dc.text_extents(label)
            if label_x + label_extent[2] < self.Size[0] - 1:  # don't display label if it doesn't fit in the control
                dc.set_source_rgb(*self._major_divisions_label_color)
                dc.move_to(label_x, label_y)
                dc.show_text(label)

            dc.set_source_rgba(*self._major_divisions_color)
            dc.set_dash(self._major_divisions_dash)
            dc.move_to(x, label_y - label_extent[3])
            dc.line_to(x, self._history_bottom)
            dc.stroke()
            dc.set_dash([])

    def _draw_minor_divisions(self, dc, stamps, start_stamp, division):
        xs = [self.map_stamp_to_x(stamp) for stamp in stamps]

        dc.set_source_rgba(*self._minor_divisions_color_tick)
        for x in xs:
            dc.move_to(x, self._history_top - self._time_tick_height)
            dc.line_to(x, self._history_top)
        dc.stroke()

        dc.set_dash(self._minor_divisions_dash)
        dc.set_source_rgba(*self._minor_divisions_color)
        for x in xs:
            dc.move_to(x, self._history_top)
            dc.line_to(x, self._history_bottom)
        dc.stroke()
        dc.set_dash([])

    def _get_stamps(self, start_stamp, stamp_step):
        """
        Generate visible stamps every stamp_step
        """
        if start_stamp >= self._stamp_left:
            stamp = start_stamp
        else:
            stamp = start_stamp + int((self._stamp_left - start_stamp) / stamp_step) * stamp_step + stamp_step

        while stamp < self._stamp_right:
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
        for topic in sorted(self._history_bounds.keys()):
            self._draw_topic_history(dc, topic)

    def _update_index_cache(self, topic):
        """
        Updates the cache of message timestamps for the given topic.

        @return: number of messages added to the index cache
        """
        if self.start_stamp is None or self.end_stamp is None:
            return 0
        
        if topic not in self.index_cache:
            # Don't have any cache of messages in this topic
            start_time = self.start_stamp
            topic_cache = []
            self.index_cache[topic] = topic_cache
        else:
            topic_cache = self.index_cache[topic]               

            # Check if the cache has been invalidated
            if topic not in self.invalidated_caches:
                return 0

            if len(topic_cache) == 0:
                start_time = self.start_stamp
            else:
                start_time = rospy.Time.from_sec(max(0.0, topic_cache[-1]))

        end_time = self.end_stamp

        topic_cache_len = len(topic_cache)

        for entry in self.get_entries(topic, start_time, end_time):
            topic_cache.append(entry.time.to_sec())

        if topic in self.invalidated_caches:
            self.invalidated_caches.remove(topic)

        return len(topic_cache) - topic_cache_len

    def _draw_topic_history(self, dc, topic):
        """
        Draw boxes to show message regions on timelines.
        """
        
        x, y, w, h = self._history_bounds[topic]
        
        msg_y      = y + 1
        msg_height = h - 1

        datatype = self.get_datatype(topic)

        # Get the renderer and the message combine interval
        renderer = None
        msg_combine_interval = None
        if topic in self._rendered_topics:
            renderer = self._timeline_renderers.get(datatype)
            if not renderer is None:
                msg_combine_interval = self.map_dx_to_dstamp(renderer.msg_combine_px)
        if msg_combine_interval is None:
            msg_combine_interval = self.map_dx_to_dstamp(self._default_msg_combine_px)

        # Get the cache
        if topic not in self.index_cache:
            return
        all_stamps = self.index_cache[topic]

        start_index = bisect.bisect_left(all_stamps, self._stamp_left)
        end_index   = bisect.bisect_left(all_stamps, self._stamp_right)

        # Set pen based on datatype
        datatype_color = self._datatype_colors.get(datatype, self._default_datatype_color)

        # Iterate through regions of connected messages
        width_interval = self._history_width / (self._stamp_right - self._stamp_left)

        # Clip to bounds
        dc.save()
        dc.rectangle(self._history_left, self._history_top, self._history_width, self._history_bottom - self._history_top)
        dc.clip()

        # Draw stamps
        dc.set_line_width(1)
        dc.set_source_rgb(*datatype_color)

        for (stamp_start, stamp_end) in self._find_regions(all_stamps[:end_index], self.map_dx_to_dstamp(self._default_msg_combine_px)):
            if stamp_end < self._stamp_left:
                continue
            
            region_x_start = self._history_left + (stamp_start - self._stamp_left) * width_interval
            region_x_end   = self._history_left + (stamp_end   - self._stamp_left) * width_interval
            region_width   = max(1, region_x_end - region_x_start)

            dc.rectangle(region_x_start, msg_y, region_width, msg_height)

        dc.fill()

        # Draw active message
        if topic in self._listeners:
            dc.set_line_width(self._active_message_line_width)
            playhead_stamp = None
            playhead_index = bisect.bisect_right(all_stamps, self._playhead.to_sec()) - 1
            if playhead_index >= 0:
                playhead_stamp = all_stamps[playhead_index]
                if playhead_stamp > self._stamp_left and playhead_stamp < self._stamp_right:
                    playhead_x = self._history_left + (all_stamps[playhead_index] - self._stamp_left) * width_interval
                    dc.move_to(playhead_x, msg_y)
                    dc.line_to(playhead_x, msg_y + msg_height)
                    dc.stroke()

        # Custom renderer
        if renderer:
            # Iterate through regions of connected messages
            for (stamp_start, stamp_end) in self._find_regions(all_stamps[:end_index], msg_combine_interval):
                if stamp_end < self._stamp_left:
                    continue

                region_x_start = self._history_left + (stamp_start - self._stamp_left) * width_interval
                region_x_end   = self._history_left + (stamp_end   - self._stamp_left) * width_interval
                region_width   = max(1, region_x_end - region_x_start)

                renderer.draw_timeline_segment(dc, topic, stamp_start, stamp_end, region_x_start, msg_y, region_width, msg_height)

        dc.restore()

    def _find_regions(self, stamps, max_interval):
        """
        Group timestamps into regions connected by timestamps less than max_interval secs apart
        """
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

    def _draw_topic_names(self, dc):
        """
        Draw topic names.
        """
        topics = self._history_bounds.keys()
        coords = [(self._margin_left, y + (h / 2) + (self._topic_font_height / 2)) for (x, y, w, h) in self._history_bounds.values()]
        
        dc.set_font_size(self._topic_font_size)
        dc.set_source_rgb(*self._topic_font_color)
        for text, coords in zip([t.lstrip('/') for t in topics], coords):
            dc.move_to(*coords)
            dc.show_text(text)

    def _draw_bag_ends(self, dc):
        """
        Draw markers to indicate the extent of the bag file.
        """
        x_start, x_end = self.map_stamp_to_x(self.start_stamp.to_sec()), self.map_stamp_to_x(self.end_stamp.to_sec())
        dc.set_source_rgba(*self._bag_end_color)
        dc.rectangle(self._history_left, self._history_top, x_start - self._history_left,                    self._history_bottom - self._history_top)
        dc.rectangle(x_end,             self._history_top, self._history_left + self._history_width - x_end, self._history_bottom - self._history_top)
        dc.fill()

    def _draw_history_border(self, dc):
        bounds_width = min(self._history_width, self.Size[0])

        x, y, w, h = self._history_left, self._history_top, bounds_width, self._history_bottom - self._history_top

        dc.set_source_rgb(0.1, 0.1, 0.1)
        dc.set_line_width(1)
        dc.rectangle(x, y, w, h)
        dc.stroke()

        dc.set_source_rgba(0.6, 0.6, 0.6, 0.3)
        dc.move_to(x + 2,     y + h + 1)
        dc.line_to(x + w + 1, y + h + 1)
        dc.line_to(x + w + 1, y + 2)
        dc.stroke()

    def _draw_playhead(self, dc):
        px = self.map_stamp_to_x(self.playhead.to_sec())
        pw, ph = self._playhead_pointer_size

        # Line
        dc.set_line_width(self._playhead_line_width)
        dc.set_source_rgba(*self._playhead_color)
        dc.move_to(px, self._history_top - 1)
        dc.line_to(px, self._history_bottom + 2)
        dc.stroke()

        # Upper triangle
        py = self._history_top - ph
        dc.move_to(px,      py + ph)
        dc.line_to(px + pw, py)
        dc.line_to(px - pw, py)
        dc.line_to(px ,     py + ph)
        dc.fill()

        # Lower triangle
        py = self._history_bottom + 1
        dc.move_to(px,      py)
        dc.line_to(px + pw, py + ph)
        dc.line_to(px - pw, py + ph)
        dc.line_to(px,      py)
        dc.fill()

    ### Pixel location <-> time

    def map_x_to_stamp(self, x, clamp_to_visible=True):
        fraction = float(x - self._history_left) / self._history_width

        if clamp_to_visible:
            if fraction <= 0.0:
                return self._stamp_left
            elif fraction >= 1.0:
                return self._stamp_right

        return self._stamp_left + fraction * (self._stamp_right - self._stamp_left)

    def map_dx_to_dstamp(self, dx):
        return float(dx) * (self._stamp_right - self._stamp_left) / self._history_width

    def map_stamp_to_x(self, stamp, clamp_to_visible=True):
        fraction = (stamp - self._stamp_left) / (self._stamp_right - self._stamp_left)

        if clamp_to_visible:
            fraction = min(1.0, max(0.0, fraction))

        return self._history_left + fraction * self._history_width

    def map_dstamp_to_dx(self, dstamp):
        return (float(dstamp) * self._history_width) / (self._stamp_right - self._stamp_left)

    ### Keyboard

    def on_key_down(self, event):
        key_code = event.KeyCode
        
        if   key_code == wx.WXK_SPACE:           self.toggle_play()
        elif key_code == wx.WXK_NUMPAD_ADD:      self.navigate_fastforward()
        elif key_code == wx.WXK_NUMPAD_SUBTRACT: self.navigate_rewind()
        elif key_code == wx.WXK_HOME:            self.navigate_start()
        elif key_code == wx.WXK_END:             self.navigate_end()
        elif key_code == wx.WXK_PAGEUP:          self.zoom_in()
        elif key_code == wx.WXK_PAGEDOWN:        self.zoom_out()
        elif key_code == wx.WXK_LEFT:            self.translate_timeline((self._stamp_right - self._stamp_left) * -0.05)
        elif key_code == wx.WXK_RIGHT:           self.translate_timeline((self._stamp_right - self._stamp_left) *  0.05)
        elif key_code == wx.WXK_RETURN:          self.toggle_selecting()
        elif key_code == wx.WXK_PAUSE:           self.toggle_recording()

    ### Playing

    def _step_playhead(self):
        # Reset on switch of playing mode
        if self.playhead != self.last_playhead:
            self.last_frame       = None
            self.last_playhead    = None
            self.desired_playhead = None

        if self._play_all:
            self.step_next_message()
        else:
            self.step_fixed()

    def step_fixed(self):
        if self.play_speed == 0.0 or not self.playhead:
            self.last_frame    = None
            self.last_playhead = None
            return

        now = rospy.Time.from_sec(time.time())
        if self.last_frame:
            # Get new playhead
            if self.stick_to_end:
                new_playhead = self.end_stamp
            else:
                new_playhead = self.playhead + rospy.Duration.from_sec((now - self.last_frame).to_sec() * self.play_speed)
    
                start_stamp, end_stamp = self.play_region

                if new_playhead > end_stamp:
                    if self.wrap:
                        if self.play_speed > 0.0:
                            new_playhead = start_stamp
                        else:
                            new_playhead = end_stamp
                    else:
                        new_playhead = end_stamp

                        if self.play_speed > 0.0:
                            self.stick_to_end = True

                elif new_playhead < start_stamp:
                    if self.wrap:
                        if self.play_speed < 0.0:
                            new_playhead = end_stamp
                        else:
                            new_playhead = start_stamp
                    else:
                        new_playhead = start_stamp

            # Update the playhead
            self.playhead = new_playhead

        self.last_frame    = now
        self.last_playhead = self.playhead

    def step_next_message(self):
        if self.play_speed <= 0.0 or not self.playhead:
            self.last_frame    = None
            self.last_playhead = None
            return

        if self.last_frame:
            if not self.desired_playhead:
                self.desired_playhead = self.playhead
            else:
                delta = rospy.Time.from_sec(time.time()) - self.last_frame
                if delta > rospy.Duration.from_sec(0.1):
                    delta = rospy.Duration.from_sec(0.1)
                self.desired_playhead += delta

            # Get the occurrence of the next message
            next_message_time = self.get_next_message_time()

            if next_message_time < self.desired_playhead:
                self.playhead = next_message_time
            else:
                self.playhead = self.desired_playhead

        self.last_frame    = rospy.Time.from_sec(time.time())
        self.last_playhead = self.playhead

    ### Mouse events

    def on_left_down(self, event):
        self.SetFocus()
        self._clicked_pos = self._dragged_pos = event.Position

        self._paused = True

        if event.ShiftDown():
            return

        x, y = self._clicked_pos

        if x >= self._history_left and x <= self.history_right:
            if y >= self._history_top and y <= self._history_bottom:
                # Clicked within timeline - set playhead
                playhead_secs = self.map_x_to_stamp(x)
                
                if playhead_secs <= 0.0:
                    self.playhead = rospy.Time(0, 1)
                else:
                    self.playhead = rospy.Time.from_sec(playhead_secs)
                
                self.Refresh()
                    
            elif y <= self._history_top:
                # Clicked above timeline
                
                if self._selecting_mode == _SelectionMode.NONE:
                    self._selected_left  = None
                    self._selected_right = None
                    self._selecting_mode = _SelectionMode.LEFT_MARKED

                    self.Refresh()
                
                elif self._selecting_mode == _SelectionMode.MARKED:
                    left_x  = self.map_stamp_to_x(self._selected_left)
                    right_x = self.map_stamp_to_x(self._selected_right)
                    
                    if x < left_x - self._selection_handle_width or x > right_x + self._selection_handle_width:
                        self._selected_left  = None
                        self._selected_right = None
                        self._selecting_mode = _SelectionMode.LEFT_MARKED
                        self.Refresh()

    def on_middle_down(self, event):
        self.SetFocus()
        self._clicked_pos = self._dragged_pos = event.Position
        
        self._paused = True

    def on_right_down(self, event):
        self.SetFocus()
        self._clicked_pos = self._dragged_pos = event.Position

        self.PopupMenu(TimelinePopupMenu(self), self._clicked_pos)

    def on_left_up  (self, event): self._on_mouse_up(event)
    def on_middle_up(self, event): self._on_mouse_up(event)
    def on_right_up (self, event): pass

    def _on_mouse_up(self, event):
        self._paused = False
        
        if self._selecting_mode in [_SelectionMode.LEFT_MARKED, _SelectionMode.MOVE_LEFT, _SelectionMode.MOVE_RIGHT, _SelectionMode.SHIFTING]:
            if self._selected_left is None:
                self._selecting_mode = _SelectionMode.NONE
            else:
                self._selecting_mode = _SelectionMode.MARKED

        self.Cursor = wx.StockCursor(wx.CURSOR_ARROW)
        
        self.Refresh()

    def on_mousewheel(self, event):
        dz = event.WheelRotation / event.WheelDelta
        self.zoom_timeline(1.0 - dz * 0.2)

    def on_mouse_move(self, event):
        if not self._history_left:  # @todo: need a better notion of initialized
            return

        x, y = event.Position

        if not event.Dragging():
            # Mouse moving

            if self._selecting_mode in [_SelectionMode.MARKED, _SelectionMode.MOVE_LEFT, _SelectionMode.MOVE_RIGHT, _SelectionMode.SHIFTING]:
                if y <= self.history_top:
                    left_x  = self.map_stamp_to_x(self._selected_left)
                    right_x = self.map_stamp_to_x(self._selected_right)

                    if abs(x - left_x) <= self._selection_handle_width:
                        self._selecting_mode = _SelectionMode.MOVE_LEFT
                        self.Cursor = wx.StockCursor(wx.CURSOR_SIZEWE)
                        return
                    elif abs(x - right_x) <= self._selection_handle_width:
                        self._selecting_mode = _SelectionMode.MOVE_RIGHT
                        self.Cursor = wx.StockCursor(wx.CURSOR_SIZEWE)
                        return
                    elif x > left_x and x < right_x:
                        self._selecting_mode = _SelectionMode.SHIFTING
                        self.Cursor = wx.StockCursor(wx.CURSOR_SIZING)
                        return
                    else:
                        self._selecting_mode = _SelectionMode.MARKED

            self.Cursor = wx.StockCursor(wx.CURSOR_ARROW)
    
        else:
            # Mouse dragging

            if event.MiddleIsDown() or event.ShiftDown():
                # Middle or shift: zoom
                
                dx_click, dy_click = x - self._clicked_pos[0], y - self._clicked_pos[1]
                dx_drag,  dy_drag  = x - self._dragged_pos[0], y - self._dragged_pos[1]
                
                if dx_drag != 0:
                    self.translate_timeline(-self.map_dx_to_dstamp(dx_drag))
                if (dx_drag == 0 and abs(dy_drag) > 0) or (dx_drag != 0 and abs(float(dy_drag) / dx_drag) > 0.2 and abs(dy_drag) > 1):
                    zoom = min(self._max_zoom_speed, max(self._min_zoom_speed, 1.0 + self._zoom_sensitivity * dy_drag))
                    self.zoom_timeline(zoom)
    
                self.Cursor = wx.StockCursor(wx.CURSOR_HAND)
    
            elif event.LeftIsDown():
                clicked_x, clicked_y = self._clicked_pos
    
                x_stamp = self.map_x_to_stamp(x)

                if self._selecting_mode == _SelectionMode.LEFT_MARKED:
                    # Left and selecting: change selection region

                    clicked_x_stamp = self.map_x_to_stamp(clicked_x)
                    
                    self._selected_left  = min(clicked_x_stamp, x_stamp)
                    self._selected_right = max(clicked_x_stamp, x_stamp)
    
                    self.Refresh()
                    
                elif self._selecting_mode == _SelectionMode.MOVE_LEFT:
                    self._selected_left = x_stamp
                    self.Refresh()
                    
                elif self._selecting_mode == _SelectionMode.MOVE_RIGHT:
                    self._selected_right = x_stamp
                    self.Refresh()

                elif self._selecting_mode == _SelectionMode.SHIFTING:
                    dx_drag = x - self._dragged_pos[0]
                    dstamp = self.map_dx_to_dstamp(dx_drag)

                    self._selected_left  = max(self._start_stamp.to_sec(), min(self._end_stamp.to_sec(), self._selected_left  + dstamp))
                    self._selected_right = max(self._start_stamp.to_sec(), min(self._end_stamp.to_sec(), self._selected_right + dstamp))
                    self.Refresh()

                elif clicked_x >= self._history_left and clicked_x <= self.history_right and clicked_y >= self._history_top and clicked_y <= self._history_bottom:
                    # Left and clicked within timeline: change playhead
                    
                    if x_stamp <= 0.0:
                        self.playhead = rospy.Time(0, 1)
                    else:
                        self.playhead = rospy.Time.from_sec(x_stamp)
                        
                    self.Refresh()
    
            self._dragged_pos = event.Position

    def toggle_selecting(self):
        """
        Transitions selection mode from NONE -> LEFT_MARKED -> MARKED -> NONE
        """
        if self._selecting_mode == _SelectionMode.NONE:
            self._selected_left  = self._playhead.to_sec()
            self._selected_right = None
            
            self._selecting_mode = _SelectionMode.LEFT_MARKED

        elif self._selecting_mode == _SelectionMode.LEFT_MARKED:
            current_mark = self._selected_left
            self._selected_left  = min(current_mark, self._playhead.to_sec())
            self._selected_right = max(current_mark, self._playhead.to_sec())

            self._selecting_mode = _SelectionMode.MARKED

        elif self._selecting_mode == _SelectionMode.MARKED:
            self._selected_left  = None
            self._selected_right = None
            
            self._selecting_mode = _SelectionMode.NONE

        self.Refresh()

class IndexCacheThread(threading.Thread):
    """
    Updates invalid caches.
    
    One thread per timeline.
    """
    def __init__(self, timeline):
        threading.Thread.__init__(self)

        self.timeline  = timeline

        self._stop_flag = False

        self.setDaemon(True)
        self.start()

    def run(self):
        last_updated_topic = None
        
        while not self._stop_flag:
            with self.timeline.index_cache_cv:
                # Wait until the cache is dirty
                while len(self.timeline.invalidated_caches) == 0:
                    self.timeline.index_cache_cv.wait()
                    if self._stop_flag:
                        return

                # Update the index for one topic
                updated = False
                for topic in self.timeline.topics:
                    if topic in self.timeline.invalidated_caches and topic != last_updated_topic:
                        updated = (self.timeline._update_index_cache(topic) > 0)
                        if updated:
                            last_updated_topic = topic
                            break

            if updated:
                wx.CallAfter(self.timeline.Refresh)

                # Give the GUI some time to update
                time.sleep(0.2)

    def stop(self):
        self._stop_flag = True
        cv = self.timeline.index_cache_cv
        with cv:
            cv.notify()
        self.join()

class MessageLoader(threading.Thread):
    """
    Waits for a new playhead position on the given topic, then loads the message at that position and notifies the view threads.

    One thread per topic.  Maintains a cache of recently loaded messages.
    """
    def __init__(self, timeline, topic):
        threading.Thread.__init__(self)

        self.timeline = timeline
        self.topic    = topic
        
        self.bag_playhead_position = None

        self._message_cache_capacity = 50
        self._message_cache          = {}
        self._message_cache_keys     = []
        
        self._stop_flag = False
        
        self.setDaemon(True)
        self.start()

    def reset(self):
        self.bag_playhead_position = None

    def run(self):
        while not self._stop_flag:
            # Wait for a new entry
            cv = self.timeline._playhead_positions_cvs[self.topic]
            with cv:
                while (self.topic not in self.timeline._playhead_positions) or (self.bag_playhead_position == self.timeline._playhead_positions[self.topic]):
                    cv.wait()
                    if self._stop_flag:
                        return
                bag, playhead_position = self.timeline._playhead_positions[self.topic]

            self.bag_playhead_position = (bag, playhead_position)

            # Don't bother loading the message if there are no listeners
            if not self.timeline.has_listeners(self.topic):
                continue

            # Load the message
            if playhead_position is None:
                msg_data = None
            else:
                msg_data = self._get_message(bag, playhead_position)

            # Inform the views
            messages_cv = self.timeline._messages_cvs[self.topic]
            with messages_cv:
                self.timeline._messages[self.topic] = (bag, msg_data)
                messages_cv.notify_all()      # notify all views that a message is loaded

    def _get_message(self, bag, position):
        key = '%s%s' % (bag.filename, str(position))
        if key in self._message_cache:
            return self._message_cache[key]

        msg_data = self.timeline.read_message(bag, position)

        self._message_cache[key] = msg_data
        self._message_cache_keys.append(key)

        if len(self._message_cache) > self._message_cache_capacity:
            oldest_key = self._message_cache_keys[0]
            del self._message_cache[oldest_key]
            self._message_cache_keys.remove(oldest_key)

        return msg_data

    def stop(self):
        self._stop_flag = True
        cv = self.timeline._playhead_positions_cvs[self.topic]
        with cv:
            cv.notify_all()
        self.join()

class MessageListenerThread(threading.Thread):
    """
    Waits for new messages loaded on the given topic, then calls the message listener.
    
    One thread per listener, topic pair.
    """
    def __init__(self, timeline, topic, listener):
        threading.Thread.__init__(self)

        self.timeline = timeline
        self.topic    = topic
        self.listener = listener

        self.bag_msg_data = None

        self._stop_flag = False

        self.setDaemon(True)
        self.start()

    def run(self):
        while not self._stop_flag:
            # Wait for a new message
            cv = self.timeline._messages_cvs[self.topic]
            with cv:
                while (self.topic not in self.timeline._messages) or (self.bag_msg_data == self.timeline._messages[self.topic]):
                    cv.wait()
                    if self._stop_flag:
                        return
                bag_msg_data = self.timeline._messages[self.topic]

            # View the message
            self.bag_msg_data = bag_msg_data
            try:
                bag, msg_data = bag_msg_data
                if msg_data:
                    self.listener.message_viewed(bag, msg_data)
                else:
                    self.listener.message_cleared()
            except wx.PyDeadObjectError:
                self.timeline.remove_listener(self.topic, self.listener)

    def stop(self):
        self._stop_flag = True
        cv = self.timeline._messages_cvs[self.topic]
        with cv:
            cv.notify_all()
        self.join()
