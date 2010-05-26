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

from __future__ import with_statement

PKG = 'rxbag'
import roslib; roslib.load_manifest(PKG)
import rospy

import bisect
import Queue
import sys
import threading
import time

import wx

class TimelineCache(threading.Thread):
    def __init__(self, loader, listener=None, max_cache_size=50):
        threading.Thread.__init__(self)

        self.loader    = loader
        self.listener  = listener
        self.stop_flag = False

        self.lock = threading.RLock()

        self.items         = {}     # topic -> [(timestamp, items), ...]
        self.last_accessed = {}     # topic -> [(access time, timestamp), ...]
        self.item_access   = {}     # topic -> timestamp -> access time

        self.max_cache_size = max_cache_size    # max number of items to cache (per topic)

        self.queue = Queue.Queue()

        self.setDaemon(True)
        self.start()

        self._debugging = False
        if self._debugging:
            self._debug_frame = wx.Frame(None, -1, 'TimelineCache debug')
            self._debug_text = wx.TextCtrl(self._debug_frame, style=wx.TE_READONLY | wx.NO_BORDER | wx.TE_MULTILINE)
            self._debug_text.SetPosition((1, 0))
            self._debug_text.SetFont(wx.Font(7, wx.FONTFAMILY_MODERN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
            self._debug_frame.SetSize((400, 1200))
            self._debug_frame.Show()

    def run(self):
        try:
            while not self.stop_flag:
                # Get next item to load
                entry = self.queue.get()
                
                # self used to signal a change in stop_flag
                if entry == self:
                    continue

                # Update debugging info (if enabled)
                if self._debugging:
                    wx.CallAfter(self._update_debug)

                # Check we haven't already cached it
                topic, stamp, time_threshold, item_details = entry

                if not self.get_item(topic, stamp, time_threshold):
                    # Load the item
                    msg_stamp, item = self.loader(topic, stamp, item_details)
                    if item:
                        # Store in the cache
                        self.cache_item(topic, msg_stamp, item)

                        if self.listener:
                            self.listener(topic, msg_stamp, item)
                    else:
                        print 'Failed to load:', entry

                self.queue.task_done()
                
        except Exception, ex:
            print ex
            raise
        
    def enqueue(self, entry):
        self.queue.put(entry)

    def cache_item(self, topic, t, item):
        with self.lock:
            if topic not in self.items:
                self.items[topic] = []
            topic_cache = self.items[topic]
    
            cache_entry = (t.to_sec(), item)
            cache_index = bisect.bisect_right(topic_cache, cache_entry)
            topic_cache.insert(cache_index, cache_entry)

            self._update_last_accessed(topic, t.to_sec())
    
            self._limit_cache()

    def get_item(self, topic, stamp, time_threshold):
        with self.lock:
            # Attempt to get a item from the cache that's within time_threshold secs from stamp
            topic_cache = self.items.get(topic)
            if topic_cache:
                cache_index = max(0, bisect.bisect_right(topic_cache, (stamp, None)) - 1)
                
                if cache_index <= len(topic_cache) - 1:
                    # Get cache entry before (or at) timestamp, and entry after
                    (cache_before_stamp, cache_before_item) = topic_cache[cache_index]
                    if cache_index < len(topic_cache) - 1:
                        cache_after_stamp, cache_after_item = topic_cache[cache_index + 1]
                    else:
                        cache_after_stamp = None
    
                    # Find closest entry
                    cache_before_dist = abs(stamp - cache_before_stamp)
                    if cache_after_stamp:
                        cache_after_dist = abs(cache_after_stamp - stamp)
                        
                    if cache_after_stamp and cache_after_dist < cache_before_dist:
                        cache_dist, cache_stamp, cache_item = cache_after_dist, cache_after_stamp, cache_after_item
                    else:
                        cache_dist, cache_stamp, cache_item = cache_before_dist, cache_before_stamp, cache_before_item
    
                    # Check entry is close enough
                    if cache_dist <= time_threshold:
                        self._update_last_accessed(topic, cache_stamp)
                        
                        return cache_item
    
            return None

    def _update_last_accessed(self, topic, stamp):
        """
        Maintains a sorted list of cache accesses by timestamp for each topic.
        """
        with self.lock:
            access_time = time.time()
    
            if topic not in self.last_accessed:
                self.last_accessed[topic] = [(access_time, stamp)]
                self.item_access[topic] = { stamp: access_time }
                return
    
            topic_last_accessed = self.last_accessed[topic]
            topic_item_access = self.item_access[topic]
    
            if stamp in topic_item_access:
                last_access = topic_item_access[stamp]
                
                index = bisect.bisect_left(topic_last_accessed, (last_access, None))
                assert(topic_last_accessed[index][1] == stamp)
    
                del topic_last_accessed[index]
    
            topic_last_accessed.append((access_time, stamp))
            topic_item_access[stamp] = access_time

    def _limit_cache(self):
        """
        Removes LRU's from cache until size of each topic's cache is <= max_cache_size.
        """
        with self.lock:
            for topic, topic_cache in self.items.items():
                while len(topic_cache) > self.max_cache_size:
                    lru_stamp = self.last_accessed[topic][0][1]
                    
                    cache_index = bisect.bisect_left(topic_cache, (lru_stamp, None))
                    assert(topic_cache[cache_index][0] == lru_stamp)
    
                    del topic_cache[cache_index]
                    del self.last_accessed[topic][0]
                    del self.item_access[topic][lru_stamp]

    def stop(self):
        self.stop_flag = True
        self.queue.put(self)

    def _update_debug(self):
        try:
            s = ''
    
            start_stamp = self.timeline.start_stamp.to_sec()
    
            for topic, topic_cache in self.items.items():
                s += topic + '\n'
                for t, _ in topic_cache:
                    s += '%.5f\n' % (t - start_stamp)

                s += 'Last Accessed:\n'
                topic_last_accessed = self.last_accessed[topic]
                for access_time, t in topic_last_accessed:
                    s += '%.02f: %.5f\n' % (time.time() - access_time, t - start_stamp)
    
            self._debug_text.SetValue(s)
            
        except Exception:
            pass
