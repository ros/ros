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
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICTS
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

PKG = 'rxbag'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag

import bisect
import cPickle
import math
import os
import sys
import threading
import time

import numpy

import util.progress_meter

class BagIndex:
    def __init__(self, bag):
        self.topic_datatypes = {}   # topic -> datatype
        self.msg_positions   = {}   # topic -> [(timestamp, file offset), ...]

        self._dirty_stats = True
        self._start_stamp = None
        self._end_stamp   = None
        
        print 'Building index'
        
        for (topic, msg, t) in bag.read_messages(raw=True):
            if not topic in self.topic_datatypes:
                datatype, data, md5sum, pos, msg_type = msg

                self.topic_datatypes[topic] = datatype
                self.msg_positions[topic] = []

            entry = (t.to_sec(), pos)

            topic_msg_positions = self.msg_positions[topic]
            topic_msg_positions.insert(bisect.bisect(topic_msg_positions, entry), entry)

        print 'Done with index'

    @property
    def start_stamp(self):
        self._update_statistics()
        return self._start_stamp

    @property
    def end_stamp(self):
        self._update_statistics()
        return self._end_stamp

    def get_topics_by_datatype(self):
        topics_by_datatype = {}
        for topic, datatype in self.topic_datatypes.items():
            topics_by_datatype.setdefault(datatype, []).append(topic)
        return topics_by_datatype

    @property
    def topics(self): return self.topic_datatypes.keys()

    @property
    def datatypes(self): return self.datatype_first_pos

    @property
    def msg_positions(self): return self.msg_positions

    def get_datatype(self, topic): return self.topic_datatypes.get(topic)

    def _update_statistics(self):
        if not self._dirty_stats:
            return
        
        self._start_stamp = self.get_start_stamp()
        self._end_stamp   = self.get_end_stamp()
        self._dirty_stats = False

    @staticmethod
    def stamp_to_str(secs):
        secs_frac = secs - int(secs) 
        secs_frac_str = ('%.2f' % secs_frac)[1:]

        return time.strftime('%b %d %Y %H:%M:%S', time.localtime(secs)) + secs_frac_str

    def get_topic_start_stamp(self, topic):
        """@return: first timestamp for given topic"""
        if topic not in self.msg_positions or len(self.msg_positions[topic]) == 0:
            return None
        return self.msg_positions[topic][0][0]

    def get_topic_end_stamp(self, topic):
        """@return: last timestamp for given topic"""
        if topic not in self.msg_positions or len(self.msg_positions[topic]) == 0:
            return None
        return self.msg_positions[topic][-1][0]

    def get_start_stamp(self):
        """@return: earliest timestamp in the index"""
        topic_start_stamps = [self.get_topic_start_stamp(topic) for topic in self.msg_positions.keys() if len(self.msg_positions[topic]) > 0]
        if len(topic_start_stamps) == 0:
            return None
        return min(topic_start_stamps)

    def get_end_stamp(self):
        """@return: latest timestamp in the index"""
        topic_end_stamps = [self.get_topic_end_stamp(topic) for topic in self.msg_positions.keys() if len(self.msg_positions[topic]) > 0]
        if len(topic_end_stamps) == 0:
            return None
        return max(topic_end_stamps)

    def find_stamp_position(self, topic, stamp):
        """Binary search to find position before given timestamp"""
        if topic not in self.msg_positions:
            return None
        
        index = self.find_stamp_index(topic, stamp)
        if not index:
            return None
        
        return self.msg_positions[topic][index][1]

    def find_stamp_index(self, topic, stamp):
        """Binary search to find first index in topic before given timestamp"""
        if topic not in self.msg_positions or len(self.msg_positions[topic]) == 0:
            return None

        index = bisect.bisect_right(self.msg_positions[topic], (stamp, 0))

        return max(0, min(index, len(self.msg_positions[topic]) - 1))
