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

PKG = 'rxplay'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosrecord

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
    def __init__(self):
        self._data              = BagIndexData()
        self.datatype_defs_read = None
        self.loaded             = False

        self._dirty_stats = True
        self._start_stamp = None
        self._end_stamp   = None

    @property
    def start_stamp(self):
        self.update_statistics()
        return self._start_stamp

    @property
    def end_stamp(self):
        self.update_statistics()
        return self._end_stamp

    def get_topics_by_datatype(self):
        topics_by_datatype = {}
        for topic, datatype in self._data.topic_datatypes.items():
            topics_by_datatype.setdefault(datatype, []).append(topic)
        return topics_by_datatype

    @property
    def topics(self): return self._data.topic_datatypes.keys()

    @property
    def datatypes(self): return self._data.datatype_first_pos

    @property
    def msg_positions(self): return self._data.msg_positions
    
    def get_datatype(self, topic): return self._data.topic_datatypes.get(topic)
        
    def update_statistics(self):
        if not self._dirty_stats:
            return
        
        self._start_stamp = self._data.get_start_stamp()
        self._end_stamp = self._data.get_end_stamp()
        self._dirty_stats = False

    ## Add a record to the index
    def add_record(self, pos, topic, datatype, stamp):
        if not topic in self._data.topic_datatypes:
            # A topic's been seen for the first time
            self._data.topic_datatypes[topic] = datatype
            self._data.msg_positions[topic] = []

            # If the topic has a new message datatype, record the first position
            if datatype not in self._data.datatype_first_pos:
                self._data.datatype_first_pos[datatype] = pos
                if not self.datatype_defs_read:
                    self.datatype_defs_read = set()

                self.datatype_defs_read.add(datatype)

        # Record the timestamp and position
        self._data.msg_positions[topic].append((stamp.to_sec(), pos))

        self._dirty_stats = True

    @staticmethod
    def stamp_to_str(secs):
        secs_frac = secs - int(secs) 
        secs_frac_str = ('%.2f' % secs_frac)[1:]

        return time.strftime('%b %d %Y %H:%M:%S', time.localtime(secs)) + secs_frac_str

    def __str__(self):
        if self.start_stamp is None:
            return '<empty>'

        duration = self.end_stamp - self.start_stamp

        dur_secs = duration % 60
        dur_mins = duration / 60
        dur_hrs = dur_mins / 60
        if dur_hrs > 0:
            dur_mins = dur_mins % 60
            s = 'Duration: %dhr %dmin %ds (%ds)\n' % (dur_hrs, dur_mins, dur_secs, duration)
        else:
            s = 'Duration: %dmin %ds (%ds)\n' % (dur_mins, dur_secs, duration)

        s += 'Start:    %s (%.2f)\n' % (self.stamp_to_str(self.start_stamp), self.start_stamp)
        s += 'End:      %s (%.2f)\n' % (self.stamp_to_str(self.end_stamp), self.end_stamp)
        s += 'Messages: %d\n' % (sum([len(p) for p in self._data.msg_positions.values()]))

        s += 'Topics:'
        max_topic_len = max([len(topic) for topic in self.topics])
        max_datatype_len = max([len(self.get_datatype(topic)) for topic in self.topics]) 
        for i, topic in enumerate(sorted(self.topics)):
            indent = (3 if i == 0 else 10)

            positions = numpy.array([stamp for (stamp, pos) in self.msg_positions[topic]])
            datatype = self.get_datatype(topic)
            msg_count = len(positions)

            s += '%s%-*s : %-*s %7d msgs' % (' ' * indent, max_datatype_len, datatype, max_topic_len, topic, msg_count)

            if msg_count > 1:
                spacing = positions[1:] - positions[:-1]
                s += ' @ %5.1f Hz' % (1.0 / numpy.median(spacing))

            s += '\n'

        return s

## The data for a bag index, allows for random access of a bag file. Separate from BagIndex for pickling.
## Currently contains:
##   - positions of first occurrence of datatype: { datatype : datatype_first_pos, ... }
##   - datatypes of topics:                       { topic : datatype, ... }
##   - positions of messages:                     { topic : [(timestamp, position), ...], ... }
class BagIndexData:
    def __init__(self):
        self.datatype_first_pos = {}   # datatype -> file offset
        self.topic_datatypes = {}   # topic -> datatype
        self.msg_positions = {}   # topic -> [(timestamp, file offset), ...]

    ## Return first timestamp for a given topic
    def get_topic_start_stamp(self, topic):
        if topic not in self.msg_positions or len(self.msg_positions[topic]) == 0:
            return None
        return self.msg_positions[topic][0][0]

    ## Return last timestamp for a given topic
    def get_topic_end_stamp(self, topic):
        if topic not in self.msg_positions or len(self.msg_positions[topic]) == 0:
            return None
        return self.msg_positions[topic][-1][0]

    ## Returns the earliest timestamp in the index
    def get_start_stamp(self):
        topic_start_stamps = [self.get_topic_start_stamp(topic) for topic in self.msg_positions.keys() if len(self.msg_positions[topic]) > 0]
        if len(topic_start_stamps) == 0:
            return None
        return min(topic_start_stamps)

    ## Returns the latest timestamp in the index
    def get_end_stamp(self):
        topic_end_stamps = [self.get_topic_end_stamp(topic) for topic in self.msg_positions.keys() if len(self.msg_positions[topic]) > 0]
        if len(topic_end_stamps) == 0:
            return None
        return max(topic_end_stamps)

    ## Binary search to find position before given timestamp
    def find_stamp_position(self, topic, stamp):
        if topic not in self.msg_positions:
            return None
        
        index = self.find_stamp_index(topic, stamp)
        if not index:
            return None
        
        return self.msg_positions[topic][index][1]

    ## Binary search to find first index in topic before given timestamp 
    def find_stamp_index(self, topic, stamp):
        if topic not in self.msg_positions or len(self.msg_positions[topic]) == 0:
            return None

        index = bisect.bisect_right(self.msg_positions[topic], (stamp, 0))

        return max(0, min(index, len(self.msg_positions[topic]) - 1))

## Reads an index from a bag file
class BagIndexReader:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        
        self.index = BagIndex()

    def load(self):
        try:
            bag_file = rosrecord.BagReader(self.bag_path)

            if bag_file.read_index() is None:
                self.index = None
                return None

            # Read the index in ascending order of topic's first message position (ensures that each datatype message definition gets read)
            topic_first_positions = sorted([(ti[0][1], t) for t, ti in bag_file.index.items()])

            for _, topic in topic_first_positions:
                topic_index = bag_file.index[topic]
                datatype    = bag_file.datatypes[topic]

                for stamp, pos in topic_index:
                    self.index.add_record(pos, topic, datatype, stamp)

            self.index.loaded = True

        except Exception, e:
            rospy.logerr('Unsuccessful loading index: %s' % e)
            self.index = None

        return self.index

## Serialize bag file index to/from a binary file
class BagIndexPickler:
    def __init__(self, index_path):
        self.index_path = index_path
    
    def load(self):
        try:
            # Open the index file
            rospy.logdebug('Opening index: %s' % self.index_path)
            index_file = file(self.index_path, 'rb')
            
            # Load the index data
            index = BagIndex()
            index._data = cPickle.load(index_file)
            index.loaded = True

            rospy.logdebug('Successful.\n\n%s\n%s\n%s' % (self.index_path, '-' * len(self.index_path), index))

            return index

        except Exception, e:
            rospy.logerr('Unsuccessful loading index: %s' % e)
            return None

    def save(self, index):
        try:
            index_file = file(self.index_path, 'wb')
            cPickle.dump(index._data, index_file, -1)

            return True

        except Exception, e:
            rospy.logerr('Error writing index.')
            return False
        
        finally:
            index_file.close()

## Constructs a bag index object given a path to a bag file
class BagIndexFactory:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        
        self.index = BagIndex()

    def load(self):
        try:
            bag_file = rosrecord.BagReader(self.bag_path)
            
            if bag_file.read_index() is not None:
                for topic, topic_index in bag_file.index.items():
                    datatype = bag_file.datatypes[topic]
                    
                    for stamp, pos in topic_index:
                        self.index.add_record(pos, topic, datatype, stamp)
            else:
                file_size = os.path.getsize(self.bag_path)
                
                progress = util.progress_meter.ProgressMeter(self.bag_path, file_size, 1.0)
                
                for i, (pos, topic, raw_msg, stamp) in enumerate(bag_file.raw_messages()):
                    (datatype, message_data, md5, bag_pos, pytype) = raw_msg
                
                    self.index.add_record(pos, topic, datatype, stamp)
                
                    progress.step(pos)
                
                progress.finish()
            
            self.index.loaded = True

        except Exception, e:
            rospy.logerr('Unsuccessful creating index from %s: %s' % (self.bag_path, e))
            self.index = None
            
        return self.index
