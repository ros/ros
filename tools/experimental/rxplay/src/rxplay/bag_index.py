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

PKG = 'rxplay'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosrecord

import bisect
import cPickle
import math
import os
import threading
import time

import numpy

from bag_file import BagFile
import timeline

class BagIndex:
    def __init__(self):
        self._data              = BagIndexData()
        self.datatype_defs_read = None
        self.loaded             = False

    @property
    def topics(self): return self._data.topic_datatypes.keys()

    @property
    def datatypes(self): return self._data.datatype_first_pos

    @property
    def msg_positions(self): return self._data.msg_positions
    
    def get_datatype(self, topic): return self._data.topic_datatypes.get(topic)
    
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
        
    def __str__(self):
        start_stamp = self._data.get_start_stamp()
        end_stamp   = self._data.get_end_stamp()
        duration    = end_stamp - start_stamp
        
        s  = 'Duration: %d min %d secs\n' % (int(math.floor(duration / 60)), duration % 60)
        s += 'Start:    %s (%.2f)\n' % (timeline.Timeline.stamp_to_str(start_stamp), start_stamp)
        s += 'End:      %s (%.2f)\n' % (timeline.Timeline.stamp_to_str(end_stamp),   end_stamp)
        s += 'Messages: %d\n' % (sum([len(p) for p in self._data.msg_positions.values()]))

        s += 'Topics:'
        max_topic_len    = max([len(topic) for topic in self.topics])
        max_datatype_len = max([len(self.get_datatype(topic)) for topic in self.topics]) 
        for i, topic in enumerate(sorted(self.topics)):
            indent = (3 if i == 0 else 10)
            
            positions = numpy.array([stamp for (stamp, pos) in self.msg_positions[topic]])
            datatype  = self.get_datatype(topic)
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
        self.topic_datatypes    = {}   # topic -> datatype
        self.msg_positions      = {}   # topic -> [(timestamp, file offset), ...]

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

## Serialize bag file index to/from a binary file
class BagIndexPickler:
    def __init__(self, index_path):
        self.index_path = index_path
    
    def load(self):
        try:
            # Open the index file
            rospy.loginfo('Opening index: %s' % self.index_path)
            index_file = file(self.index_path, 'rb')
            
            # Load the index data
            index = BagIndex()
            index._data = cPickle.load(index_file)
            index.loaded = True

            rospy.loginfo('Successful.\n\n%s\n%s\n%s' % (self.index_path, '-' * len(self.index_path), index))
            
            return index
        
        except Exception, e:
            rospy.logerr('Unsuccessful loading index from %s: %s' % (self.index_path, e))
            return None

    def save(self, index):
        try:
            index_file = file(self.index_path, 'wb')
            cPickle.dump(index._data, index_file, -1)

            rospy.loginfo('Index saved to %s' % self.index_path)

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
            bag_file = BagFile(self.bag_path)

            file_size = os.path.getsize(self.bag_path)
            last_complete_fraction = 0
            
            for i, (pos, topic, raw_msg, stamp) in enumerate(bag_file.raw_messages()):
                (datatype, message_data, md5, bag_pos, pytype) = raw_msg
   
                self.index.add_record(pos, topic, datatype, stamp)
    
                # Output % complete
                complete_fraction = int(100.0 * float(pos) / file_size)
                if complete_fraction != last_complete_fraction:
                    rospy.loginfo('%d%% complete' % complete_fraction)
                    last_complete_fraction = complete_fraction

            self.index.loaded = True
            
            rospy.loginfo('Successful.')

        except Exception, e:
            rospy.logerr('Unsuccessful creating index from %s: %s' % (self.bag_path, e))
            self.index = None
            
        return self.index
