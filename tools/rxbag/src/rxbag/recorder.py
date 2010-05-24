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
Recorder subscribes to ROS messages and writes them to a bag file.
"""

from __future__ import with_statement

PKG = 'rxbag'
import roslib; roslib.load_manifest(PKG)

import Queue
import re
import threading
import time

import rosbag
import rosgraph.masterapi
import rospy

import sys

class Recorder(object):
    def __init__(self, filename, bag_lock=None, all=True, topics=[], regex=False, limit=0, master_check_interval=1.0):
        """
        Subscribe to ROS messages and record them to a bag file.
        
        @param filename: filename of bag to write to
        @type  filename: str
        @param all: all topics are to be recorded [default: True]
        @type  all: bool
        @param topics: topics (or regexes if regex is True) to record [default: empty list]
        @type  topics: list of str
        @param regex: topics should be considered as regular expressions [default: False]
        @type  regex: bool
        @param limit: record only this number of messages on each topic (if non-positive, then unlimited) [default: 0]
        @type  limit: int
        @param master_check_interval: period (in seconds) to check master for new topic publications [default: 1]
        @type  master_check_interval: float
        """
        self._all                   = all
        self._topics                = topics
        self._regex                 = regex
        self._limit                 = limit
        self._master_check_interval = master_check_interval

        self._bag                = rosbag.Bag(filename, 'w')
        self._bag_lock           = bag_lock if bag_lock else threading.Lock()
        self._listeners          = []
        self._subscriber_helpers = {}
        self._limited_topics     = set()
        self._failed_topics      = set()       
        self._last_update        = time.time()
        self._write_queue        = Queue.Queue()
        self._paused             = False
        self._stop_condition     = threading.Condition()
        self._stop_flag          = False

        # Compile regular expressions
        if self._regex:
            self._regexes = [re.compile(t) for t in self._topics]
        else:
            self._regexes = None

        self._message_count = {}  # topic -> int (track number of messages recorded on each topic)

        self._master_check_thread = threading.Thread(target=self._run_master_check)
        self._write_thread        = threading.Thread(target=self._run_write)

    @property
    def bag(self): return self._bag

    def add_listener(self, listener):
        """
        Add a listener which gets called whenever a message is recorded.
        @param listener: function to call
        @type  listener: function taking (topic, message, time)
        """
        self._listeners.append(listener)

    def start(self):
        """
        Start subscribing and recording messages to bag.
        """
        self._master_check_thread.start()
        self._write_thread.start()

    @property
    def paused(self):        return self._paused
    def pause(self):         self._paused = True
    def unpause(self):       self._paused = False
    def toggle_paused(self): self._paused = not self._paused

    def stop(self):
        """
        Stop recording.
        """
        with self._stop_condition:
            self._stop_flag = True
            self._stop_condition.notify_all()
        
        self._write_queue.put(self)

    ## Implementation

    def _run_master_check(self):
        master = rosgraph.masterapi.Master('rxbag.recorder')

        try:
            while not self._stop_flag:
                # Check for new topics
                for topic, datatype in master.getPublishedTopics(''):
                    # Check if: 
                    #    the topic is already subscribed to, or
                    #    we've failed to subscribe to it already, or
                    #    we've already reached the message limit, or
                    #    we don't want to subscribe 
                    if topic in self._subscriber_helpers or topic in self._failed_topics or topic in self._limited_topics or not self._should_subscribe_to(topic):
                        continue

                    try:
                        pytype = roslib.message.get_message_class(datatype)
                        
                        self._message_count[topic] = 0

                        self._subscriber_helpers[topic] = _SubscriberHelper(self, topic, pytype)
                    except Exception, ex:
                        print >> sys.stderr, 'Error subscribing to %s (ignoring): %s' % (topic, str(ex))
                        self._failed_topics.add(topic)

                # Wait a while
                self._stop_condition.acquire()
                self._stop_condition.wait(self._master_check_interval)

        except Exception, ex:
            print >> sys.stderr, 'Error recording to bag: %s' % str(ex)

        # Unsubscribe from all topics
        for topic in list(self._subscriber_helpers.keys()):
            self._unsubscribe(topic)
        
        # Close the bag file so that the index gets written
        try:
            self._bag.close()
        except Exception, ex:
            print >> sys.stderr, 'Error closing bag [%s]: %s' % (self._bag.filename, str(ex))

    def _should_subscribe_to(self, topic):
        if self._all:
            return True
        
        if not self._regex:
            return topic in self._topics

        for regex in self._regexes:
            if regex.match(topic):
                return True
            
        return False
    
    def _unsubscribe(self, topic):
        try:
            self._subscriber_helpers[topic].subscriber.unregister()
        except Exception:
            return

        del self._subscriber_helpers[topic]

    def _record(self, topic, m):
        if self._paused:
            return

        if self._limit and self._message_count[topic] >= self._limit:
            self._limited_topics.add(topic)
            self._unsubscribe(topic)
            return

        self._write_queue.put((topic, m, rospy.get_rostime()))
        self._message_count[topic] += 1

    def _run_write(self):
        try:
            while not self._stop_flag:
                # Wait for a message
                item = self._write_queue.get()
                
                if item == self:
                    continue
                
                topic, m, t = item
                
                # Write to the bag
                with self._bag_lock:
                    self._bag.write(topic, m, t)

                # Notify listeners that a message has been recorded
                for listener in self._listeners:
                    listener(topic, m, t)

        except Exception, ex:
            print >> sys.stderr, 'Error write to bag: %s' % str(ex)

class _SubscriberHelper(object):
    def __init__(self, recorder, topic, pytype):
        self.recorder = recorder
        self.topic    = topic

        self.subscriber = rospy.Subscriber(self.topic, pytype, self.callback)

    def callback(self, m):
        self.recorder._record(self.topic, m)
