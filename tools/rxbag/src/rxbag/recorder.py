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

import threading
import time

import rosbag
import rosgraph.masterapi
import rospy

import sys

class Recorder(threading.Thread):   
    def __init__(self, filename, bag_lock=None, master_check_interval=5):
        """
        Subscribe to ROS messages and record them to a bag file.
        @param filename: filename of bag to write to
        @type  filename: str
        @param master_check_interval: period to check master for new topic publications [default: 5]
        @type  master_check_interval: float
        """
        threading.Thread.__init__(self, target=self._run)

        self._master_check_interval = master_check_interval
       
        self._bag                = rosbag.Bag(filename, 'w')
        self._bag_lock           = bag_lock if bag_lock else threading.Lock()
        self._listeners          = []
        self._subscriber_helpers = {}
        self._failed_topics      = set()       
        self._last_update        = time.time()

        self._stop_condition = threading.Condition()
        self._stop_flag      = False

        self.setDaemon(False)

    @property
    def bag(self): return self._bag

    def add_listener(self, listener):
        """
        Add a listener which gets called whenever a message is recorded.
        @param listener: function to call
        @type  listener: function taking (topic, message, time)
        """
        self._listeners.append(listener)

    def stop(self):
        """
        Stop recording.
        """
        self._stop_condition.acquire()
        self._stop_flag = True
        self._stop_condition.notify()
        self._stop_condition.release()

    ## Implementation

    def _run(self):
        master = rosgraph.masterapi.Master('rxbag.recorder')
        
        try:
            while not self._stop_flag:
                # Subscribe to any new topics
                for topic, datatype in master.getPublishedTopics(''):
                    if topic not in self._subscriber_helpers and topic not in self._failed_topics:
                        print 'Found new topic: %s [%s]' % (topic, datatype)
                        
                        try:
                            pytype = roslib.message.get_message_class(datatype)
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
        for subscriber_helper in self._subscriber_helpers.values():
            subscriber_helper.subscriber.unregister()
        
        # Close the bag file so that the index gets written
        try:
            self._bag.close()
        except Exception, ex:
            print >> sys.stderr, 'Error closing bag [%s]: %s' % (self._bag.filename, str(ex))

    def _record(self, topic, m):
        # Write the message to the bag
        t = roslib.rostime.Time.from_sec(time.time())
        with self._bag_lock:
            self._bag.write(topic, m, t)

        # Notify listeners that a message has been recorded
        for listener in self._listeners:
            listener(topic, m, t)

class _SubscriberHelper(object):
    def __init__(self, recorder, topic, pytype):
        self.recorder = recorder
        self.topic    = topic

        self.subscriber = rospy.Subscriber(self.topic, pytype, self.callback)

    def callback(self, m):
        self.recorder._record(self.topic, m)
