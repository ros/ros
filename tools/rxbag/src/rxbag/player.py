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
Player listens to messages from the timeline and publishes them to ROS.
"""

PKG = 'rxbag'
import roslib; roslib.load_manifest(PKG)

import sys
import threading
import time

import wx

import rosbag
import rosgraph.masterapi
import rospy

class Player(object):
    def __init__(self, timeline):
        self.timeline = timeline

        self._publishing = set()
        self._publishers = {}
        
        # Attempt to connect to the ROS master
        if not wx.GetApp().connect_to_ros():
            raise Exception('Error connecting to ROS')

    def is_publishing(self, topic):
        return topic in self._publishing
    
    def start_publishing(self, topic):
        if topic in self._publishing:
            return
        
        self._publishing.add(topic)

        self.timeline.add_listener(topic, self)
    
    def stop_publishing(self, topic):
        if topic not in self._publishing:
            return
        
        self.timeline.remove_listener(topic, self)

        if topic in self._publishers:
            self._publishers[topic].unregister()
            del self._publishers[topic]

        self._publishing.remove(topic)

    def stop(self):
        for topic in list(self._publishing):
            self.stop_publishing(topic)

    ## 

    def message_viewed(self, bag, msg_data):
        # Don't publish unless the playhead is moving.
        if self.timeline.play_speed <= 0.0:
            return
        
        topic, msg, t = msg_data

        # Create publisher if this is the first message on the topic
        if topic not in self._publishers:
            try:
                self._publishers[topic] = rospy.Publisher(topic, type(msg))
            except Exception, ex:
                # Any errors, stop listening/publishing to this topic
                rospy.logerr('Error creating publisher on topic %s for type %s' % (topic, str(type(msg))))
                self.stop_publishing(topic)

        self._publishers[topic].publish(msg)

    def message_cleared(self):
        pass
