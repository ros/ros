#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

## talker that receives notification of new subscriptions

PKG = 'rospy_tutorials' # this package name
NAME = 'talker_callback'

import roslib; roslib.load_manifest(PKG) 

import sys

import rospy
from std_msgs.msg import String

class ChatterListener(rospy.SubscribeListener):
    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        print "a peer subscribed to topic [%s]"%topic_name
        
        str = "Hey everyone, we have a new friend!"
        print str
        topic_publish(String(str))
        str = "greetings. welcome to topic "+topic_name
        print str
        peer_publish(String(str))
        
    def peer_unsubscribe(self, topic_name, numPeers):
        print "a peer unsubscribed from topic [%s]"%topic_name
        if numPeers == 0:
            print "I have no friends"
    
def talker_callback():
    pub = rospy.Publisher("chatter", String, ChatterListener())
    rospy.init_node(NAME, anonymous=True)
    count = 0
    while not rospy.is_shutdown():
        str = "hello world %d"%count
        print str
        pub.publish(String(str))
        count += 1
        rospy.sleep(0.1)
        
if __name__ == '__main__':
    try:
        talker_callback()
    except rospy.ROSInterruptException: pass
