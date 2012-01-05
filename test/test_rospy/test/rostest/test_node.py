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

## Simple talker demo that publishes std_msg/Strings to the 'chatter' topic

PKG = 'test_rospy'
NAME = 'test_node'
import roslib; roslib.load_manifest(PKG)

import sys

import rospy
import test_ros.msg

# Copied from test_ros.test_node
#_required_publications  = 'test_string_out', 'test_primitives_out', 'test_arrays_out', 'test_header_out'
#_required_subscriptions = 'test_string_in', 'test_primitives_in', 'test_arrays_in', 'test_header_in', 'probe_topic'

## pass-through callback that republishes message on \a pub.
## @param pub TopicPub: topic to republish incoming messages on
## @return fn: callback fn for TopicSub
def chain_callback(pub):
    def chained_callback(data):
        # special logic for handling TestHeader
        if isinstance(data, test_ros.msg.TestHeader):
            if data.auto == 0:
                new_data = test_ros.msg.TestHeader()
                # when auto is 0, must send 1234, 5678 as time
                new_data.header.stamp = Time(1234, 5678)
                # frame_id not really important
                new_data.header.frame_id = 1234
            else: # force auto-header timestamp
                new_data = test_ros.msg.TestHeader(None, rospy.caller_id(), data.auto)
            data = new_data
        data.caller_id = rospy.caller_id()
        pub.publish(data)

def test_node():
    # required publications
    string_out = rospy.Publisher("test_string_out", test_ros.msg.TestString)
    primitives_out = rospy.Publisher("test_primitives_out", test_ros.msg.TestPrimitives)
    arrays_out = rospy.Publisher("test_arrays_out", test_ros.msg.TestArrays)
    header_out = rospy.Publisher("test_header_out", test_ros.msg.TestHeader)

    #required subs
    rospy.Subscriber("test_string_in", test_ros.msg.TestString, chain_callback(string_out))
    rospy.Subscriber("test_primitives_in", test_ros.msg.TestPrimitives, chain_callback(primitives_out))
    rospy.Subscriber("test_arrays_in", test_ros.msg.TestArrays, chain_callback(arrays_out))
    rospy.Subscriber("test_header_in", test_ros.msg.TestHeader, chain_callback(header_out))
     
    # subscription with no publisher
    probe_in = rospy.Subscriber("probe_topic", test_ros.msg.TestString)
    
    rospy.init_node(NAME)
    rospy.spin()
        
if __name__ == '__main__':
    test_node()

        
