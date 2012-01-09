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

## Utility node for testing. Listens to chatter and when it gets a
## message it starts rebroadcasting on 'listenerpublisher'.  Unlike
## the normal listener, listenerpublisher is NOT anonymous.

import sys

import rospy
from test_rospy.msg import EmbedTest

_publishing = False
_pub = None
def start_publishing():
    global _pub
    if _pub is not None:
        return
    print "registering onto listenerpublisher"
    _pub = rospy.Publisher("listenerpublisher", EmbedTest)
    
def callback(data):
    print rospy.get_caller_id(), "I heard %s, %s, %s"%(data.str1.data, data.int1.data, data.val.val)
    start_publishing()
    print "re-publishing"
    _pub.publish(data)
    
def listener():
    rospy.init_node("listenerpublisher_embed")
    rospy.Subscriber("chatter", EmbedTest, callback)
    rospy.spin()
        
if __name__ == '__main__':
    listener()
