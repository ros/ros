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

## Simple listener demo that demonstrates how to register additional
## arguments to be passed to a subscription callback

PKG = 'rospy_tutorials' # this package name
NAME = 'listener_with_user_data'

import roslib; roslib.load_manifest(PKG) 

import sys

import rospy
from std_msgs.msg import *

def callback(data, args):
    if args == 1:
        print "#1: I heard [%s]"%data.data
    elif args == 2:
        print "#2: I heard [%s]"%data.data
    else:
        print "I heard [%s] with userdata [%s]"%(data.data, str(args)) 
    
def listener_with_user_data():
    # Callback arguments (aka user data) allow you to reuse the same
    # callback for different topics, or they can even allow you to use
    # the same callback for the same topic, but have it do something
    # different based on the arguments.
    rospy.Subscriber("chatter", String, callback, 1)
    rospy.Subscriber("chatter", String, callback, 2)
    rospy.Subscriber("chatter", String, callback, ("chatter1", 4))    
    rospy.Subscriber("chatter2", String, callback, "This is from chatter2")
    rospy.init_node(NAME, anonymous=True)
    rospy.spin()
        
if __name__ == '__main__':
    try:
        listener_with_user_data()
    except KeyboardInterrupt, e:
        pass
    print "exiting"
