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
# Revision $Id: talker.py 4223 2009-04-16 21:47:54Z jfaustwg $

## talker demo that published std_msgs/ColorRGBA messages
## to the 'color' topic. To see these messages, type:
##   rostopic echo color
## this demo shows some of the more advanced APIs in rospy.

import roslib; roslib.load_manifest('rospy_tutorials')

import rospy
from std_msgs.msg import ColorRGBA

def talker():
    topic = 'color'
    pub = rospy.Publisher(topic, ColorRGBA)
    rospy.init_node('color_talker', anonymous=True)
    print "\n\nNode running. To see messages, please type\n\t'rostopic echo %s'\nIn another window\n\n"%(rospy.resolve_name(topic))
    while not rospy.is_shutdown():

        # publish with in-order initialization of arguments (r, g, b, a)
        pub.publish(1, 2, 3, 4)
        rospy.sleep(.5)

        # publish with a=1, use default values for rest
        pub.publish(a=1.0)
        rospy.sleep(.5)

        # print the number of subscribers
        rospy.loginfo("I have %s subscribers"%pub.get_num_connections())
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
