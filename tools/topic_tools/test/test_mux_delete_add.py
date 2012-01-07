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
# Author: Brian Gerkey

PKG = 'topic_tools'

import rospy

from topic_tools.srv import MuxAdd
from topic_tools.srv import MuxDelete
from topic_tools.srv import MuxList
from topic_tools.srv import MuxSelect
from std_msgs.msg import String

def go():
    rospy.init_node('chatter')

    rospy.wait_for_service('mux/add', 5)
    rospy.wait_for_service('mux/delete', 5)
    rospy.wait_for_service('mux/list', 5)
    rospy.wait_for_service('mux/select', 5)

    add_srv = rospy.ServiceProxy('mux/add', MuxAdd)
    delete_srv = rospy.ServiceProxy('mux/delete', MuxDelete)
    list_srv = rospy.ServiceProxy('mux/list', MuxList)
    select_srv = rospy.ServiceProxy('mux/select', MuxSelect)

    b_pub = rospy.Publisher('b', String)

    # Execute the sequence given in #2863
    select_srv('c')
    delete_srv('b')
    add_srv('b')
    select_srv('b')

    # Now start publishing on b
    while not rospy.is_shutdown():
        b_pub.publish('foo')
	rospy.sleep(0.2)

if __name__ == "__main__":
    go()
