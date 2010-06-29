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
import roslib; roslib.load_manifest(PKG)

import unittest
import rospy

from topic_tools.srv import MuxAdd
from topic_tools.srv import MuxDelete
from topic_tools.srv import MuxList

class MuxServiceTestCase(unittest.TestCase):
    def make_srv_proxies(self):
        try:
            rospy.wait_for_service('mux/add', 5)
            rospy.wait_for_service('mux/delete', 5)
            rospy.wait_for_service('mux/list', 5)
        except rospy.ROSException, e:
            self.fail('failed to find a required service: ' + `e`)

        add_srv = rospy.ServiceProxy('mux/add', MuxAdd)
        delete_srv = rospy.ServiceProxy('mux/delete', MuxDelete)
        list_srv = rospy.ServiceProxy('mux/list', MuxList)

        return (add_srv, delete_srv, list_srv)
            
    def test_add_delete_list(self):
        add_srv, delete_srv, list_srv = self.make_srv_proxies()
        topics = list_srv().topics
        self.assertEquals(set(topics), set(['input']))
        add_srv('new_input')
        topics = list_srv().topics
        self.assertEquals(set(topics), set(['input', 'new_input']))
        delete_srv('input')
        topics = list_srv().topics
        self.assertEquals(set(topics), set(['new_input']))
        delete_srv('new_input')
        topics = list_srv().topics
        self.assertEquals(set(topics), set([]))

if __name__ == "__main__":
    import rostest
    rostest.unitrun(PKG, 'mux_services', MuxServiceTestCase)

