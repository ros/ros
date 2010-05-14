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

import roslib; roslib.load_manifest('test_rospy')

import os
import sys
import struct
import unittest
import time

class TestRospyMsnode(unittest.TestCase):

    def test_ROSNode(self):
        # mostly a trip wire test
        import rospy.msnode
        ROSNode = rospy.msnode.ROSNode
        node = ROSNode('/foo')
        self.assertEquals('foo', node.name)
        self.assertEquals(node.port, 0)
        node.start()
        # allow 2 seconds for xmlrpc server to start
        timeout_t = time.time() + 2.0
        while time.time() < timeout_t and node.uri is None:
            time.sleep(0.1)
        self.assert_(node.uri.startswith('http://'))
        self.assert_(node.port > 0)
        self.assertEquals(None, node.handler)
        node.shutdown('test complete')

    
if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_rospy', sys.argv[0], TestRospyMsnode, coverage_packages=['rospy.msnode'])
