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
# Revision $Id: test_empty_service.py 7395 2009-12-18 00:39:24Z kwc $

## Integration test for empty services to test serializers
## and transport

PKG = 'test_rospy'
NAME = 'empty_service'
import roslib; roslib.load_manifest(PKG)

import sys, time
import unittest

import rostest

class TestServiceFailure(unittest.TestCase):
        
    def test_persistent(self):
        # fail_two_ints succeeds unless the first argument is -1.
        # this makes sure, in the persistent case, that the proxy handle remains valid after a call
        from test_ros.srv import AddTwoInts
        import rospy
        rospy.wait_for_service('fail_two_ints', 10.)
        p = rospy.ServiceProxy('fail_two_ints', AddTwoInts, persistent=True)
        for a in [1, -1, 1, -1, -1, -1, -1, 1]:
            try:
                resp = p(a, 1)
                if a == 1:
                    self.assertEquals(resp.sum, 2)
                else:
                    self.fail("service call should have failed: %s,%s, %s"%(a, 1, resp.sum))
            except rospy.ServiceException, e:
                if a == -1:
                    # expected
                    pass
                else:
                    self.fail("service call failed when it shouldn't have: %s"%str(e))

    def test_non_persistent(self):
        # fail_two_ints succeeds unless the first argument is -1.
        # this makes sure, in the non-persistent case, that the proxy handle remains valid after a call
        from test_ros.srv import AddTwoInts
        import rospy
        rospy.wait_for_service('fail_two_ints', 10.)
        p = rospy.ServiceProxy('fail_two_ints', AddTwoInts)
        for a in [1, -1, 1, -1, -1, -1, -1, 1]:
            try:
                resp = p(a, 1)
                if a == 1:
                    self.assertEquals(resp.sum, 2)
                else:
                    self.fail("service call should have failed")
            except rospy.ServiceException, e:
                if a == -1:
                    # expected
                    pass
                else:
                    self.fail("service call should have failed")
        
        
if __name__ == '__main__':
    rostest.run(PKG, 'rospy_service_failure', TestServiceFailure, sys.argv)
