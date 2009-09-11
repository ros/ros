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

## Integration test for add_two_ints 

PKG = 'rospy_tutorials'
NAME = 'add_two_ints_test'

import roslib; roslib.load_manifest(PKG)

import sys
import unittest

import rospy
import rostest
from rospy_tutorials.srv import *

class TestAddTwoInts(unittest.TestCase):
        
    def test_add_two_ints(self):
        rospy.wait_for_service('add_two_ints')
        s = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        tests = [(1, 2), (0, 0), (-1, -2), (12312, 980123), (sys.maxint, -sys.maxint), (sys.maxint, -1), (sys.maxint, 0)]
        for x, y in tests:
            print "Requesting %s+%s"%(x, y)
            # test both simple and formal call syntax
            resp = s(x, y)
            resp2 = s.call(AddTwoIntsRequest(x, y))
            self.assertEquals(resp.sum,resp2.sum)
            print "%s+%s = %s"%(x, y, resp.sum)            
            self.assertEquals(resp.sum,(x + y), "integration failure, returned sum was %s vs. %s"%(resp.sum, (x+y)))

    def test_add_two_ints_bad_then_good(self):
        rospy.wait_for_service('add_two_ints')
        try:
            s = rospy.ServiceProxy('add_two_ints', BadTwoInts)            
            resp = s(1, 2)
            self.fail("service call should have failed with exception but instead returned 1+2=%s"%resp.sum)
        except rospy.ServiceException, e:
            print "success -- ros exception was thrown: %s"%e
        s = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp = s.call(AddTwoIntsRequest(1, 2))
        self.assertEquals(3,resp.sum)
        
            
    def test_add_two_ints_bad_type(self):
        rospy.wait_for_service('add_two_ints')
        s = rospy.ServiceProxy('add_two_ints', BadTwoInts)
        tests = [(1, 2), (0, 0), (-1, -2), (12312, 980123), (sys.maxint, -sys.maxint), (sys.maxint, -1), (sys.maxint, 0)]
        for x, y in tests:
            print "Requesting %s+%s"%(x, y)
            # test both simple and formal call syntax
            try:
                resp = s(x, y)
                if resp.sum == x+y:
                    self.fail("call 1 with bad type failed: the server appears to be incorrectly deserialing the packet as it returned: %s"%resp.sum)
                else:
                    self.fail("call 1 with bad type failed to throw exception: %s"%resp.sum)
            except rospy.ServiceException, e:
                print "success -- ros exception was thrown: %s"%e
            try:
                resp = s.call(BadTwoIntsRequest(x, y))
                if resp.sum == x+y:
                    self.fail("call 2 with bad type failed: the server appears to be incorrectly deserialing the packet as it returned: %s"%resp.sum)
                else:
                    self.fail("call 2 with bad type failed to throw exception: %s"%resp.sum)
            except rospy.ServiceException, e:
                print "success -- ros exception was thrown: %s"%e
        
if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestAddTwoInts, sys.argv)
