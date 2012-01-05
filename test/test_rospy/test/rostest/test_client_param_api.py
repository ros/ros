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

## Integration test for empty services to test serializers
## and transport

PKG = 'test_rospy'
NAME = 'test_param_api'

import sys
import time
import unittest

import rospy
import rostest

class TestClientParamApi(unittest.TestCase):

    def test_param_api(self):
        # test get_param_names
        param_names = rospy.get_param_names()
        for n in ['/param1', 'param1', '~param1', 'param_int', 'param_float']:
            self.assert_(rospy.resolve_name(n) in param_names)
        
        # test has_param
        self.assert_(rospy.has_param('/run_id'))
        self.assert_(rospy.has_param('/param1'))
        self.assert_(rospy.has_param('param1'))
        self.assert_(rospy.has_param('~param1'))

        # test search_param
        self.assertEquals(None, rospy.search_param('not_param1'))
        self.assertEquals(rospy.resolve_name('~param1'), rospy.search_param('param1'))
        self.assertEquals(rospy.resolve_name('parent_param'), rospy.search_param('parent_param'))
        self.assertEquals("/global_param", rospy.search_param('global_param'))                
        
        # test get_param on params set in rostest file
        self.assertEquals('global_param1', rospy.get_param('/param1'))
        self.assertEquals('parent_param1', rospy.get_param('param1'))
        self.assertEquals('private_param1', rospy.get_param('~param1'))
        # - type tests
        self.assertEquals(1, rospy.get_param('param_int'))
        self.assertAlmostEquals(2., rospy.get_param('param_float'))        
        self.assertEquals(True, rospy.get_param('param_bool'))
        self.assertEquals("hello world", rospy.get_param('param_str'))
        
        # test default behavior get_param 
        try:
            rospy.get_param('not_param1')
            self.fail("should have raised KeyError")
        except KeyError: pass
        self.assertEquals('parent_param1', rospy.get_param('param1', 'foo'))
        self.assertEquals('private_param1', rospy.get_param('~param1', 'foo'))
        self.assertEquals('myval', rospy.get_param('not_param1', 'myval'))
        self.assertEquals('myval', rospy.get_param('~not_param1', 'myval'))
        self.assertEquals(None, rospy.get_param('not_param1', None))
        self.assertEquals(None, rospy.get_param('~not_param1', None))

        # test set/get roundtrips
        vals = [ '1', 1, 1., [1, 2, 3, 4], {'a': 1, 'b': 2}]
        for v in vals:
            self.failIf(rospy.has_param("cp_param"))
            try:
                rospy.get_param('cp_param1')
                self.fail("should have thrown KeyError")
            except KeyError: pass
            self.assertEquals(None, rospy.get_param('cp_param', None))
            self.assertEquals("default", rospy.get_param('cp_param', "default"))
            rospy.set_param("cp_param", v)
            self.assert_(rospy.has_param("cp_param"))
            self.assertEquals(v, rospy.get_param("cp_param"))
            self.assertEquals(rospy.resolve_name('cp_param'), rospy.search_param('cp_param'))
            # erase the param and recheck state
            rospy.delete_param('cp_param')
            self.failIf(rospy.has_param("cp_param"))
            self.assertEquals(None, rospy.get_param('cp_param', None))
            self.assertEquals("default", rospy.get_param('cp_param', "default"))
            self.assertEquals(None, rospy.search_param('cp_param'))
            
if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.run(PKG, NAME, TestClientParamApi, sys.argv)
