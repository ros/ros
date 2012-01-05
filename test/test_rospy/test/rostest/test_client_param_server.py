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

## Unit test of rospy.client APIs for parameter server access

PKG = 'test_rospy'
NAME = 'test_rospy_client_param_server'

import sys, time
import math
import unittest

import rospy
import rostest
from test_rospy.srv import *

def test_param_filter(params):
    return [p for p in params if not p.startswith('/roslaunch') and p not in ['/run_id', '/rosdistro', '/rosversion']]

## Test the rospy.client parameter server API: 
##- get_param
##- set_param
##- delete_param
##- has_param
##- get_param_names
class TestClientParamServer(unittest.TestCase):
    
    ## test get/has param against state initialized from rostest script
    def test_get_has_param(self):
        #test error conditions
        try: rospy.get_param(None); self.fail("get_param(None) succeeded")
        except: pass
        try: rospy.get_param(''); self.fail("get_param('') succeeded")
        except: pass
        # should be keyerror
        try: rospy.get_param('non-existent'); self.fail("get_param('non-existent') succeeded")
        except: pass
        try: rospy.has_param(None, 'foo'); self.fail("has_param(None) succeeded")
        except: pass
        try: rospy.has_param(''); self.fail("has_param('') succeeded")
        except: pass

        self.failIf(rospy.has_param('non-existent'), "has_param('non-existent') succeeded")
            
        #validate get_param against values on the param server
        rostest_tests = {
            'string': "string",
            'int0': 0,
            'int10': 10,
            'float0': 0.0,
            'float10': 10.0,
            "namespace/string": "namespaced string",
            }
        param_names = [rospy.resolve_name(k) for k in rostest_tests.keys()]
        
        # test get parameter names
        diff = set(param_names) ^ set(test_param_filter(rospy.get_param_names()))
        self.failIf(diff, diff)
        
        # test for existing and value
        for k, v in rostest_tests.iteritems():
            self.assert_(rospy.has_param(k))
            self.assert_(rospy.has_param(rospy.resolve_name(k)))            
            if not type(v) == float:
                self.assertEqual(v, rospy.get_param(k))
                self.assertEqual(v, rospy.get_param(rospy.resolve_name(k)))                
            else:
                self.assertAlmostEqual(v, rospy.get_param(k), 1)
                self.assertAlmostEqual(v, rospy.get_param(rospy.resolve_name(k)), 1)                
                
    def test_search_param(self):
        try:
            orig_caller_id = rospy.names.get_caller_id()

            rospy.names._set_caller_id('/global_node')
            
            self.assertEquals(None, rospy.search_param('search_param'))
            rospy.set_param('/search_param', 1)
            self.assertEquals('/search_param', rospy.search_param('search_param'))

            rospy.names._set_caller_id('/level1/level2/relative_node')
            self.assertEquals('/search_param', rospy.search_param('search_param'))
            rospy.set_param('/level1/search_param', 2)
            self.assertEquals('/level1/search_param', rospy.search_param('search_param'))
            rospy.set_param('~search_param', 3)
            # make sure that search starts in our private namespace first
            self.assertEquals('/level1/level2/relative_node/search_param', rospy.search_param('search_param')) 
            
        finally:
            rospy.names._set_caller_id(orig_caller_id)

    def test_delete_param(self):
        try: rospy.delete_param(None); self.fail("delete_param(None) succeeded")
        except: pass
        try: rospy.delete_param(''); self.fail("delete_param('') succeeded")
        except: pass

        rostest_tests = {
            'dpstring': "string",
            'dpint0': 0,
            'dpint10': 10,
            'dpfloat0': 0.0,
            'dpfloat10': 10.0,
            "dpnamespace/string": "namespaced string",
            }
        initial_params = rospy.get_param_names()
        # implicitly depends on set param
        for k, v in rostest_tests.iteritems():
            rospy.set_param(k, v)
        
        # test delete
        for k, v in rostest_tests.iteritems():
            self.assert_(rospy.has_param(k))
            rospy.delete_param(k)
            self.failIf(rospy.has_param(k))
            self.failIf(rospy.has_param(rospy.resolve_name(k)))
            try:
                rospy.get_param(k)
                self.fail("get_param should fail on deleted key")
            except KeyError: pass
            
        # make sure no new state
        self.failIf(set(initial_params) ^ set(rospy.get_param_names()))
        
    ## test set_param separately
    def test_set_param(self):
        try: rospy.set_param(None, 'foo'); self.fail("set_param(None) succeeded")
        except: pass
        try: rospy.set_param('', 'foo'); self.fail("set_param('') succeeded")
        except: pass

        rostest_tests = {
            'spstring': "string",
            'spint0': 0,
            'spint10': 10,
            'spfloat0': 0.0,
            'spfloat10': 10.0,
            "spnamespace/string": "namespaced string",
            }
        initial_param_names = rospy.get_param_names()
        param_names = [rospy.resolve_name(k) for k in rostest_tests.keys()]
        for k, v in rostest_tests.iteritems():
            self.failIf(rospy.has_param(k))
            self.failIf(rospy.has_param(rospy.resolve_name(k)))
            rospy.set_param(k, v)
            self.assert_(rospy.has_param(k))
            self.assert_(rospy.has_param(rospy.resolve_name(k)))
            self.assertEquals(v, rospy.get_param(k))            
            self.assertEquals(v, rospy.get_param(rospy.resolve_name(k)))
        correct_state = set(initial_param_names + param_names)
        self.failIf(correct_state ^ set(rospy.get_param_names()))
        
if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.run(PKG, NAME, TestClientParamServer, sys.argv)
