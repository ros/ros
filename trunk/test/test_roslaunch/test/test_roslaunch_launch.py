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
# Revision $Id: test_core.py 6875 2009-11-12 21:10:24Z kwc $

PKG = 'test_roslaunch'
NAME = 'test_roslaunch_launch'

import roslib; roslib.load_manifest(PKG)

import os
import sys
import unittest
    
## Test roslaunch.launch
class TestRoslaunchLaunch(unittest.TestCase):
        
    def test__unify_clear_params(self):
        from roslaunch.launch import _unify_clear_params
        self.assertEquals([], _unify_clear_params([]))
        for t in [['/foo'], ['/foo/'], ['/foo/', '/foo'],
                  ['/foo/', '/foo/'], ['/foo/', '/foo/bar', '/foo/'],
                  ['/foo/', '/foo/bar', '/foo/bar/baz']]:
            self.assertEquals(['/foo/'], _unify_clear_params(t))
        for t in [['/'], ['/', '/foo/'], ['/foo/', '/', '/baz', '/car/dog']]:
            self.assertEquals(['/'], _unify_clear_params(t))
            
        self.assertEquals(['/foo/', '/bar/', '/baz/'], _unify_clear_params(['/foo', '/bar', '/baz']))
        self.assertEquals(['/foo/', '/bar/', '/baz/'], _unify_clear_params(['/foo', '/bar', '/baz', '/bar/delta', '/baz/foo']))
        self.assertEquals(['/foo/bar/'], _unify_clear_params(['/foo/bar', '/foo/bar/baz']))
        
        
    def test__hostname_to_rosname(self):
        from roslaunch.launch import _hostname_to_rosname
        self.assertEquals("host_ann", _hostname_to_rosname('ann'))
        self.assertEquals("host_ann", _hostname_to_rosname('ANN'))
        self.assertEquals("host_", _hostname_to_rosname(''))
        self.assertEquals("host_1", _hostname_to_rosname('1'))
        self.assertEquals("host__", _hostname_to_rosname('_'))
        self.assertEquals("host__", _hostname_to_rosname('-'))
        self.assertEquals("host_foo_laptop", _hostname_to_rosname('foo-laptop'))

    def test_ROSLaunchListeners(self):
        import roslaunch.launch
        class L(roslaunch.launch.ROSLaunchListener):
            def process_died(self, process_name, exit_code):
                self.process_name = process_name
                self.exit_code = exit_code
        class LBad(roslaunch.launch.ROSLaunchListener):
            def process_died(self, process_name, exit_code):
                raise Exception("foo")

        listeners = roslaunch.launch._ROSLaunchListeners()
        l1 = L()
        l2 = L()        
        lbad = L()
        l3 = L()
        # test with no listeners
        listeners.process_died('p0', 0)
        # test with 1 listener
        listeners.add_process_listener(l1)
        listeners.process_died('p1', 1)
        self.assertEquals(l1.process_name, 'p1')
        self.assertEquals(l1.exit_code, 1)

        # test with 2 listeners        
        listeners.add_process_listener(l2)
        listeners.process_died('p2', 2)
        for l in [l1, l2]:
            self.assertEquals(l.process_name, 'p2')
            self.assertEquals(l.exit_code, 2)

        listeners.add_process_listener(lbad)
        # make sure that this catches errors
        listeners.process_died('p3', 3)
        for l in [l1, l2]:
            self.assertEquals(l.process_name, 'p3')
            self.assertEquals(l.exit_code, 3)
        # also add a third listener to make sure that listeners continues after lbad throws
        listeners.add_process_listener(l3)
        listeners.process_died('p4', 4)
        for l in [l1, l2, l3]:
            self.assertEquals(l.process_name, 'p4')
            self.assertEquals(l.exit_code, 4)            
        
if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_roslaunch', NAME, TestRoslaunchLaunch, coverage_packages=['roslaunch.launch'])
    
