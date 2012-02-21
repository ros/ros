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

import os
import sys
import struct
import unittest
import time
import random

import rosunit

import rospy


class TestRospyClient(unittest.TestCase):
    
    def test_init_node(self):
        failed = True
        try:
            # #1822
            rospy.init_node('ns/node')
        except ValueError:
            failed = False
        self.failIf(failed, "init_node allowed '/' in name")

    def test_spin(self):
        failed = True
        try:
            rospy.spin()
        except rospy.ROSInitException:
            failed = False
        self.failIf(failed, "spin() should failed if not initialized")
        
    def test_myargv(self):
        orig_argv = sys.argv
        try:
            from rospy.client import myargv
            args = myargv()
            self.assertEquals(args, sys.argv)
            self.assertEquals(['foo', 'bar', 'baz'], myargv(['foo','bar', 'baz']))
            self.assertEquals(['-foo', 'bar', '-baz'], myargv(['-foo','bar', '-baz']))
            
            self.assertEquals(['foo'], myargv(['foo','bar:=baz']))
            self.assertEquals(['foo'], myargv(['foo','-bar:=baz']))
        finally:
            sys.argv = orig_argv
    
    def test_load_command_line_node_params(self):
        
        from rospy.client import load_command_line_node_params
        
        assert {} == load_command_line_node_params([])
        assert {} == load_command_line_node_params(['a', 'b', 'c'])        
        assert {} == load_command_line_node_params(['a:=b'])        
        assert {'a': 'b'} == load_command_line_node_params(['_a:=b'])        
        assert {'a': 'b', 'foo': 'bar'} == load_command_line_node_params(['_a:=b', 'blah', '_foo:=bar', 'baz'])        
        # test yaml unmarshal
        assert {'a': 1} == load_command_line_node_params(['_a:=1'])        
        try:
            load_command_line_node_params(['_a:=b:=c'])        
        except rospy.exceptions.ROSInitException:
            pass

if __name__ == '__main__':
    rosunit.unitrun('test_rospy', sys.argv[0], TestRospyClient, coverage_packages=['rospy.client'])
