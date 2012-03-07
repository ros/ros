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
import unittest
import cStringIO
import time

import rospkg
        
from subprocess import Popen, PIPE, check_call, call

_SCRIPT_FOLDER = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts'))

class TestRosmsg(unittest.TestCase):

    def setUp(self):
        self.new_environ = os.environ
        self.new_environ["PYTHONPATH"] = os.path.join(os.getcwd(), "src")+':'+os.environ['PYTHONPATH']

    ## test that the rosmsg command works
    def test_cmd_help(self):
        sub = ['show', 'md5', 'package', 'packages', 'list']
        
        for cmd in ['rosmsg', 'rossrv']:
            glob_cmd=[os.path.join(_SCRIPT_FOLDER, cmd)]
            output = Popen(glob_cmd, stdout=PIPE, env=self.new_environ).communicate()[0]
            self.assert_('Commands' in output)
            output = Popen(glob_cmd+['-h'], stdout=PIPE, env=self.new_environ).communicate()[0]
            self.assert_('Commands' in output)
            self.assert_('Traceback' not in output)
            for c in sub:
                self.assert_("%s %s"%(cmd, c) in output, "%s %s"%(cmd, c) + " not in "+ output + " of " + str(glob_cmd))
                
            for c in sub:
                output = Popen(glob_cmd + [c, '-h'], stdout=PIPE, env=self.new_environ).communicate()[0]
                self.assert_('Usage' in output)
                self.assert_("%s %s"%(cmd, c) in output, output)
            
    def test_cmd_packages(self):
        # - single line
        output1 = Popen(['rosmsg', 'packages', '-s'], stdout=PIPE).communicate()[0]
        # - multi-line
        output2 = Popen(['rosmsg', 'packages'], stdout=PIPE).communicate()[0]
        l1 = [x for x in output1.split() if x]
        l2 = [x.strip() for x in output2.split('\n') if x.strip()]
        self.assertEquals(l1, l2)
        for p in ['std_msgs', 'test_ros']:
            self.assert_(p in l1)
        for p in ['std_srvs', 'rosmsg']:
            self.assert_(p not in l1)

        output1 = Popen(['rossrv', 'packages', '-s'], stdout=PIPE).communicate()[0]
        output2 = Popen(['rossrv', 'packages'], stdout=PIPE).communicate()[0]
        l1 = [x for x in output1.split() if x]
        l2 = [x.strip() for x in output2.split('\n') if x.strip()]
        self.assertEquals(l1, l2)
        for p in ['std_srvs', 'test_ros']:
            self.assert_(p in l1)
        for p in ['std_msgs', 'rospy']:
            self.assert_(p not in l1)

    def test_cmd_list(self):
        # - multi-line
        output1 = Popen([os.path.join(_SCRIPT_FOLDER,'rosmsg'), 'list'], stdout=PIPE).communicate()[0]
        l1 = [x.strip() for x in output1.split('\n') if x.strip()]
        for p in ['std_msgs/String', 'test_ros/Floats']:
            self.assert_(p in l1)
        for p in ['std_srvs/Empty', 'roscpp/Empty']:
            self.assert_(p not in l1)

        output1 = Popen([os.path.join(_SCRIPT_FOLDER,'rossrv'), 'list'], stdout=PIPE).communicate()[0]
        l1 = [x.strip() for x in output1.split('\n') if x.strip()]
        for p in ['std_srvs/Empty', 'roscpp/Empty']:
            self.assert_(p in l1)
        for p in ['std_msgs/String', 'test_ros/Floats']:
            self.assert_(p not in l1)
        
    def test_cmd_package(self):
        # this test is obviously very brittle, but should stabilize as the tests stabilize
        # - single line output
        output1 = Popen(['rosmsg', 'package', '-s', 'test_ros'], stdout=PIPE).communicate()[0]
        # - multi-line output
        output2 = Popen(['rosmsg', 'package', 'test_ros'], stdout=PIPE).communicate()[0]
        l = set([x for x in output1.split() if x])        
        l2 = set([x.strip() for x in output2.split('\n') if x.strip()])
        self.assertEquals(l, l2)
        
        for m in ['test_ros/RosmsgA',
                  'test_ros/RosmsgB',
                  'test_ros/RosmsgC']:
            self.assertTrue(m in l, l)
        
        output = Popen(['rossrv', 'package', '-s', 'test_ros'], stdout=PIPE).communicate()[0]
        output2 = Popen(['rossrv', 'package','test_ros'], stdout=PIPE).communicate()[0]        
        l = set([x for x in output.split() if x])
        l2 = set([x.strip() for x in output2.split('\n') if x.strip()])
        self.assertEquals(l, l2)
        
        for m in ['test_ros/RossrvA', 'test_ros/RossrvB']:
            self.assertTrue(m in l, l)
        
    ## test that the rosmsg/rossrv show command works
    def test_cmd_show(self):
        output = Popen(['rosmsg', 'show', 'std_msgs/String'], stdout=PIPE).communicate()[0]
        self.assertEquals('string data', output.strip())

        output = Popen(['rossrv', 'show', 'std_srvs/Empty'], stdout=PIPE).communicate()[0]
        self.assertEquals('---', output.strip())
        output = Popen(['rossrv', 'show', 'std_srvs/Empty'], stdout=PIPE).communicate()[0]
        self.assertEquals('---', output.strip())
        output = Popen(['rossrv', 'show', 'test_ros/AddTwoInts'], stdout=PIPE).communicate()[0]
        self.assertEquals('int64 a\nint64 b\n---\nint64 sum', output.strip())

        # test against test_rosmsg package
        rospack = rospkg.RosPack()
        d = rospack.get_path('test_ros')
        msg_d = os.path.join(d, 'msg')
        # - test with non-recursive types, which should have identical raw/non-raw
        for t in ['RosmsgA', 'RosmsgB']:
            with open(os.path.join(msg_d, '%s.msg'%t), 'r') as f:
                text = f.read()
            text = text+'\n' # running command adds one new line
            type_ ='test_ros/'+t
            output = Popen(['rosmsg', 'show', type_], stdout=PIPE).communicate()[0]
            self.assertEquals(text, output)
            output = Popen(['rosmsg', 'show', '-r',type_], stdout=PIPE, stderr=PIPE).communicate()
            self.assertEquals(text, output[0], "Failed: %s"%(str(output)))
            output = Popen(['rosmsg', 'show', '--raw', type_], stdout=PIPE).communicate()[0]
            self.assertEquals(text, output)

            # test as search
            type_ = t
            text = "[test_ros/%s]:\n%s"%(t, text)
            output = Popen(['rosmsg', 'show', type_], stdout=PIPE).communicate()[0]
            self.assertEquals(text, output)
            output = Popen(['rosmsg', 'show', '-r',type_], stdout=PIPE, stderr=PIPE).communicate()
            self.assertEquals(text, output[0], "Failed: %s"%(str(output)))
            output = Popen(['rosmsg', 'show', '--raw', type_], stdout=PIPE).communicate()[0]
            self.assertEquals(text, output)
