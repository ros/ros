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

from __future__ import with_statement

NAME = 'test_rosmsg_command_line'
import roslib; roslib.load_manifest('test_rosmsg')

import os
import sys 
import unittest
import cStringIO
import time
        
import rostest

from subprocess import Popen, PIPE, check_call, call

class TestRosmsg(unittest.TestCase):

    def setUp(self):
        pass

    ## test that the rosmsg command works
    def test_cmd_help(self):
        for cmd in ['rosmsg', 'rossrv']:
            output = Popen([cmd], stdout=PIPE).communicate()[0]
            self.assert_('Commands' in output)
            output = Popen([cmd, '-h'], stdout=PIPE).communicate()[0]
            self.assert_('Commands' in output)

            for c in ['show', 'md5', 'package', 'packages', 'users']:
                output = Popen([cmd, c, '-h'], stdout=PIPE).communicate()[0]
                self.assert_('Usage' in output)
            
    def test_cmd_packages(self):
        output1 = Popen(['rosmsg', 'packages'], stdout=PIPE).communicate()[0]
        output2 = Popen(['rosmsg', 'packages', '-p'], stdout=PIPE).communicate()[0]
        l1 = [x for x in output1.split() if x]
        l2 = [x.strip() for x in output2.split('\n') if x.strip()]
        self.assertEquals(l1, l2)
        for p in ['roslib', 'test_rosmsg', 'std_msgs', 'rospy_tutorials']:
            self.assert_(p in l1)
        for p in ['std_srvs', 'rospy']:
            self.assert_(p not in l1)

        output1 = Popen(['rossrv', 'packages'], stdout=PIPE).communicate()[0]
        output2 = Popen(['rossrv', 'packages', '-p'], stdout=PIPE).communicate()[0]
        l1 = [x for x in output1.split() if x]
        l2 = [x.strip() for x in output2.split('\n') if x.strip()]
        self.assertEquals(l1, l2)
        for p in ['test_rosmsg', 'std_srvs', 'rospy_tutorials']:
            self.assert_(p in l1)
        for p in ['std_msgs', 'rospy']:
            self.assert_(p not in l1)
        
    def test_cmd_package(self):
        # this test is obviously very brittle, but should stabilize as the tests stabilize
        output1 = Popen(['rosmsg', 'package', 'test_rosmsg'], stdout=PIPE).communicate()[0]
        #  - validate pretty print as well
        output2 = Popen(['rosmsg', 'package', '-p', 'test_rosmsg'], stdout=PIPE).communicate()[0]
        l = set([x for x in output1.split() if x])        
        l2 = set([x.strip() for x in output2.split('\n') if x.strip()])
        self.assertEquals(l, l2)
        
        self.assertEquals(set(['test_rosmsg/RosmsgA',
                               'test_rosmsg/RosmsgB',
                               'test_rosmsg/RosmsgC',
                               ]), l)
        
        output = Popen(['rossrv', 'package', 'test_rosmsg'], stdout=PIPE).communicate()[0]
        output2 = Popen(['rossrv', 'package', '-p', 'test_rosmsg'], stdout=PIPE).communicate()[0]        
        l = set([x for x in output.split() if x])
        l2 = set([x.strip() for x in output2.split('\n') if x.strip()])
        self.assertEquals(l, l2)
        
        self.assertEquals(set(['test_rosmsg/RossrvA', 'test_rosmsg/RossrvB']), l)
        
    ## test that the rosmsg/rossrv show command works
    def test_cmd_show(self):
        output = Popen(['rosmsg', 'show', 'std_msgs/String'], stdout=PIPE).communicate()[0]
        self.assertEquals('string data', output.strip())

        output = Popen(['rossrv', 'show', 'std_srvs/Empty'], stdout=PIPE).communicate()[0]
        self.assertEquals('---', output.strip())
        output = Popen(['rossrv', 'show', 'std_srvs/Empty'], stdout=PIPE).communicate()[0]
        self.assertEquals('---', output.strip())
        output = Popen(['rossrv', 'show', 'rospy_tutorials/AddTwoInts'], stdout=PIPE).communicate()[0]
        self.assertEquals('int64 a\nint64 b\n---\nint64 sum', output.strip())

        # test against test_rosmsg package
        d = roslib.packages.get_pkg_dir('test_rosmsg')
        msg_d = os.path.join(d, 'msg')
        # - test with non-recursive types, which should have identical raw/non-raw
        for t in ['RosmsgA', 'RosmsgB']:
            with open(os.path.join(msg_d, '%s.msg'%t), 'r') as f:
                text = f.read()
            text = text+'\n' # running command adds one new line
            type_ ='test_rosmsg/'+t
            output = Popen(['rosmsg', 'show', type_], stdout=PIPE).communicate()[0]
            self.assertEquals(text, output)
            output = Popen(['rosmsg', 'show', '-r',type_], stdout=PIPE, stderr=PIPE).communicate()
            self.assertEquals(text, output[0], "Failed: %s"%(str(output)))
            output = Popen(['rosmsg', 'show', '--raw', type_], stdout=PIPE).communicate()[0]
            self.assertEquals(text, output)

            # test as search
            type_ = t
            text = "[test_rosmsg/%s]:\n%s"%(t, text)
            output = Popen(['rosmsg', 'show', type_], stdout=PIPE).communicate()[0]
            self.assertEquals(text, output)
            output = Popen(['rosmsg', 'show', '-r',type_], stdout=PIPE, stderr=PIPE).communicate()
            self.assertEquals(text, output[0], "Failed: %s"%(str(output)))
            output = Popen(['rosmsg', 'show', '--raw', type_], stdout=PIPE).communicate()[0]
            self.assertEquals(text, output)
        
if __name__ == '__main__':
    rostest.unitrun('test_rosmsg', NAME, TestRosmsg, sys.argv, coverage_packages=[])
