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
NAME = 'test_rosmsg'
import roslib; roslib.load_manifest('test_rosmsg')

import os
import sys 
import unittest
import cStringIO
import time
        
import rostest

import roslib.packages
import rosmsg

from subprocess import Popen, PIPE, check_call, call

class TestRosmsg(unittest.TestCase):

    def setUp(self):
        pass

    def test_fullusage(self):
        text = rosmsg.fullusage('rosmsg')
        self.assert_("Commands" in text)
        cmds = ['show', 'users', 'md5', 'package', 'packages']
        for c in cmds:
            self.assert_(c in text)

    def test_get_msg_text(self):
        d = roslib.packages.get_pkg_dir('test_rosmsg')
        msg_d = os.path.join(d, 'msg')
        for t in ['RosmsgA', 'RosmsgB']:
            with open(os.path.join(msg_d, '%s.msg'%t), 'r') as f:
                text = f.read()
            type_ = 'test_rosmsg/'+t
            self.assertEquals(text, rosmsg.get_msg_text(type_, raw=False))
            self.assertEquals(text, rosmsg.get_msg_text(type_, raw=True))
            
        # test recursive types
        t = 'RosmsgC'
        with open(os.path.join(msg_d, '%s.msg'%t), 'r') as f:
            text = f.read()
        type_ = 'test_rosmsg/'+t
        self.assertEquals(text, rosmsg.get_msg_text(type_, raw=True))
        self.assertEquals("""std_msgs/String s1
  string data
std_msgs/String s2
  string data""", rosmsg.get_msg_text(type_, raw=False).strip())

    def test_rosmsg_list_packages(self):
        try:
            l = rosmsg.rosmsg_list_packages('.foo')
            self.fail("should have failed on invalid mode")
        except ValueError: pass

        # test msgs
        l = rosmsg.rosmsg_list_packages('.msg')
        for p in ['roslib', 'test_rosmsg', 'test_ros', 'std_msgs']:
            self.assert_(p in l, "%s not in %s"%(p, l))
        for p in ['rospy', 'std_srvs']:
            self.assert_(p not in l)

        # test srvs
        l = rosmsg.rosmsg_list_packages('.srv')
        for p in ['test_rosmsg', 'test_ros', 'std_srvs']:
            self.assert_(p in l, "%s not in %s"%(p, l))
        for p in ['roslib', 'rospy', 'std_msgs']:
            self.assert_(p not in l)
        
    def test_rosmsg_list_package(self):
        try:
            l = rosmsg.rosmsg_list_package('.foo', 'test_rosmsg')
            self.fail("should have failed on invalid mode")
        except ValueError: pass

        # test msgs
        l = rosmsg.rosmsg_list_package('.msg', 'rospy')
        self.assertEquals([], l)
        l = rosmsg.rosmsg_list_package('.msg', 'test_rosmsg')
        self.assertEquals(set(['test_rosmsg/RosmsgA',
                               'test_rosmsg/RosmsgB',
                               'test_rosmsg/RosmsgC',
                               ]), set(l))
        
        l = rosmsg.rosmsg_list_package('.srv', 'rospy')
        self.assertEquals([], l)        
        l = rosmsg.rosmsg_list_package('.srv', 'test_rosmsg')
        self.assertEquals(set(['test_rosmsg/RossrvA',
                               'test_rosmsg/RossrvB',
                               ]), set(l))

    def test_get_srv_text(self):
        d = roslib.packages.get_pkg_dir('test_rosmsg')
        srv_d = os.path.join(d, 'srv')
        with open(os.path.join(srv_d, 'RossrvA.srv'), 'r') as f:
            text = f.read()
        self.assertEquals(text, rosmsg.get_srv_text('test_rosmsg/RossrvA', raw=False))
        self.assertEquals(text, rosmsg.get_srv_text('test_rosmsg/RossrvA', raw=True))

        # std_msgs/empty / std_msgs/empty
        with open(os.path.join(srv_d, 'RossrvB.srv'), 'r') as f:
            text = f.read()
        self.assertEquals(text, rosmsg.get_srv_text('test_rosmsg/RossrvB', raw=False))
        self.assertEquals(text, rosmsg.get_srv_text('test_rosmsg/RossrvB', raw=True))
        

        
if __name__ == '__main__':
    rostest.unitrun('test_rosmsg', NAME, TestRosmsg, sys.argv, coverage_packages=['rosmsg'])
