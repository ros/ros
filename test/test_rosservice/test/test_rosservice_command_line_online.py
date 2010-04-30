#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

PKG = 'test_rosservice'
NAME = 'test_rosservice_command_line_online'
import roslib; roslib.load_manifest(PKG)

import os
import sys 
import time
import unittest

import rospy
import rostest

from subprocess import Popen, PIPE, check_call, call

class TestRosserviceOnline(unittest.TestCase):

    def setUp(self):
        pass
        
    def test_rosservice(self):
        # wait for network to initialize
        services = ['/add_two_ints', '/foo/add_two_ints', '/bar/add_two_ints']
        for s in services:
            rospy.wait_for_service(s)

        cmd = 'rosservice'
        names = ['add_two_ints', '/add_two_ints', 'foo/add_two_ints', '/bar/add_two_ints']

        # list
        # - hard to exact match as we are still adding builtin services to nodes (e.g. set_logger_level)
        output = Popen([cmd, 'list'], stdout=PIPE).communicate()[0]
        l = set(output.split())
        for s in services:
            self.assert_(s in l)

        for name in names:
            # args
            output = Popen([cmd, 'args', name], stdout=PIPE).communicate()[0]
            self.assertEquals('a b', output.strip())

            # type
            output = Popen([cmd, 'type', name], stdout=PIPE).communicate()[0]
            self.assertEquals('test_ros/AddTwoInts', output.strip())

            # find
            output = Popen([cmd, 'find', 'test_ros/AddTwoInts'], stdout=PIPE).communicate()[0]
            values = [v.strip() for v in output.split('\n') if v.strip()]
            self.assertEquals(set(values), set(services))

            # uri
            output = Popen([cmd, 'uri', name], stdout=PIPE).communicate()[0]
            # - no exact answer
            self.assert_(output.startswith('rosrpc://'), output)

            # call
            output = Popen([cmd, 'call', name, '1', '2'], stdout=PIPE).communicate()[0]
            self.assertEquals('sum: 3', output.strip())

        # verify that it respects ROS_NS
        # - the uris should be different as the names should resolve differently
        env = os.environ.copy()
        env['ROS_NAMESPACE'] = 'foo'
        uri1 = Popen([cmd, 'uri', 'add_two_ints'], stdout=PIPE).communicate()[0]
        uri2 = Popen([cmd, 'uri', 'add_two_ints'], env=env, stdout=PIPE).communicate()[0]
        self.assert_(uri2.startswith('rosrpc://'))
        self.assertNotEquals(uri1, uri2)

if __name__ == '__main__':
    rostest.run(PKG, NAME, TestRosserviceOnline, sys.argv)
