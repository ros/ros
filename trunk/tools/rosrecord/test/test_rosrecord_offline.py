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
# Revision $Id: test_rostopic_command_line_offline.py 5710 2009-08-20 03:11:04Z sfkwc $

from __future__ import with_statement

NAME = 'test_rosrecord_offline'
import roslib; roslib.load_manifest('rosrecord')

import os
import sys 
import unittest
import cStringIO
import time
        
import rostest

from subprocess import Popen, PIPE, check_call, call

class TestRosrecordOffline(unittest.TestCase):

    def setUp(self):
        pass

    ## test that the rosmsg command works
    def test_rosrecord_help(self):
        self.do_test_help('rosrecord');

    def test_rosplay_help(self):
        self.do_test_help('rosplay');

    def test_rosrebag_help(self):
        self.do_test_help('rosrebag');

    def test_rosrecord_pkg_help(self):
        self.do_test_help('rosrun rosrecord rosrecord');

    def test_rosplay_pkg_help(self):
        self.do_test_help('rosrun rosrecord rosplay');

    def do_test_help(self, cmd):
        rosrecord = Popen(cmd.split() + ['-h'], stdout=PIPE, stderr=PIPE)
        output = rosrecord.communicate()
        self.assert_('Usage:' in output[0] or 'Usage:' in output[1])

if __name__ == '__main__':
    rostest.unitrun('rosrecord', NAME, TestRosrecordOffline, sys.argv, coverage_packages=[])
