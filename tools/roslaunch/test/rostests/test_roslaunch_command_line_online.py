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
# Revision $Id: test_roslaunch_command_line_online.py 6411 2009-10-02 21:32:01Z kwc $

PKG = 'roslaunch'
NAME = 'test_roslaunch_command_line_online'

import os
import sys 
import time
import unittest
import yaml

import rostest

from subprocess import Popen, PIPE, check_call, call

class TestRoslaunchOnline(unittest.TestCase):

    def setUp(self):
        self.vals = set()
        self.msgs = {}

    def test_roslaunch(self):
        # network is initialized
        cmd = 'roslaunch'

        # regression test for #1994
        # --wait
        # master is already running, noop only sets params, so this should return
        check_call([cmd, '--wait', 'roslaunch', 'noop.launch'])

        # tripwire test for #2370, not really possible to validate output on this
        check_call([cmd, '--screen', 'roslaunch', 'noop.launch'])        

if __name__ == '__main__':
    rostest.run(PKG, NAME, TestRoslaunchOnline, sys.argv)
