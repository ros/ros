#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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

# Author: Thibault Kruse

from __future__ import with_statement
NAME = 'test_rosmsgproto'

import os
import sys 
import unittest
import cStringIO
import time
import copy
import rostest
import subprocess
from subprocess import Popen, PIPE, call
import collections
import rosmsg

from nose.plugins.skip import SkipTest

ROSMSGPROTO_FN = [os.path.join(os.getcwd(), '../scripts/rosmsg-proto')]
_NO_DICT = True
if "OrderedDict" in collections.__dict__:
    _NO_DICT = False

class RosMsgProtoCommandlineTestMsg(unittest.TestCase):

    def setUp(self):
        # proto depends on python 2.7 having OrderedDict
        if _NO_DICT: raise SkipTest("Test skipped because Python version too low")
        self.new_environ = os.environ
        self.new_environ["PYTHONPATH"] = os.path.join(os.getcwd(), "src")+':'+os.environ['PYTHONPATH']
    
    def testFail(self):
        cmd = copy.copy(ROSMSGPROTO_FN)
        cmd.extend(["msg", "foo123barxyz"])
        call = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE, env = self.new_environ)
        (output, erroutput) = call.communicate()
        self.assertEqual('', output)
        self.assertTrue('Unknown message name foo123barxyz' in erroutput)

    def testSilentFail(self):
        cmd = copy.copy(ROSMSGPROTO_FN)
        cmd.extend(["msg", "-s", "foo123barxyz"])
        call = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE, env = self.new_environ)
        (output, erroutput) = call.communicate()
        self.assertEqual('', output)
        self.assertEqual('', erroutput)

    def testSilentFailCpp(self):
        cmd = copy.copy(ROSMSGPROTO_FN)
        cmd.extend(["msg", "-s", "foo123barxyz::bar"])
        call = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE, env = self.new_environ)
        (output, erroutput) = call.communicate()
        self.assertEqual('', output)
        self.assertEqual('', erroutput)

    def testSilentFailDot(self):
        cmd = copy.copy(ROSMSGPROTO_FN)
        cmd.extend(["msg", "-s", "foo123barxyz.bar"])
        call = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE, env = self.new_environ)
        (output, erroutput) = call.communicate()
        self.assertEqual('', output)
        self.assertEqual('', erroutput)

    def testSilentFailMode(self):
        cmd = copy.copy(ROSMSGPROTO_FN)
        cmd.extend(["msgfoobar", "-s", "foo123barxyz.bar"])
        call = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE, env = self.new_environ)
        (output, erroutput) = call.communicate()
        self.assertEqual('', output)
        self.assertEqual('', erroutput)
