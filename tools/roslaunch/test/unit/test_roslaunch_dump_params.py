#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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
import time
import unittest
import yaml

from subprocess import Popen, PIPE, check_call, call

class TestDumpParams(unittest.TestCase):

    def setUp(self):
        pass

    def test_roslaunch(self):
        # network is initialized
        cmd = 'roslaunch'

        # Smoke test for testing parameters
        p = Popen([cmd, '--dump-params', 'roslaunch', 'noop.launch'], stdout = PIPE)
        o, e = p.communicate()
        self.assert_(p.returncode == 0, "Return code nonzero for param dump! Code: %d" % (p.returncode))

        self.assertEquals({'/noop': 'noop'}, yaml.load(o))

        p = Popen([cmd, '--dump-params', 'roslaunch', 'test-dump-rosparam.launch'], stdout = PIPE)
        o, e = p.communicate()
        self.assert_(p.returncode == 0, "Return code nonzero for param dump! Code: %d" % (p.returncode))

        val = {
            '/string1': 'bar',
            '/dict1/head': 1,
            '/dict1/shoulders': 2,
            '/dict1/knees': 3,
            '/dict1/toes': 4,

            '/rosparam/string1': 'bar',
            '/rosparam/dict1/head': 1,
            '/rosparam/dict1/shoulders': 2,
            '/rosparam/dict1/knees': 3,
            '/rosparam/dict1/toes': 4,
            
            '/node_rosparam/string1': 'bar',
            '/node_rosparam/dict1/head': 1,
            '/node_rosparam/dict1/shoulders': 2,
            '/node_rosparam/dict1/knees': 3,
            '/node_rosparam/dict1/toes': 4,

            '/inline_str': 'value1',
            '/inline_list': [1, 2, 3, 4],
            '/inline_dict/key1': 'value1',
            '/inline_dict/key2': 'value2',

            '/inline_dict2/key3': 'value3',
            '/inline_dict2/key4': 'value4',
            
            '/override/key1': 'override1',
            '/override/key2': 'value2',
            '/noparam1': 'value1',
            '/noparam2': 'value2',
            }
        output_val = yaml.load(o)
        if not val == output_val:
            for k, v in val.iteritems():
                if k not in output_val:
                    self.fail("key [%s] not in output: %s"%(k, output_val))
                elif v != output_val[k]:
                    self.fail("key [%s] value [%s] does not match output: %s"%(k, v, output_val[k])) 
        self.assertEquals(val, output_val)
