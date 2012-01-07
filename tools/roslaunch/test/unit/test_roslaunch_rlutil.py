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
# Revision $Id: roslaunch_node_args.py 5229 2009-07-16 22:31:17Z sfkwc $

import os
import sys
import unittest
    
import roslaunch
import roslaunch.rlutil

def get_test_path():
    # two directories up from here
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

# path to example.launch directory
def get_example_path():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'resources'))

## Test roslaunch.node_args
class TestRoslaunchRlutil(unittest.TestCase):

    def test_resolve_launch_arguments(self):
        from roslaunch.rlutil import resolve_launch_arguments

        roslaunch_dir = get_test_path()
        example_xml_p = os.path.join(get_example_path(), 'example.launch')
        tests = [
            ([], []),
            (['roslaunch', 'example.launch'], [example_xml_p]),
            ([example_xml_p], [example_xml_p]),

            (['roslaunch', 'example.launch', 'foo', 'bar'], [example_xml_p, 'foo', 'bar']),
            ([example_xml_p, 'foo', 'bar'], [example_xml_p,'foo', 'bar']),

            ]
        bad = [
            ['does_not_exist'],
            ['does_not_exist', 'foo.launch'],
            ['roslaunch', 'does_not_exist.launch'],                        
            ]

        for test, result in tests:
            for v1, v2 in zip(result, resolve_launch_arguments(test)):
                # workaround for nfs 
                if os.path.exists(v1):
                    self.assert_(os.path.samefile(v1, v2))
                else:
                    self.assertEquals(v1, v2)
        for test in bad:
            try:
                resolve_launch_arguments(test)
                self.fail("should have failed")
            except roslaunch.RLException:
                pass
            
