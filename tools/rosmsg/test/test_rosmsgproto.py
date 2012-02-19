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
import std_msgs

import rostest

import rosmsg
from rosmsg import *

from nose.plugins.skip import SkipTest

_NO_DICT=True
if "OrderedDict" in collections.__dict__:
    _NO_DICT=False

class RosMsgProtoTest(unittest.TestCase):

    def test_get_array_type_instance(self):
        self.assertEqual(0, get_array_type_instance("int16[]"))
        self.assertEqual(0, get_array_type_instance("char[]"))
        self.assertEqual(0, get_array_type_instance("uint16[]"))
        self.assertEqual(0, get_array_type_instance("int32[]"))
        self.assertEqual(0, get_array_type_instance("uint16[]"))
        self.assertEqual(False,get_array_type_instance("bool[]"))
        self.assertEqual(0, get_array_type_instance("float32[]"))
        self.assertEqual(0, get_array_type_instance("float64[]"))
        self.assertEqual("", get_array_type_instance("string[]"))
        self.assertFalse(None == get_array_type_instance("time[]"))
        self.assertFalse(None == get_array_type_instance("duration[]"))
        self.assertTrue(None == get_array_type_instance("colorRGBA[]"))
        self.assertTrue(None == get_array_type_instance("empty[]"))
        # TODO check for complex types


    def test_create_names_filter(self):
        class foo:
            def __init__(self):
                self.__slots__= ["bar","foo","bar","baz","bar"]
        self.assertEqual(["foo", "baz"], create_names_filter("bar")(foo()))


        
    def test_rosmsg_cmd_prototype_std_msgs_Int16(self):
        if _NO_DICT: raise SkipTest("Test skipped because Python version too low")
        self.assertEqual('"data: 0"', rosmsg_cmd_prototype(["msg", "std_msgs/Int16"]))
        self.assertEqual('data: 0', rosmsg_cmd_prototype(["msg", "std_msgs/Int16", "-H"]))
        self.assertEqual('"  data: 0"', rosmsg_cmd_prototype(["msg", "std_msgs/Int16", "-p", "  "]))
        self.assertEqual('  data: 0', rosmsg_cmd_prototype(["msg", "std_msgs/Int16", "-p", "  ", "-H"]))
        self.assertEqual('"{}"', rosmsg_cmd_prototype(["msg", "std_msgs/Int16","-x", "data"]))

    def test_rosmsg_cmd_prototype_std_msgs_String(self):
        if _NO_DICT: raise SkipTest("Test skipped because Python version too low")
        self.assertEqual('"data: \'\'"', rosmsg_cmd_prototype(["msg", "std_msgs/String"]))
        self.assertEqual('data: \'\'', rosmsg_cmd_prototype(["msg", "std_msgs/String", "-H"]))
        self.assertEqual('  data: \'\'', rosmsg_cmd_prototype(["msg", "std_msgs/String", "-p", "  ", "-H"]))

    def test_rosmsg_cmd_prototype_std_msgs_Header(self):
        if _NO_DICT: raise SkipTest("Test skipped because Python version too low")
        self.assertEqual('"seq: 0\nstamp:\n  secs: 0\n  nsecs: 0\nframe_id: \'\'"', rosmsg_cmd_prototype(["msg", "std_msgs/Header"]))
        self.assertEqual('"{seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: \'\'}"', rosmsg_cmd_prototype(["msg", "std_msgs/Header", "-f1"]))

    def test_rosmsg_cmd_prototype_std_msgs_Bool(self):
        if _NO_DICT: raise SkipTest("Test skipped because Python version too low")
        self.assertEqual('"data: false"', rosmsg_cmd_prototype(["msg", "std_msgs/Bool"]))

    def test_rosmsg_cmd_prototype_std_msgs_Time(self):
        if _NO_DICT: raise SkipTest("Test skipped because Python version too low")
        self.assertEqual('"data:\n  secs: 0\n  nsecs: 0"', rosmsg_cmd_prototype(["msg", "std_msgs/Time"]))
        self.assertEqual('"{data: {secs: 0, nsecs: 0}}"', rosmsg_cmd_prototype(["msg", "std_msgs/Time", "-f1"]))

    def test_rosmsg_cmd_prototype_std_msgs_Duration(self):
        if _NO_DICT: raise SkipTest("Test skipped because Python version too low")
        self.assertEqual('"data:\n  secs: 0\n  nsecs: 0"', rosmsg_cmd_prototype(["msg", "std_msgs/Duration"]))
        self.assertEqual('"{data: {secs: 0, nsecs: 0}}"', rosmsg_cmd_prototype(["msg", "std_msgs/Duration", "-f1"]))




