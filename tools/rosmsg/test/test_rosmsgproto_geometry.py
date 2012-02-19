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

import roslib.packages
import rosmsg
from rosmsg import *

from nose.plugins.skip import SkipTest

_NO_DICT=True
if "OrderedDict" in collections.__dict__:
    _NO_DICT=False

class RosMsgProtoTest(unittest.TestCase):

    def setUp(self):
        # proto depends on python 2.7 having OrderedDict
        if _NO_DICT: raise SkipTest("Test skipped because Python version too low")
    
    def test_rosmsg_cmd_prototype_geom_msgs_Pose(self):
        self.assertEqual('"position:\n  x: 0.0\n  y: 0.0\n  z: 0.0\norientation:\n  x: 0.0\n  y: 0.0\n  z: 0.0\n  w: 0.0"', rosmsg_cmd_prototype(["msg", "geometry_msgs/Pose", "-f0"]))

    def test_rosmsg_cmd_prototype_geom_msgs_PoseArray(self):
        self.assertEqual('"header:\n  seq: 0\n  stamp:\n    secs: 0\n    nsecs: 0\n  frame_id: \'\'\nposes:\n- position:\n    x: 0.0\n    y: 0.0\n    z: 0.0\n  orientation:\n    x: 0.0\n    y: 0.0\n    z: 0.0\n    w: 0.0"', rosmsg_cmd_prototype(["msg", "geometry_msgs/PoseArray"]))
        self.assertEqual('"header:\n  seq: 0\n  stamp:\n    secs: 0\n    nsecs: 0\n  frame_id: \'\'\nposes: []"', rosmsg_cmd_prototype(["msg", "geometry_msgs/PoseArray", "-e"]))
        
    def test_rosmsg_cmd_prototype_geom_msgs_PoseStamped(self):
        self.assertEqual('"header:\n  seq: 0\n  stamp:\n    secs: 0\n    nsecs: 0\n  frame_id: \'\'\npose:\n  position:\n    x: 0.0\n    y: 0.0\n    z: 0.0\n  orientation:\n    x: 0.0\n    y: 0.0\n    z: 0.0\n    w: 0.0"', rosmsg_cmd_prototype(["msg", "geometry_msgs/PoseStamped"]))
        self.assertEqual('"header:\n  frame_id: \'\'\npose:\n  position:\n    x: 0.0\n    y: 0.0\n    z: 0.0\n  orientation:\n    x: 0.0\n    y: 0.0\n    z: 0.0\n    w: 0.0"', rosmsg_cmd_prototype(["msg", "geometry_msgs/PoseStamped", "-x", "stamp,seq"]))

    def test_rosmsg_cmd_prototype_geom_msgs_Transform(self):
        self.assertEqual('"translation:\n  x: 0.0\n  y: 0.0\n  z: 0.0\nrotation:\n  x: 0.0\n  y: 0.0\n  z: 0.0\n  w: 0.0"', rosmsg_cmd_prototype(["msg", "geometry_msgs/Transform"]))

    def test_rosmsg_cmd_prototype_geom_msgs_TransformStamped(self):
        self.assertEqual('"header:\n  seq: 0\n  stamp:\n    secs: 0\n    nsecs: 0\n  frame_id: \'\'\nchild_frame_id: \'\'\ntransform:\n  translation:\n    x: 0.0\n    y: 0.0\n    z: 0.0\n  rotation:\n    x: 0.0\n    y: 0.0\n    z: 0.0\n    w: 0.0"', rosmsg_cmd_prototype(["msg", "geometry_msgs/TransformStamped"]))
        self.assertEqual('"header:\n  seq: 0\n  stamp:\n    secs: 0\n    nsecs: 0\n  frame_id: \'\'\nchild_frame_id: \'\'\ntransform:\n  translation:\n    x: 0.0\n    y: 0.0\n    z: 0.0\n  rotation:\n    x: 0.0\n    y: 0.0\n    z: 0.0\n    w: 0.0"', rosmsg_cmd_prototype(["msg", "geometry_msgs/TransformStamped", "-f0"]))
        self.assertEqual('"{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: \'\'}, child_frame_id: \'\', transform: {\n    translation: {x: 0.0, y: 0.0, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}"', rosmsg_cmd_prototype(["msg", "geometry_msgs/TransformStamped", "-f1"]))

    def test_rosmsg_cmd_prototype_geom_msgs_TwistWithCovariance(self):
        self.assertEqual('"twist:\n  linear:\n    x: 0.0\n    y: 0.0\n    z: 0.0\n  angular:\n    x: 0.0\n    y: 0.0\n    z: 0.0\ncovariance:\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0"', rosmsg_cmd_prototype(["msg", "geometry_msgs/TwistWithCovariance", "-f0"]))
        self.assertEqual('"twist:\n  linear:\n    x: 0.0\n    y: 0.0\n    z: 0.0\n  angular:\n    x: 0.0\n    y: 0.0\n    z: 0.0\ncovariance:\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0\n- 0.0"', rosmsg_cmd_prototype(["msg", "geometry_msgs/TwistWithCovariance", "-e", "-f0"]))
        self.assertEqual('"{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, covariance: [\n    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\n    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\n    0.0, 0.0, 0.0, 0.0]}"', rosmsg_cmd_prototype(["msg", "geometry_msgs/TwistWithCovariance", "-f1"]))
        self.assertEqual('"twist:\n  linear: {x: 0.0, y: 0.0, z: 0.0}\n  angular: {x: 0.0, y: 0.0, z: 0.0}\ncovariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\n  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\n  0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"', rosmsg_cmd_prototype(["msg", "geometry_msgs/TwistWithCovariance"]))

    def test_rosmsg_cmd_prototype_geom_msgs_Polygon(self):
        self.assertEqual('"points:\n- x: 0.0\n  y: 0.0\n  z: 0.0"', rosmsg_cmd_prototype(["msg", "geometry_msgs/Polygon"]))
