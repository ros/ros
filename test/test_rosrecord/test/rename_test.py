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

import roslib
roslib.load_manifest('test_rosrecord')

import unittest
import rospy
import rostest
import sys

import test_rosrecord.msg
import test_crosspackage.msg

class RenameTest(unittest.TestCase):

  def msgmove_cb(self, msg):
    self.move_received_msg = True
    if (msg._type != "test_rosrecord/TestMsgMove"):
      self.correct_type = False

    if ("Foo " + str(self.move_count) != msg.field1):
      self.field1_valid = False

    if (self.move_count != msg.field2):
      self.field2_valid = False

    if ("Bar " + str(1000 - self.move_count) != msg.field3.field1):
      self.subfield1_valid = False

    if (1000 - self.move_count != msg.field3.field2):
      self.subfield2_valid = False

    self.move_count = self.move_count + 1

  def msgrename_cb(self, msg):
    self.rename_received_msg = True
    if (msg._type != "test_crosspackage/TestMsgRename"):
      self.correct_type = False

    if ("Foo " + str(self.rename_count) != msg.field1):
      self.field1_valid = False

    if (self.rename_count != msg.field2):
      self.field2_valid = False

    if ("Bar " + str(1000 - self.rename_count) != msg.field3.field1):
      self.subfield1_valid = False

    if (1000 - self.rename_count != msg.field3.field2):
      self.subfield2_valid = False

    self.rename_count = self.rename_count + 1



  def test_rename(self):
    rospy.init_node('rename_test', anonymous=True)

    self.move_count = 1
    self.move_received_msg = False
    self.move_correct_type = True
    self.move_field1_valid = True
    self.move_field2_valid = True
    self.move_subfield1_valid = True
    self.move_subfield2_valid = True
    rospy.Subscriber('test_msg', test_rosrecord.msg.TestMsgMove, self.msgmove_cb)

    self.rename_count = 1
    self.rename_received_msg = False
    self.rename_correct_type = True
    self.rename_field1_valid = True
    self.rename_field2_valid = True
    self.rename_subfield1_valid = True
    self.rename_subfield2_valid = True
    rospy.Subscriber('test_msg', test_crosspackage.msg.TestMsgRename, self.msgrename_cb)
    
    r = rospy.Rate(10)

    # We know this is when the bag file should be over
    while (rospy.rostime.get_time() < 1245707880.0):
      r.sleep()

    self.assertTrue(self.move_correct_type)
    self.assertTrue(self.move_received_msg)
    self.assertTrue(self.move_field1_valid)
    self.assertTrue(self.move_field2_valid)
    self.assertTrue(self.move_subfield1_valid)
    self.assertTrue(self.move_subfield2_valid)

    self.assertTrue(self.rename_correct_type)
    self.assertTrue(self.rename_received_msg)
    self.assertTrue(self.rename_field1_valid)
    self.assertTrue(self.rename_field2_valid)
    self.assertTrue(self.rename_subfield1_valid)
    self.assertTrue(self.rename_subfield2_valid)
    

if __name__ == '__main__':
  rostest.rosrun('test_rosrecord', 'rename_test', RenameTest, sys.argv)
