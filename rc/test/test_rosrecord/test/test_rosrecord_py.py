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
import roslib; roslib.load_manifest('test_roslib')

import os
import sys
import unittest

import rostest

class RosrecordTest(unittest.TestCase):
  
  def test_logplayer_normal(self):
      from ros import rosrecord
      import roslib.packages
      d = os.path.join(roslib.packages.get_pkg_dir('test_rosrecord'), 'test')
      f = os.path.join(d, 'hello_world.bag')
      self.assert_(os.path.isfile(f))
      from std_msgs.msg import String
      for topic, msg, t in rosrecord.logplayer(f):
          self.assertEquals('chatter', topic)
          self.assertEquals('hello world', msg.data)
          self.assertEquals(msg._type, 'std_msgs/String')
          # make sure message time is within expected range
          self.assert_(abs(t.to_sec() - 1260988735.) < 30.)
          
  def test_logplayer_raw(self):
      from ros import rosrecord
      import roslib.packages
      d = os.path.join(roslib.packages.get_pkg_dir('test_rosrecord'), 'test')
      f = os.path.join(d, 'hello_world.bag')
      self.assert_(os.path.isfile(f))
      from std_msgs.msg import String
      for topic, msg, t in rosrecord.logplayer(f, raw=True):
          self.assertEquals('chatter', topic)
          datatype = msg[0]
          data = msg[1]
          md5sum = msg[2]
          bag_position = msg[3]
          self.assertEquals(datatype, 'std_msgs/String')
          self.assertEquals('992ce8a1687cec8c8bd883ec73ca41d1', md5sum)
          # need better testing than this
          self.assert_(type(bag_position == int))
          # make sure message time is within expected range
          self.assert_(abs(t.to_sec() - 1260988735.) < 30.)
          # test that we can deserialize it
          s = String()
          s.deserialize(data)
          self.assertEquals('hello world', s.data)

          
    
if __name__ == '__main__':
  rostest.unitrun('test_rosrecord_py', 'test_rosrecord', RosrecordTest, coverage_packages=['rosrecord'])

