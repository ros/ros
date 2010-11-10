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
import struct
import sys
import unittest

import rosunit

class RoslibRospackTest(unittest.TestCase):
  
  def test_rospack(self):
    from roslib.rospack import rospackexec, rospack_depends, rospack_depends_1,\
        rospack_depends_on, rospack_depends_on_1
    
    val = rospackexec(['list'])
    self.assertEquals(set(['rospack']), set(rospack_depends('roslib')))
    self.assertEquals(set(['rospack']), set(rospack_depends_1('roslib')))    
    self.assertEquals(set(['roslib', 'roslang', 'std_msgs', 'rosgraph_msgs']), set(rospack_depends_1('rospy')))
    self.assertEquals(set(['rospack', 'roslib', 'std_msgs', 'rosgraph_msgs', 'roslang']), set(rospack_depends('rospy')))

    val = rospack_depends_on('roslang')
    self.assert_('rospy' in val, val)
    self.assert_('roscpp' in val)    
    val = rospack_depends_on_1('roslang')
    self.assert_('rospy' in val)
    self.assert_('roscpp' in val)

  def test_rosstack(self):
    from roslib.rospack import rosstackexec, rosstack_depends, rosstack_depends_1,\
        rosstack_depends_on, rosstack_depends_on_1
    self.assertEquals(set([]), set(rosstack_depends('ros')))
    self.assertEquals(set([]), set(rosstack_depends_1('ros')))
    
    # can't actually test these w/o setting up stack environment, so just make sure they don't throw
    self.assert_(rosstack_depends_on('ros') is not None)
    self.assert_(rosstack_depends_on_1('ros') is not None)


if __name__ == '__main__':
  rosunit.unitrun('test_roslib', 'test_rospack', RoslibRospackTest, coverage_packages=['roslib.rospack'])

