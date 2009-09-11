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

import rostest

class RoslibScriptutilTest(unittest.TestCase):
  
  def test_rospack(self):
    from roslib.scriptutil import rospackexec, rospack_depends, rospack_depends_1,\
        rospack_depends_on, rospack_depends_on_1
    
    val = rospackexec(['list'])
    self.assertEquals(['genmsg_cpp'], rospack_depends('roslib'))
    self.assertEquals(['genmsg_cpp'], rospack_depends_1('roslib'))    
    self.assertEquals(set(['roslib', 'roslang']), set(rospack_depends_1('rospy')))
    self.assertEquals(set(['roslib', 'roslang', 'genmsg_cpp']), set(rospack_depends('rospy')))

    val = rospack_depends_on('roslang')
    self.assert_('rospy' in val, val)
    self.assert_('roscpp' in val)    
    val = rospack_depends_on_1('roslang')
    self.assert_('rospy' in val)
    self.assert_('roscpp' in val)

  def test_get_message_class(self):
    from roslib.scriptutil import get_message_class

    try:
      self.assertEquals(None, get_message_class('String'))
      self.fail("should have thrown ValueError")
    except ValueError: pass
    # non-existent package
    self.assertEquals(None, get_message_class('fake/Fake'))
    # non-existent message
    self.assertEquals(None, get_message_class('roslib/Fake'))
    # package with no messages
    self.assertEquals(None, get_message_class('genmsg_cpp/Fake'))
    
    import roslib.msg
    self.assertEquals(roslib.msg.Header, get_message_class('Header'))
    self.assertEquals(roslib.msg.Header, get_message_class('roslib/Header'))
    self.assertEquals(roslib.msg.Log, get_message_class('roslib/Log'))    

  def test_get_service_class(self):
    from roslib.scriptutil import get_service_class

    # non-existent package
    self.assertEquals(None, get_service_class('fake/Fake'))
    # non-existent message
    self.assertEquals(None, get_service_class('roslib/Fake'))
    # package with no messages
    self.assertEquals(None, get_service_class('genmsg_cpp/Fake'))
    
    # hidden dep, should really move roslib/test to test_roslib now
    roslib.load_manifest('std_srvs')
    import std_srvs.srv 
    self.assertEquals(std_srvs.srv.Empty, get_service_class('std_srvs/Empty'))    
    
if __name__ == '__main__':
  rostest.unitrun('test_roslib', 'test_scriptutil', RoslibScriptutilTest, coverage_packages=['roslib.scriptutil'])

