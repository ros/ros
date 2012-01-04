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

import os
import sys
import unittest

import rospkg

def get_test_path():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), 'fake-pkg'))

class RoscreateStackTest(unittest.TestCase):
  
  def test_command_line(self):
    from subprocess import Popen, PIPE
    from rospkg import get_ros_root
    # ros has no deps.  this test is also a tripwire for crasher bug
    output = Popen(['roscreate-stack', get_ros_root(), '--show-deps'], stdout=PIPE, stderr=PIPE).communicate()
    self.assertEquals('', output[0].strip())
    self.assertEquals('', output[1].strip())
    
    # go into fake environment
    d = get_test_path()
    # manipulate our environment as roscreate-stack is dependent on this
    os.environ['ROS_PACKAGE_PATH'] = d
    stack1 = os.path.join(d, 'stack1')
    stack2 = os.path.join(d, 'stack2')
    
    output = Popen(['roscreate-stack', stack1, '--show-deps'], stdout=PIPE, stderr=PIPE).communicate()
    self.assertEquals('<depend stack="ros"/> <!-- roslib -->', output[0].strip())
    self.assertEquals('', output[1].strip())

    output = Popen(['roscreate-stack', stack2, '--show-deps'], stdout=PIPE, stderr=PIPE).communicate()
    self.assert_('<depend stack="ros"/>' in output[0], output[0])
    self.assert_('<depend stack="stack1"/>' in output[0], output[0])
    self.assertEquals(2, len(output[0].strip().split('\n')))
    self.assertEquals('', output[1].strip())
    
  def test_compute_stack_depends_and_licenses(self):
    # this will catch only the most basic of bugs. the issue here is
    # that the test can't assume the existence of other stacks, so we
    # need to create an artificial tree

    from roscreate.roscreatestack import compute_stack_depends_and_licenses

    d = get_test_path()
    print "TEST PATH", d
    # manipulate our environment as roscreate-stack is dependent on this
    os.environ['ROS_PACKAGE_PATH'] = d
    stack1 = os.path.join(d, 'stack1')
    stack2 = os.path.join(d, 'stack2')

    # test on stack1
    depends, licenses = compute_stack_depends_and_licenses(stack1)
    self.assertEquals(['ros'], depends.keys())
    self.assertEquals(['roslib'], depends['ros'])
    self.assertEquals(set(['BSD']), licenses)

    d2, licenses = compute_stack_depends_and_licenses(stack2)
    self.assertEquals(set(['ros', 'stack1']), set(d2.keys()))
    self.assertEquals(['depends_roslib'], d2['stack1'])
    self.assertEquals(set(['BSD']), licenses)

    
    # test on ros_root
    depends, licenses = compute_stack_depends_and_licenses(os.environ['ROS_ROOT'])
    self.assertEquals({}, depends)
    self.assert_('BSD' in licenses)
