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

import roslib.rosenv
import roslib.stacks
import rostest

class RoslibStacksTest(unittest.TestCase):
  
    def test_packages_of(self):
        from roslib.stacks import packages_of
        pkgs = packages_of('ros')
        for p in ['test_roslib', 'roslib', 'rospy', 'roscpp']:
            self.assert_(p in pkgs)
        # due to caching behavior, test twice
        pkgs = packages_of('ros')
        for p in ['test_roslib', 'roslib', 'rospy', 'roscpp']:
            self.assert_(p in pkgs)

        try:
            packages_of(None)
            self.fail("should have raised ValueError")
        except ValueError: pass
    
    def test_stack_of(self):
        import roslib.packages
        from roslib.stacks import stack_of
        self.assertEquals('ros', stack_of('test_roslib'))
        # due to caching, test twice
        self.assertEquals('ros', stack_of('test_roslib'))
        try:
            stack_of('fake_test_roslib')
            self.fail("should have failed")
        except roslib.packages.InvalidROSPkgException:
            pass
    
    def test_get_stack_dir(self):
        # TODO setup artificial tree with more exhaustive tests
        import roslib.rosenv
        from roslib.stacks import get_stack_dir, InvalidROSStackException
        self.assertEquals(roslib.rosenv.get_ros_root(), get_stack_dir('ros'))
        self.assertEquals(None, get_stack_dir('non_existent'))
    
    def test_expand_to_packages(self):
        from roslib.stacks import expand_to_packages
        try:
            # it's possible to accidentally pass in a sequence type
            # like a string and get weird results, so check that we
            # don't
            self.assertEquals(([], []), expand_to_packages('ros'))
            self.fail("expand_to_packages should only take in a list of strings")
        except ValueError: pass
        
        self.assertEquals(([], []), expand_to_packages([]))
        self.assertEquals((['rospy', 'test_roslib', 'roslib'], []), expand_to_packages(['rospy', 'test_roslib', 'roslib']))
        self.assertEquals(([], ['bogus_one', 'bogus_two']), expand_to_packages(['bogus_one', 'bogus_two']))
  
        self.assertEquals(([], ['bogus_one', 'bogus_two']), expand_to_packages(['bogus_one', 'bogus_two']))      
  
        # TODO: setup directory tree so that this can be more precisely calculated
        valid, invalid = expand_to_packages(['ros', 'bogus_one'])
        self.assertEquals(['bogus_one'], invalid)
        check = ['rospy', 'roscpp', 'rospack', 'rosmake', 'roslib', 'rostest', 'std_msgs']
        for c in check:
            self.assert_(c in valid, "expected [%s] to be in ros expansion"%c)
      
if __name__ == '__main__':
    rostest.unitrun('test_roslib', 'test_stacks', RoslibStacksTest, coverage_packages=['roslib.stacks'])

