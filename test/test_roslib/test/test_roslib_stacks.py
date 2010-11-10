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
import rosunit

class RoslibStacksTest(unittest.TestCase):
  
    def test_packages_of(self):
        from roslib.stacks import packages_of
        pkgs = packages_of('ros_comm')
        for p in ['test_roslib', 'rospy', 'roscpp']:
            self.assert_(p in pkgs)
        # due to caching behavior, test twice
        pkgs = packages_of('ros_comm')
        for p in ['test_roslib', 'rospy', 'roscpp']:
            self.assert_(p in pkgs)

        try:
            packages_of(None)
            self.fail("should have raised ValueError")
        except ValueError: pass
    
    def test_stack_of(self):
        import roslib.packages
        from roslib.stacks import stack_of
        self.assertEquals('ros_comm', stack_of('test_roslib'))
        # due to caching, test twice
        self.assertEquals('ros_comm', stack_of('test_roslib'))
        try:
            stack_of('fake_test_roslib')
            self.fail("should have failed")
        except roslib.packages.InvalidROSPkgException:
            pass

    def test_list_stacks(self):
        from roslib.stacks import list_stacks
        l = list_stacks()
        self.assert_('ros_comm' in l)

        # make sure it is equivalent to rosstack list
        from roslib.rospack import rosstackexec
        l2 = [x for x in rosstackexec(['list']).split('\n') if x]
        l2 = [x.split()[0] for x in l2]
        self.assertEquals(set(l), set(l2), set(l) ^ set(l2))

    def test_list_stacks_by_path(self):
        from roslib.stacks import list_stacks_by_path

        # test with the ros stack
        rr = roslib.rosenv.get_ros_root()
        self.assertEquals(['ros'], list_stacks_by_path(rr))
        stacks = []
        self.assertEquals(['ros'], list_stacks_by_path(rr, stacks))
        self.assertEquals(['ros'], stacks)
        self.assertEquals(['ros'], list_stacks_by_path(rr, stacks))

        stacks.extend(['fake_stack', 'fake_stack2'])
        self.assertEquals(['ros', 'fake_stack', 'fake_stack2'], list_stacks_by_path(rr, stacks))

        cache = {}
        self.assertEquals(['ros'], list_stacks_by_path(rr, cache=cache))
        self.assertEquals({'ros': rr}, cache)

        # test with synthetic stacks
        test_dir = os.path.join(roslib.packages.get_pkg_dir('test_roslib'), 'test', 'stack_tests')
        self.assertEquals(set(['bar', 'foo']), set(list_stacks_by_path(test_dir)))

        test_dir = os.path.join(roslib.packages.get_pkg_dir('test_roslib'), 'test', 'stack_tests', 's1')
        self.assertEquals(set(['bar', 'foo']), set(list_stacks_by_path(test_dir)))

        test_dir = os.path.join(roslib.packages.get_pkg_dir('test_roslib'), 'test', 'stack_tests', 's1', 'bar')
        self.assertEquals(['bar'], list_stacks_by_path(test_dir))
        
        # test symlink following

        test_dir = os.path.join(roslib.packages.get_pkg_dir('test_roslib'), 'test', 'stack_tests2')
        self.assertEquals(set(['foo', 'bar']), set(list_stacks_by_path(test_dir)))
        
    def test_get_stack_dir(self):
        import roslib.rosenv
        import roslib.packages
        from roslib.stacks import get_stack_dir, InvalidROSStackException, list_stacks
        self.assertEquals(roslib.rosenv.get_ros_root(), get_stack_dir('ros'))
        try:
            get_stack_dir('non_existent')
            self.fail("should have raised")
        except roslib.stacks.InvalidROSStackException:
            pass

        # make sure it agrees with rosstack
        stacks = list_stacks()
        from roslib.rospack import rosstackexec
        for s in stacks:
            self.assertEquals(get_stack_dir(s), rosstackexec(['find', s]))

        # now manipulate the environment to test precedence
        # - save original RPP as we popen rosstack in other tests
        rpp = os.environ.get(roslib.rosenv.ROS_PACKAGE_PATH, None)
        try:
            d = roslib.packages.get_pkg_dir('test_roslib')
            d = os.path.join(d, 'test', 'stack_tests')

            # - s1/s2/s3
            print "s1/s2/s3"            
            paths = [os.path.join(d, p) for p in ['s1', 's2', 's3']]
            os.environ[roslib.rosenv.ROS_PACKAGE_PATH] = os.pathsep.join(paths)
            # - run multiple times to test caching
            for i in xrange(2):
                stacks = roslib.stacks.list_stacks()
                self.assert_('foo' in stacks)
                self.assert_('bar' in stacks)

                foo_p = os.path.join(d, 's1', 'foo')
                bar_p = os.path.join(d, 's1', 'bar')
                self.assertEquals(foo_p, roslib.stacks.get_stack_dir('foo'))
                self.assertEquals(bar_p, roslib.stacks.get_stack_dir('bar'))

            # - s2/s3/s1
            print "s2/s3/s1"
            
            paths = [os.path.join(d, p) for p in ['s2', 's3', 's1']]
            os.environ[roslib.rosenv.ROS_PACKAGE_PATH] = os.pathsep.join(paths)
            stacks = roslib.stacks.list_stacks()
            self.assert_('foo' in stacks)
            self.assert_('bar' in stacks)

            foo_p = os.path.join(d, 's2', 'foo')
            bar_p = os.path.join(d, 's1', 'bar')
            self.assertEquals(foo_p, roslib.stacks.get_stack_dir('foo'))
            self.assertEquals(bar_p, roslib.stacks.get_stack_dir('bar'))
        finally:
            #restore rpp
            if rpp is not None:
                os.environ[roslib.rosenv.ROS_PACKAGE_PATH] = rpp
            else:
                del os.environ[roslib.rosenv.ROS_PACKAGE_PATH] 
            
    def test_expand_to_packages(self):
        from roslib.stacks import expand_to_packages
        try:
            # it's possible to accidentally pass in a sequence type
            # like a string and get weird results, so check that we
            # don't
            self.assertEquals(([], []), expand_to_packages('ros_comm'))
            self.fail("expand_to_packages should only take in a list of strings")
        except ValueError: pass
        
        self.assertEquals(([], []), expand_to_packages([]))
        self.assertEquals((['rospy', 'test_roslib', 'roslib'], []), expand_to_packages(['rospy', 'test_roslib', 'roslib']))
        self.assertEquals(([], ['bogus_one', 'bogus_two']), expand_to_packages(['bogus_one', 'bogus_two']))
  
        self.assertEquals(([], ['bogus_one', 'bogus_two']), expand_to_packages(['bogus_one', 'bogus_two']))      
  
        # TODO: setup directory tree so that this can be more precisely calculated
        valid, invalid = expand_to_packages(['ros_comm', 'bogus_one'])
        self.assertEquals(['bogus_one'], invalid)
        check = ['rospy', 'roscpp', 'rostest', 'std_msgs']
        for c in check:
            self.assert_(c in valid, "expected [%s] to be in ros expansion"%c)
      
if __name__ == '__main__':
    rosunit.unitrun('test_roslib', 'test_stacks', RoslibStacksTest, coverage_packages=['roslib.stacks'])

