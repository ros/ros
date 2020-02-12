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
import unittest

import roslib

import rospkg


class RoslibStacksTest(unittest.TestCase):

    def test_list_stacks(self):
        from roslib.stacks import list_stacks
        # roslib can't depend on ros and therefore can't expect it being in the environment
        # l = list_stacks()
        # self.assert_('ros' in l)

        # test with env
        test_dir = os.path.join(roslib.packages.get_pkg_dir('roslib'), 'test', 'stack_tests', 's1')
        env = os.environ.copy()
        env['ROS_PACKAGE_PATH'] = test_dir
        val = set(list_stacks(env=env))
        # ros stack not guaranteed to list anymore as ROS_ROOT may not be set
        if 'ros' in val:
            val.remove('ros')
        self.assertEquals({'foo', 'bar'}, val)

    def test_list_stacks_by_path(self):
        from roslib.stacks import list_stacks_by_path

        # test with synthetic stacks
        test_dir = os.path.join(roslib.packages.get_pkg_dir('roslib'), 'test', 'stack_tests')
        self.assertEquals({'bar', 'foo'}, set(list_stacks_by_path(test_dir)))

        test_dir = os.path.join(roslib.packages.get_pkg_dir('roslib'), 'test', 'stack_tests', 's1')
        self.assertEquals({'bar', 'foo'}, set(list_stacks_by_path(test_dir)))

        test_dir = os.path.join(roslib.packages.get_pkg_dir('roslib'), 'test', 'stack_tests', 's1', 'bar')
        self.assertEquals(['bar'], list_stacks_by_path(test_dir))

        # test symlink following

        test_dir = os.path.join(roslib.packages.get_pkg_dir('roslib'), 'test', 'stack_tests2')
        self.assertEquals({'foo', 'bar'}, set(list_stacks_by_path(test_dir)))

    def test_list_stacks_by_path_unary(self):
        from roslib.stacks import list_stacks_by_path
        # test with synthetic stacks
        test_dir = os.path.join(roslib.packages.get_pkg_dir('roslib'), 'test', 'stack_tests_unary')
        self.assertEquals({'bar', 'foo', 'baz'}, set(list_stacks_by_path(test_dir)))

    def test_get_stack_dir_unary(self):
        # now manipulate the environment to test precedence
        # - save original RPP as we popen rosstack in other tests
        d = roslib.packages.get_pkg_dir('roslib')
        d = os.path.join(d, 'test', 'stack_tests_unary')
        s1_d = os.path.join(d, 's1')
        rpp = rospkg.get_ros_package_path()
        try:
            paths = [d]
            os.environ[rospkg.environment.ROS_PACKAGE_PATH] = os.pathsep.join(paths)
            self.assertEquals(os.path.join(s1_d, 'foo'), roslib.stacks.get_stack_dir('foo'))
            self.assertEquals(os.path.join(s1_d, 'bar'), roslib.stacks.get_stack_dir('bar'))
            self.assertEquals(os.path.join(s1_d, 'baz'), roslib.stacks.get_stack_dir('baz'))
        finally:
            # restore rpp
            if rpp is not None:
                os.environ[rospkg.environment.ROS_PACKAGE_PATH] = rpp
            else:
                del os.environ[rospkg.environment.ROS_PACKAGE_PATH]

    def test_get_stack_dir(self):
        import roslib.packages
        from roslib.stacks import get_stack_dir
        try:
            get_stack_dir('non_existent')
            self.fail('should have raised')
        except roslib.stacks.InvalidROSStackException:
            pass

        # now manipulate the environment to test precedence
        # - save original RPP as we popen rosstack in other tests
        rpp = os.environ.get(rospkg.environment.ROS_PACKAGE_PATH, None)
        try:
            d = roslib.packages.get_pkg_dir('roslib')
            d = os.path.join(d, 'test', 'stack_tests')

            # - s1/s2/s3
            print('s1/s2/s3')
            paths = [os.path.join(d, p) for p in ['s1', 's2', 's3']]
            os.environ[rospkg.environment.ROS_PACKAGE_PATH] = os.pathsep.join(paths)
            # - run multiple times to test caching
            for i in range(2):
                stacks = roslib.stacks.list_stacks()
                self.assert_('foo' in stacks)
                self.assert_('bar' in stacks)

                foo_p = os.path.join(d, 's1', 'foo')
                bar_p = os.path.join(d, 's1', 'bar')
                self.assertEquals(foo_p, roslib.stacks.get_stack_dir('foo'))
                self.assertEquals(bar_p, roslib.stacks.get_stack_dir('bar'))

            # - s2/s3/s1
            print('s2/s3/s1')

            paths = [os.path.join(d, p) for p in ['s2', 's3', 's1']]
            os.environ[rospkg.environment.ROS_PACKAGE_PATH] = os.pathsep.join(paths)
            stacks = roslib.stacks.list_stacks()
            self.assert_('foo' in stacks)
            self.assert_('bar' in stacks)

            foo_p = os.path.join(d, 's2', 'foo')
            bar_p = os.path.join(d, 's1', 'bar')
            self.assertEquals(foo_p, roslib.stacks.get_stack_dir('foo'))
            self.assertEquals(bar_p, roslib.stacks.get_stack_dir('bar'))
        finally:
            # restore rpp
            if rpp is not None:
                os.environ[rospkg.environment.ROS_PACKAGE_PATH] = rpp
            else:
                del os.environ[rospkg.environment.ROS_PACKAGE_PATH]

    def test_expand_to_packages_unary(self):
        # test unary
        test_dir = os.path.join(roslib.packages.get_pkg_dir('roslib'), 'test', 'stack_tests_unary')

        env = os.environ.copy()
        env[rospkg.environment.ROS_PACKAGE_PATH] = test_dir

        from roslib.stacks import expand_to_packages
        self.assertEquals((['foo'], []), expand_to_packages(['foo'], env=env))
        self.assertEquals((['foo', 'bar'], []), expand_to_packages(['foo', 'bar'], env=env))

    def test_expand_to_packages(self):
        from roslib.stacks import expand_to_packages
        try:
            # it's possible to accidentally pass in a sequence type
            # like a string and get weird results, so check that we
            # don't
            self.assertEquals(([], []), expand_to_packages('ros'))
            self.fail('expand_to_packages should only take in a list of strings')
        except ValueError:
            pass

        self.assertEquals(([], []), expand_to_packages([]))
        self.assertEquals((['rosmake', 'roslib', 'roslib'], []), expand_to_packages(['rosmake', 'roslib', 'roslib']))
        self.assertEquals(([], ['bogus_one', 'bogus_two']), expand_to_packages(['bogus_one', 'bogus_two']))

        # this test case is no more valid in a package-only world
        # TODO: setup directory tree so that this can be more precisely calculated
        # valid, invalid = expand_to_packages(['ros', 'bogus_one'])
        # self.assertEquals(['bogus_one'], invalid)
        # check = ['rosbuild', 'rosunit', 'roslib']
        # print valid
        # for c in check:
        #     self.assert_(c in valid, "expected [%s] to be in ros expansion"%c)

    def test_get_stack_version(self):
        test_dir = os.path.join(get_test_path(), 'stack_tests', 's1')
        env = os.environ.copy()
        env[rospkg.environment.ROS_PACKAGE_PATH] = test_dir

        # REP 109: stack.xml has precedence over CMakeLists.txt, version is whitespace stripped
        self.assertEquals('1.6.0-manifest', roslib.stacks.get_stack_version('foo', env=env))
        # REP 109: test fallback to CMakeLists.txt version
        self.assertEquals('1.5.0-cmake', roslib.stacks.get_stack_version('bar', env=env))

        if 0:
            test_dir = os.path.join(roslib.packages.get_pkg_dir('roslib'), 'test', 'stack_tests_unary')
            env = os.environ.copy()
            env[rospkg.environment.ROS_PACKAGE_PATH] = test_dir


def get_test_path():
    return os.path.abspath(os.path.dirname(__file__))
