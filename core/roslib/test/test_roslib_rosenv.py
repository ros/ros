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

import roslib.rosenv


class EnvTest(unittest.TestCase):

    def test_get_ros_root(self):
        from roslib.rosenv import get_ros_root
        self.assertEquals(None, get_ros_root(required=False, env={}))
        self.assertEquals(None, get_ros_root(False, {}))
        try:
            get_ros_root(required=True, env={})
            self.fail('get_ros_root should have failed')
        except Exception:
            pass

        env = {'ROS_ROOT': '/fake/path'}
        self.assertEquals('/fake/path', get_ros_root(required=False, env=env))
        try:
            get_ros_root(required=True, env=env)
            self.fail('get_ros_root should have failed')
        except Exception:
            pass

    def test_get_ros_package_path(self):
        from roslib.rosenv import get_ros_package_path
        self.assertEquals(None, get_ros_package_path(required=False, env={}))
        self.assertEquals(None, get_ros_package_path(False, {}))
        try:
            get_ros_package_path(required=True, env={})
            self.fail('get_ros_package_path should have raised')
        except Exception:
            pass
        env = {'ROS_PACKAGE_PATH': ':'}
        self.assertEquals(':', get_ros_package_path(True, env=env))
        self.assertEquals(':', get_ros_package_path(False, env=env))

        # trip-wire tests. Cannot guarantee that ROS_PACKAGE_PATH is set
        # to valid value on test machine, just make sure logic doesn't crash
        self.assertEquals(os.environ.get('ROS_PACKAGE_PATH', None), get_ros_package_path(required=False))

    def test_get_ros_master_uri(self):
        from roslib.rosenv import get_master_uri
        self.assertEquals(None, get_master_uri(required=False, env={}))
        self.assertEquals(None, get_master_uri(False, {}))
        try:
            get_master_uri(required=True, env={})
            self.fail('get_ros_package_path should have raised')
        except Exception:
            pass
        env = {'ROS_MASTER_URI': 'http://localhost:1234'}
        self.assertEquals('http://localhost:1234', get_master_uri(True, env=env))
        self.assertEquals('http://localhost:1234', get_master_uri(False, env=env))

        argv = ['__master:=http://localhost:5678']
        self.assertEquals('http://localhost:5678', get_master_uri(False, env=env, argv=argv))

        try:
            argv = ['__master:=http://localhost:5678:=http://localhost:1234']
            get_master_uri(required=False, env=env, argv=argv)
            self.fail('should have thrown')
        except roslib.rosenv.ROSEnvException:
            pass

        try:
            argv = ['__master:=']
            get_master_uri(False, env=env, argv=argv)
            self.fail('should have thrown')
        except roslib.rosenv.ROSEnvException:
            pass

        # make sure test works with os.environ
        self.assertEquals(os.environ.get('ROS_MASTER_URI', None), get_master_uri(required=False))
