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

import roslib.rosenv
import rostest

class EnvTest(unittest.TestCase):
  
  def test_get_ros_root(self):
    from roslib.rosenv import get_ros_root
    self.assertEquals(None, get_ros_root(required=False, environ={}))
    self.assertEquals(None, get_ros_root(False, {}))    
    try:
      get_ros_root(required=True, environ={})
      self.fail("get_ros_root should have failed")
    except: pass

    env = {'ROS_ROOT': '/fake/path'}
    self.assertEquals('/fake/path', get_ros_root(required=False, environ=env))
    try:
      get_ros_root(required=True, environ=env)
      self.fail("get_ros_root should have failed")
    except: pass

    real_ros_root = get_ros_root(required=True)
    
    # make sure that ros root is a directory
    p = os.path.join(real_ros_root, 'Makefile')
    env = {'ROS_ROOT': p}
    self.assertEquals(p, get_ros_root(required=False, environ=env))
    try:
      get_ros_root(required=True, environ=env)
      self.fail("get_ros_root should have failed")
    except: pass
    
  def test_get_ros_package_path(self):
    from roslib.rosenv import get_ros_package_path
    self.assertEquals(None, get_ros_package_path(required=False, environ={}))
    self.assertEquals(None, get_ros_package_path(False, {}))
    try:
      get_ros_package_path(required=True, environ={})
      self.fail("get_ros_package_path should have raised")
    except: pass
    env = {'ROS_PACKAGE_PATH': ':'}
    self.assertEquals(':', get_ros_package_path(True, environ=env))
    self.assertEquals(':', get_ros_package_path(False, environ=env))    

  def test_get_ros_master_uri(self):
    from roslib.rosenv import get_master_uri
    self.assertEquals(None, get_master_uri(required=False, environ={}))
    self.assertEquals(None, get_master_uri(False, {}))
    try:
      get_master_uri(required=True, environ={})
      self.fail("get_ros_package_path should have raised")
    except: pass
    env = {'ROS_MASTER_URI': 'http://localhost:1234'}
    self.assertEquals('http://localhost:1234', get_master_uri(True, environ=env))
    self.assertEquals('http://localhost:1234', get_master_uri(False, environ=env))    


  def test_setup_default_environment(self):
    from roslib.rosenv import setup_default_environment
    # can't test actual functionality as coded, but verify that it is a noop on non rosdeb systems
    if os.path.isdir('/usr/lib/ros'):
      return
    e = os.environ.copy()
    setup_default_environment()
    self.assertEquals(e, os.environ)
    
  def test_get_log_dir(self):
    from roslib.roslogging import get_log_dir
    from roslib.rosenv import get_ros_root
    import tempfile, os
    base = tempfile.gettempdir()
    ros_log_dir = os.path.join(base, 'ros_log_dir')
    ros_home_dir = os.path.join(base, 'ros_home_dir')
    home_dir = os.path.expanduser('~')

    # ROS_LOG_DIR has precedence
    env = {'ROS_ROOT': get_ros_root(), 'ROS_LOG_DIR': ros_log_dir, 'ROS_HOME': ros_home_dir }
    self.assertEquals(ros_log_dir, get_log_dir(environ=env))

    env = {'ROS_ROOT': get_ros_root(), 'ROS_HOME': ros_home_dir }
    self.assertEquals(os.path.join(ros_home_dir, 'log'), get_log_dir(environ=env))

    env = {'ROS_ROOT': get_ros_root()}
    self.assertEquals(os.path.join(home_dir, '.ros', 'log'), get_log_dir(environ=env))

    # test default assignment of env. Don't both checking return value as we would duplicate get_log_dir
    self.assert_(get_log_dir() is not None)

  def test_get_test_results_dir(self):
    from roslib.rosenv import get_ros_root, get_test_results_dir
    import tempfile, os
    base = tempfile.gettempdir()
    ros_test_results_dir = os.path.join(base, 'ros_test_results_dir')
    ros_home_dir = os.path.join(base, 'ros_home_dir')
    home_dir = os.path.expanduser('~')

    # ROS_TEST_RESULTS_DIR has precedence
    env = {'ROS_ROOT': get_ros_root(), 'ROS_TEST_RESULTS_DIR': ros_test_results_dir, 'ROS_HOME': ros_home_dir }
    self.assertEquals(ros_test_results_dir, get_test_results_dir(environ=env))

    env = {'ROS_ROOT': get_ros_root(), 'ROS_HOME': ros_home_dir }
    self.assertEquals(os.path.join(ros_home_dir, 'test_results'), get_test_results_dir(environ=env))

    env = {'ROS_ROOT': get_ros_root()}
    self.assertEquals(os.path.join(home_dir, '.ros', 'test_results'), get_test_results_dir(environ=env))

    # test default assignment of env. Don't both checking return value as we would duplicate get_test_results_dir
    self.assert_(get_test_results_dir() is not None)

  def test_get_ros_home(self):
    from roslib.rosenv import get_ros_root, get_ros_home
    import tempfile, os
    base = tempfile.gettempdir()
    ros_home_dir = os.path.join(base, 'ros_home_dir')
    home_dir = os.path.expanduser('~')

    # ROS_HOME has precedence
    env = {'ROS_ROOT': get_ros_root(), 'ROS_HOME': ros_home_dir }
    self.assertEquals(ros_home_dir, get_ros_home(environ=env))

    env = {'ROS_ROOT': get_ros_root()}
    self.assertEquals(os.path.join(home_dir, '.ros'), get_ros_home(environ=env))

    # test default assignment of env. Don't both checking return value 
    self.assert_(get_ros_home() is not None)
    
if __name__ == '__main__':
  rostest.unitrun('test_roslib', 'test_env', EnvTest, coverage_packages=['roslib.rosenv'])

