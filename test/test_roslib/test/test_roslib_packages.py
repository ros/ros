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
import roslib.packages
import rostest


class RoslibPackagesTest(unittest.TestCase):
  
  def test_list_pkgs_by_path(self):
    from roslib.packages import list_pkgs_by_path
    # regression test for bug found where list_pkgs_by_path returns empty package name if path is a package and path is a relpath
    d = roslib.packages.get_pkg_dir('test_roslib')

    os.chdir(d)

    self.assertEquals(['test_roslib'], list_pkgs_by_path('.'))
    self.assertEquals(set(['bar', 'foo']), set(list_pkgs_by_path(os.path.join('test', 'package_tests'))))
    self.assertEquals(set(['bar', 'foo']), set(list_pkgs_by_path(os.path.join('test', 'package_tests', 'p1'))))
    self.assertEquals(['foo'], list_pkgs_by_path(os.path.join('test', 'package_tests', 'p1', 'foo')))

    self.assertEquals([], list_pkgs_by_path(os.path.join('bin')))
    

  def test_list_pkgs(self):
    # should be equal to rospack list
    import roslib.rospack
    import roslib.packages    
    pkgs = [s.split()[0] for s in roslib.rospack.rospackexec(['list']).split('\n')]
    retval = roslib.packages.list_pkgs()
    self.assertEquals(set(pkgs), set(retval), set(pkgs) ^ set(retval))
    
    # test twice for caching
    retval = roslib.packages.list_pkgs()
    self.assertEquals(set(pkgs), set(retval), set(pkgs) ^ set(retval))

    # now manipulate the environment to test ordering
    d = roslib.packages.get_pkg_dir('test_roslib')
    d = os.path.join(d, 'test', 'package_tests')
    
    # - p1/p2/p3
    paths = [os.path.join(d, p) for p in ['p1', 'p2', 'p3']]
    cache = {}
    packages = roslib.packages.list_pkgs(pkg_dirs=paths, cache=cache)
    self.assert_('foo' in packages)
    self.assert_('bar' in packages)

    foo_p = os.path.join(d, 'p1', 'foo')
    bar_p = os.path.join(d, 'p1', 'bar')
    self.assertEquals(foo_p, cache['foo'][0])
    self.assertEquals(bar_p, cache['bar'][0])

    # - p2/p3/p1
    paths = [os.path.join(d, p) for p in ['p2', 'p3', 'p1']]
    cache = {}
    packages = roslib.packages.list_pkgs(pkg_dirs=paths, cache=cache)
    self.assert_('foo' in packages)
    self.assert_('bar' in packages)

    foo_p = os.path.join(d, 'p2', 'foo')
    bar_p = os.path.join(d, 'p1', 'bar')
    self.assertEquals(foo_p, cache['foo'][0])
    self.assertEquals(bar_p, cache['bar'][0])
    
  def test_find_node(self):
    import roslib.packages
    d = roslib.packages.get_pkg_dir('test_roslib')
    p = os.path.join(d, 'test', 'fake_node.py')
    self.assertEquals(p, roslib.packages.find_node('test_roslib', 'fake_node.py'))
    
    self.assertEquals(None, roslib.packages.find_node('test_roslib', 'not_a_node'))
    
  def test_get_pkg_dir(self):
    import roslib.packages
    import roslib.rospack
    path = roslib.rospack.rospackexec(['find', 'test_roslib'])
    self.assertEquals(path, roslib.packages.get_pkg_dir('test_roslib'))
    try:
      self.assertEquals(path, roslib.packages.get_pkg_dir('fake_test_roslib'))      
      self.fail("should have raised")
    except roslib.packages.InvalidROSPkgException: pass

  def test_get_dir_pkg(self):
    import roslib.packages
    import roslib.rospack
    path = os.path.normpath(roslib.rospack.rospackexec(['find', 'test_roslib']))

    self.assertEquals((path, 'test_roslib'), roslib.packages.get_dir_pkg(path))
    self.assertEquals((path, 'test_roslib'), roslib.packages.get_dir_pkg(os.path.join(path, 'test')))

    # must fail on parent of test_roslib
    self.assertEquals((None, None), roslib.packages.get_dir_pkg(os.path.dirname(path)))
    
  def test_get_package_paths(self):
    from roslib.packages import get_package_paths

    s = os.pathsep
    tests = [
      ('', []),
      (s, []),
      (s+s+s+s+s, []),
      (s+s+s+s+'/fake/kwc/ros-pkg'+s+s+s, ['/fake/kwc/ros-pkg']),
      (s.join(['/fake/kwc/ros-pkg', '/fake/wg-ros-pkg']), ['/fake/kwc/ros-pkg', '/fake/wg-ros-pkg']),
      ]
    for rpp, v in tests:
      rr = os.getcwd()
      for ros_root_required in [True, False]:
        env = { 'ROS_PACKAGE_PATH' : rpp}
        # test with ros root not set
        if ros_root_required:
          try:
            get_package_paths(ros_root_required, env)
            self.fail("should have failed")
          except roslib.rosenv.ROSEnvException:
            pass
        else:
          paths = get_package_paths(ros_root_required, env)
          self.assertEquals(v, paths)

        # test with ros root set
        env['ROS_ROOT'] = rr
        paths = get_package_paths(ros_root_required, env)
        self.assertEquals(v + [rr], paths)


  def test__platform_supported(self):
    self.assertTrue(roslib.packages._platform_supported(os.path.join(roslib.packages.get_pkg_dir("test_roslib"), "test", "platform_supported.manifest.xml"), "test_os", "test_version"))
    self.assertFalse(roslib.packages._platform_supported(os.path.join(roslib.packages.get_pkg_dir("test_roslib"), "test", "platform_supported.manifest.xml"), "test_os", "not_test_version"))
    self.assertFalse(roslib.packages._platform_supported(os.path.join(roslib.packages.get_pkg_dir("test_roslib"), "test", "platform_supported.manifest.xml"), "not_test_os", "test_version"))
    self.assertFalse(roslib.packages._platform_supported(os.path.join(roslib.packages.get_pkg_dir("test_roslib"), "test", "platform_supported.manifest.xml"), "not_test_os", "not_test_version"))

  def test_platform_supported_tripwire(self):
    self.assertFalse(roslib.packages.platform_supported("test_roslib", "nonextant_os", "noextant_version"))

  def test_current_platform_supported_tripwire(self):
    roslib.packages.current_platform_supported("test_roslib")

    
    
if __name__ == '__main__':
  rostest.unitrun('test_roslib', 'test_packages', RoslibPackagesTest, coverage_packages=['roslib.packages'])

