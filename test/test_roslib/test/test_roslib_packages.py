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

    
if __name__ == '__main__':
  rostest.unitrun('test_roslib', 'test_packages', RoslibPackagesTest, coverage_packages=['roslib.packages'])

