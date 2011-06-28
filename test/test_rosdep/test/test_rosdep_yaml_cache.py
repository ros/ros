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
import roslib; roslib.load_manifest('test_rosdep')

import os
import struct
import sys
import unittest

import rosunit
import rosdep.core


class RosdepYamlCacheTest(unittest.TestCase):
    def test_YamlCache_init(self):
        yc = rosdep.core.YamlCache("rosdep_test_os", "rosdep_test_version")

    def test_YamlCache_get_yaml(self):
        yc = rosdep.core.YamlCache("rosdep_test_os", "rosdep_test_version")
        yaml_dict = yc.get_yaml(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"), "test", "example_rosdep.yaml"))

        yaml_truth = {'other_rosdep_test': 
                      {'rosdep_test_os': 
                       {'other_version': 'foo'}
                       }, 
                      'no_os_rosdep_test': 
                      {'other_os': 'not_useful'
                       }, 
                      'zlib': 
                      {'rosdep_test_os': 'zlib1g-dev', 
                       'debian': 'zlib1g-dev'
                       }, 
                      'rosdep_test': 
                      {'rosdep_test_os': 
                       {'rosdep_test_version': 'librosdep_test1.37-dev', 
                        '9.10': 'librosdep_test1.40-all-dev', 
                        '8.10': 'this is a script\n'
                        }
                       }
                      }
        self.assertEqual(yaml_dict, yaml_truth)

    def test_YamlCache_get_yaml_repeated(self):
        yc = rosdep.core.YamlCache("rosdep_test_os", "rosdep_test_version")
        yaml_dict = yc.get_yaml(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"), "test", "example_rosdep.yaml"))
        yaml_dict2 = yc.get_yaml(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"), "test", "example_rosdep.yaml"))
        self.assertEqual(yaml_dict, yaml_dict2)

        yaml_truth = {'other_rosdep_test': 
                      {'rosdep_test_os': 
                       {'other_version': 'foo'}
                       }, 
                      'no_os_rosdep_test': 
                      {'other_os': 'not_useful'
                       }, 
                      'zlib': 
                      {'rosdep_test_os': 'zlib1g-dev', 
                       'debian': 'zlib1g-dev'
                       }, 
                      'rosdep_test': 
                      {'rosdep_test_os': 
                       {'rosdep_test_version': 'librosdep_test1.37-dev', 
                        '9.10': 'librosdep_test1.40-all-dev', 
                        '8.10': 'this is a script\n'
                        }
                       }
                      }
        self.assertEqual(yaml_dict2, yaml_truth)

    def test_YamlCache_rosstack_depends(self): ## TODO add this in a walled garden so it can be asserted as being equal to something specific
        yc = rosdep.core.YamlCache("rosdep_test_os", "rosdep_test_version")
        depends = yc.get_rosstack_depends("ros")
        depends2 = yc.get_rosstack_depends("ros")
        self.assertEqual(depends, depends2)

    def test_YamlCache_no_file(self):
        yc = rosdep.core.YamlCache("rosdep_test_os", "rosdep_test_version")
        self.assertEqual({}, yc.get_yaml("non_extant_rosdep.yaml"))

    def test_YamlCache_parse_failure(self):
        yc = rosdep.core.YamlCache("rosdep_test_os", "rosdep_test_version")
        self.assertEqual({}, yc.get_yaml(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"), "test", "invalid_rosdep.yaml")))

    def test_YamlCache_get_specific_rosdeps(self):
        yc = rosdep.core.YamlCache("rosdep_test_os", "rosdep_test_version")
        yaml_dict = yc.get_specific_rosdeps(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"), "test", "example_rosdep.yaml"))
        print "yaml_dict", yaml_dict
        yaml_truth = { 
            'zlib': 'zlib1g-dev',
            'rosdep_test':'librosdep_test1.37-dev', 
            }       
        self.assertEqual(yaml_dict, yaml_truth)

    def test_YamlCache_get_specific_rosdeps_repeated(self):
        yc = rosdep.core.YamlCache("rosdep_test_os", "rosdep_test_version")
        yaml_dict = yc.get_specific_rosdeps(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"), "test", "example_rosdep.yaml"))
        yaml_dict2 = yc.get_specific_rosdeps(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"), "test", "example_rosdep.yaml"))
        self.assertEqual(yaml_dict, yaml_dict2)

        yaml_truth = { 
            'zlib': 'zlib1g-dev',
            'rosdep_test':'librosdep_test1.37-dev', 
            }       
        self.assertEqual(yaml_dict2, yaml_truth)

    def test_RosdepLookupPackage_get_os_from_yaml(self):
        yc = rosdep.core.YamlCache("rosdep_test_os", "rosdep_test_version")
        yaml_os_map = {"rosdep_test_os":"one", "other":"two", "three":"three"};
        output = yc.get_os_from_yaml("rosdep_name", yaml_os_map, "source_path")
        self.assertEqual("one", output)

    def test_RosdepLookupPackage_get_version_from_yaml(self):
        yc = rosdep.core.YamlCache("rosdep_test_os", "rosdep_test_version")
        yaml_map = {"8.04":"one", "rosdep_test_version":"two", "three":"three"};
        output = yc.get_version_from_yaml("rosdep_name", yaml_map, "source_path" )
        self.assertEqual("two", output)
        


if __name__ == '__main__':
  rosunit.unitrun('test_rosdep', 'test_yaml_cache', RosdepYamlCacheTest, coverage_packages=['rosdep.core'])  

