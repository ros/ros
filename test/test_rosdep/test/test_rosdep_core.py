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

import rostest
import rosdep.core


class RosdepCoreTest(unittest.TestCase):
    def test_RosdepLookupPackage_get_os_from_yaml(self):
        rdlp = rosdep.core.RosdepLookupPackage("ubuntu", "9.04", "rosdep")
        yaml_os_map = {"ubuntu":"one", "other":"two", "three":"three"};
        output = rdlp.get_os_from_yaml(yaml_os_map)
        self.assertEqual("one", output)

    def test_RosdepLookupPackage_get_version_from_yaml(self):
        rdlp = rosdep.core.RosdepLookupPackage("ubuntu", "9.04", "rosdep")
        yaml_map = {"8.04":"one", "9.04":"two", "three":"three"};
        output = rdlp.get_version_from_yaml(yaml_map)
        self.assertEqual("two", output)
        

    def test_RosdepLookupPackage_parse_yaml_package(self):
        rdlp = rosdep.core.RosdepLookupPackage("ubuntu", "9.04", "test_rosdep")
        yaml_map = rdlp.parse_yaml(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"),"test", "example_rosdep.yaml"))
        rdlp.insert_map(yaml_map, "example_yaml_path", False)
        output = rdlp.lookup_rosdep("rosdep_test")
        self.assertEqual("librosdep_test1.37-dev", output)
        output = rdlp.lookup_rosdep("foobar")
        self.assertEqual(False, output)


    def test_RosdepLookupPackage_parse_yaml_package_override(self):
        rdlp = rosdep.core.RosdepLookupPackage("ubuntu", "9.04", "test_rosdep")
        yaml_map = rdlp.parse_yaml(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"),"test", "example_rosdep.yaml"))
        rdlp.insert_map(yaml_map, "example_yaml_path", False)
        rdlp.insert_map(yaml_map, "example_yaml_path2", True)
        output = rdlp.lookup_rosdep("rosdep_test")
        self.assertEqual("librosdep_test1.37-dev", output)
        output = rdlp.lookup_rosdep("foobar")
        self.assertEqual(False, output)


    def test_RosdepLookupPackage_parse_yaml_package_collision_pass(self):
        rdlp = rosdep.core.RosdepLookupPackage("ubuntu", "9.04", "test_rosdep")
        yaml_map = rdlp.parse_yaml(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"),"test", "example_rosdep.yaml"))
        rdlp.insert_map(yaml_map, "example_yaml_path", False) 
        rdlp.insert_map(yaml_map, "example_yaml_path2", False) 
        output = rdlp.lookup_rosdep("rosdep_test")
        self.assertEqual("librosdep_test1.37-dev", output)
        output = rdlp.lookup_rosdep("foobar")
        self.assertEqual(False, output)


    def test_RosdepLookupPackage_parse_yaml_package_collision_fail(self):
        rdlp = rosdep.core.RosdepLookupPackage("ubuntu", "9.04", "test_rosdep")
        yaml_map = rdlp.parse_yaml(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"),"test", "example_rosdep.yaml"))
        rdlp.insert_map(yaml_map, "example_yaml_path", False) 
        yaml_map = rdlp.parse_yaml(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"),"test", "example_rosdep_conflicting.yaml"))
        self.assertRaises(rosdep.core.RosdepException, rdlp.insert_map, yaml_map, "example_yaml_path2", False)

    def test_RosdepLookupPackage_parse_yaml_package_collision_override(self):
        rdlp = rosdep.core.RosdepLookupPackage("ubuntu", "9.04", "test_rosdep")
        yaml_map = rdlp.parse_yaml(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"),"test", "example_rosdep.yaml"))
        rdlp.insert_map(yaml_map, "example_yaml_path", False)
        yaml_map = rdlp.parse_yaml(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"),"test", "example_rosdep_conflicting.yaml"))
        rdlp.insert_map(yaml_map, "example_yaml_path2", True)
        output = rdlp.lookup_rosdep("rosdep_test")
        self.assertEqual("not-librosdep_test1.37-dev", output)
        output = rdlp.lookup_rosdep("foobar")
        self.assertEqual(False, output)

    def test_RosdepLookupPackage_sources(self):
        rdlp = rosdep.core.RosdepLookupPackage("ubuntu", "9.04", "test_rosdep")

        sources = rdlp.get_sources("rosdep_test")
        self.assertEqual([], sources)

        yaml_map = rdlp.parse_yaml(os.path.join(roslib.packages.get_pkg_dir("test_rosdep"),"test", "example_rosdep.yaml"))
        rdlp.insert_map(yaml_map, "example_yaml_path", False) 
        rdlp.insert_map(yaml_map, "example_yaml_path2", False) 

        sources = rdlp.get_sources("rosdep_test")
        self.assertEqual(["example_yaml_path", "example_yaml_path2"], sources)

        sources = rdlp.get_sources("undefined")
        self.assertEqual([], sources)
        

        
    def test_Rosdep_tripwire(self):
        rdlp = rosdep.core.Rosdep(["rosdep"], "rosdep", robust=True)
        rdlp = rosdep.core.Rosdep(["rosdep"], "rosdep", robust=False)
        
    
if __name__ == '__main__':
  rostest.unitrun('test_rosdep', 'test_core', RosdepCoreTest, coverage_packages=['rosdep.core'])  

