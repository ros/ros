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

class RosdepCoreTest(unittest.TestCase):
    def setUp(self):
        yaml_path = os.path.join(roslib.packages.get_pkg_dir("test_rosdep"), "test", "yaml_script.yaml")
        yc = rosdep.core.YamlCache("rosdep_test_os", "rosdep_test_version")

        self.rdlp = rosdep.core.RosdepLookupPackage("rosdep_test_os", "rosdep_test_version", "test_rosdep", yc)
        yaml_dict = self.rdlp.parse_yaml(yaml_path)
        self.rdlp._insert_map(yaml_dict, yaml_path)

    def test_script_success(self):
        rd = rosdep.core.Rosdep(["rosdep"], "rosdep", robust=True)
        print( "lookup rosdep ->>>", self.rdlp.lookup_rosdep("return0") )
        self.assertTrue(rd.install_rosdep("return0", self.rdlp, True, True))
        
    def test_script_fail(self):
        rd = rosdep.core.Rosdep(["rosdep"], "rosdep", robust=True)
        self.assertFalse(rd.install_rosdep("return1", self.rdlp, True, True))


    
if __name__ == '__main__':
  os.environ["ROSDEP_TEST_OS"] = "rosdep_test_os"
  rosunit.unitrun('test_rosdep', 'test_script', RosdepCoreTest, coverage_packages=['rosdep.core'])  

