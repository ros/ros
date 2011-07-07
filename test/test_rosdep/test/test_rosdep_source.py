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
import rosdep.installers

class RosdepSourceTest(unittest.TestCase):
    
    def test_aptinstaller_installed(self):
        args = {}
        args["packages"] = "libc6 gcc"
        ai = rosdep.installers.AptInstaller(args)
        self.assertTrue(ai.check_presence())
        ## Requires sudo self.assertTrue(ai.generate_package_install_command())
        
    def test_aptinstaller_not_installed(self):
        args = {}
        args["packages"] = "not-a-package"
        ai = rosdep.installers.AptInstaller(args)
        self.assertFalse(ai.check_presence())
        ## Requres sudo self.assertFalse(ai.generate_package_install_command())


    def test_sourceinstaller(self):
        args = {}
        args["uri"] = 'https://kforge.ros.org/rosrelease/viewvc/sourcedeps/test_sourcedep/test_sourcedep-0.0.rdmanifest'
        ai = rosdep.installers.SourceInstaller(args)
        self.assertFalse(ai.check_presence())
        self.assertTrue(ai.generate_package_install_command())

        

if __name__ == '__main__':
  os.environ["ROSDEP_TEST_OS"] = "rosdep_test_os"
  rosunit.unitrun('test_rosdep', 'test_source', RosdepSourceTest, coverage_packages=['rosdep.core'])  

