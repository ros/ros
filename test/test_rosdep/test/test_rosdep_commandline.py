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
import subprocess
import shutil
import tempfile

import rosunit
import rosdep.core


class RosdepCommandlineTest(unittest.TestCase):
    """ Basic Tripwire Testing"""



    def test_Rosdep_commandline_satisfy(self):
        self.assertEqual(0,subprocess.call(["rosdep", "satisfy", "test_rosdep"]))
        self.assertEqual(0,subprocess.call(["rosdep", "generate_bash", "test_rosdep"]))
    def test_Rosdep_commandline_satisfy_y(self):
        self.assertEqual(0,subprocess.call(["rosdep", "satisfy", "-y", "test_rosdep", "rosdep"]))
    def test_Rosdep_commandline_satisfy_multiple(self):
        self.assertEqual(0,subprocess.call(["rosdep", "satisfy", "test_rosdep", "rosdep"]))
    def test_Rosdep_commandline_check(self):
        self.assertEqual(0,subprocess.call(["rosdep", "check", "test_rosdep"]))
    def test_Rosdep_commandline_what_needs(self):
        self.assertEqual(0,subprocess.call(["rosdep", "what_needs", "boost"]))
    def test_Rosdep_commandline_where_defined(self):
        self.assertEqual(0,subprocess.call(["rosdep", "where_defined", "boost"]))


class RosdepCommandlineExternalPackages(unittest.TestCase):

    def setUp(self):
        self.tempdir = tempfile.mkdtemp()
        shutil.copytree(os.path.join(roslib.packages.get_pkg_dir('test_rosdep'),
                                     'embedded_test_packages'), 
                        self.tempdir+"/packges")
        
        self.env = os.environ
        self.env['ROS_PACKAGE_PATH'] = self.tempdir+":"+self.env['ROS_PACKAGE_PATH']
        self.env['ROSDEP_DEBUG']='true'            

    def tearDown(self):
        shutil.rmtree(self.tempdir)

    def test_check_legacy_apt(self):
        my_env = self.env.copy()
        my_env['ROS_OS_OVERRIDE']='ubuntu:lucid'
        self.assertEqual(0,subprocess.call(["rosdep", "check", "rosdep_legacy_apt"], env=my_env))
        self.assertEqual(0,subprocess.call(["rosdep", "satisfy", "rosdep_legacy_apt"], env=my_env))
        self.assertEqual(0,subprocess.call(["rosdep", "install", "rosdep_legacy_apt"], env=my_env))


    def test_apt(self):
        my_env = self.env.copy()
        my_env['ROS_OS_OVERRIDE']='ubuntu:lucid'
        self.assertEqual(0,subprocess.call(["rosdep", "check", "rosdeptest"], env=my_env))
        self.assertEqual(0,subprocess.call(["rosdep", "satisfy", "rosdeptest"], env=my_env))
        self.assertEqual(0,subprocess.call(["rosdep", "install", "rosdeptest"], env=my_env))

    def REMOVED_UNTIL_PIP_IN_DEFAULT_test_pip(self):
        my_env = self.env.copy()
        my_env['ROS_OS_OVERRIDE']='ubuntu:lucid'
        self.assertEqual(0,subprocess.call(["rosdep", "check", "rosdep_pip_test"], env=my_env))
        self.assertEqual(0,subprocess.call(["rosdep", "satisfy", "rosdep_pip_test"], env=my_env))
        self.assertEqual(0,subprocess.call(["rosdep", "install", "rosdep_pip_test"], env=my_env))

    def test_check_test_missing(self):
        my_env = self.env.copy()
        my_env['ROS_OS_OVERRIDE']='ubuntu:lucid'
        self.assertEqual(1,subprocess.call(["rosdep", "check", "rosdep_test_missing"], env=my_env))
        self.assertEqual(1,subprocess.call(["rosdep", "satisfy", "rosdep_test_missing"], env=my_env))
        self.assertEqual(1,subprocess.call(["rosdep", "install", "rosdep_test_missing"], env=my_env))

    def test_source(self):
        my_env = self.env.copy()
        my_env['ROS_OS_OVERRIDE']='ubuntu:lucid'
        try:
            os.remove('/tmp/test_sourcedep_installed')
        except:
            pass #it's ok if the file's not there
        self.assertEqual(1,subprocess.call(["rosdep", "check", "rosdep_source"], env=my_env))
        self.assertEqual(1,subprocess.call(["rosdep", "satisfy", "rosdep_source"], env=my_env))
        self.assertEqual(0,subprocess.call(["rosdep", "install", "rosdep_source"], env=my_env)) # install first it touches a file the others detect
        self.assertEqual(0,subprocess.call(["rosdep", "check", "rosdep_source"], env=my_env))
        self.assertEqual(0,subprocess.call(["rosdep", "satisfy", "rosdep_source"], env=my_env))


    def test_md5_source(self):
        my_env = self.env.copy()
        my_env['ROS_OS_OVERRIDE']='ubuntu:lucid'
        try:
            os.remove('/tmp/test_sourcedep_installed')
        except:
            pass #it's ok if the file's not there
        self.assertEqual(1,subprocess.call(["rosdep", "check", "rosdep_md5_source"], env=my_env))
        self.assertEqual(1,subprocess.call(["rosdep", "satisfy", "rosdep_md5_source"], env=my_env))
        self.assertEqual(0,subprocess.call(["rosdep", "install", "rosdep_md5_source"], env=my_env)) # install first it touches a file the others detect
        self.assertEqual(0,subprocess.call(["rosdep", "check", "rosdep_md5_source"], env=my_env))
        self.assertEqual(0,subprocess.call(["rosdep", "satisfy", "rosdep_md5_source"], env=my_env))

    def test_invalid_md5_source(self):
        my_env = self.env.copy()
        my_env['ROS_OS_OVERRIDE']='ubuntu:lucid'
        try:
            os.remove('/tmp/test_sourcedep_installed')
        except:
            pass #it's ok if the file's not there
        self.assertEqual(1,subprocess.call(["rosdep", "check", "rosdep_invalid_md5_source"], env=my_env))
        self.assertEqual(1,subprocess.call(["rosdep", "satisfy", "rosdep_invalid_md5_source"], env=my_env))
        self.assertEqual(1,subprocess.call(["rosdep", "install", "rosdep_invalid_md5_source"], env=my_env)) 


if __name__ == '__main__':
  rosunit.unitrun('test_rosdep', 'test_commandline', RosdepCommandlineTest, coverage_packages=['rosdep.commandline'])  
  rosunit.unitrun('test_rosdep', 'test_commandline', RosdepCommandlineExternalPackages, coverage_packages=['rosdep.commandline'])  

