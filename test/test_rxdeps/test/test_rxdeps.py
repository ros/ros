# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#
# Author: Brian Gerkey/Ken Conley

from __future__ import with_statement

PKG = 'test_rxdeps'
import roslib; roslib.load_manifest(PKG)

import os
import unittest
import tempfile
import shutil
from subprocess import Popen, PIPE

ROS_ROOT = 'ROS_ROOT'
ROS_PACKAGE_PATH = 'ROS_PACKAGE_PATH'
ROS_LANG_DISABLE = 'ROS_LANG_DISABLE'

## Process-level tests of rxdeps executable
class RxdepsTestCase(unittest.TestCase):
    
    ## runs rxdeps with ROS_ROOT set to ./test and ROS_PACKAGE_PATH unset
    ## @return int, str: return code, stdout
    def _run_rxdeps(self, ros_package_path, pkgname, command):
        env = os.environ.copy()
        if ros_package_path is not None:
            env[ROS_PACKAGE_PATH] = ros_package_path

        # Must split up the command string into its whitespace separated
        # components; otherwise you get multiple words as one element of
        # argv.
        #args = ["rxdeps", command, pkgname]
        args = ["rxdeps"]
        if command:
          for s in command.split():
            args.append(s)
        if pkgname is not None:
          args.append("--target=%s"%pkgname)
        p = Popen(args, stdout=PIPE, stderr=PIPE, env=env)
        stdout, stderr = p.communicate()

        # Also test command aliases, verifying that they give the same 
        # return code and console output
        if command:
            aliases = {}
            cmd = command.split()[-1] 
            if cmd in aliases:
                args[-2] = aliases[cmd]
                alias_p = Popen(args, stdout=PIPE, stderr=PIPE, env=env)
                alias_stdout, alias_stderr = alias_p.communicate()
                self.assertEquals(p.returncode, alias_p.returncode)
                self.assertEquals(stdout, alias_stdout)
                #self.assertEquals(stderr, alias_stderr)

        return p.returncode, stdout.strip(), stderr


    def test_utest(self):
        ret, out, err = self._run_rxdeps(os.path.join(roslib.packages.get_pkg_dir("test_rxdeps"),"test/test_packages"), "pkg1", "--graphviz-output=deps.gv")
        self.assertTrue(ret == 0)
        #print ret, out, err
        with open("deps.gv") as fh:
            lines = fh.read().split("\n")
            self.assertTrue("  \"pkg2\" -> \"pkg1\";" in lines)
            self.assertTrue("  \"pkg3\" -> \"pkg2\";" in lines)
            self.assertTrue("  \"pkg4\" -> \"pkg2\";" in lines)
            self.assertTrue("  \"pkg5\" -> \"pkg3\";" in lines)
            self.assertTrue("  \"pkg5\" -> \"pkg4\";" in lines)

        # make sure the intermediate is cleaned up
        os.remove("deps.gv")

        # clean up the output too
        self.assertTrue(os.path.exists("deps.pdf"))
        os.remove("deps.pdf")


    def test_output_arg(self):
        ret, out, err = self._run_rxdeps(os.path.join(roslib.packages.get_pkg_dir("test_rxdeps"),"test/test_packages"), "pkg1",  "--graphviz-output=deps.gv -oout.pdf")
        self.assertTrue(ret == 0)
        #print ret, out, err
        with open("deps.gv") as fh:
            lines = fh.read().split("\n")
            self.assertTrue("  \"pkg2\" -> \"pkg1\";" in lines)
            self.assertTrue("  \"pkg3\" -> \"pkg2\";" in lines)
            self.assertTrue("  \"pkg4\" -> \"pkg2\";" in lines)
            self.assertTrue("  \"pkg5\" -> \"pkg3\";" in lines)
            self.assertTrue("  \"pkg5\" -> \"pkg4\";" in lines)
            

        # make sure the intermediate is cleaned up
        os.remove("deps.gv")

        # clean up the output too
        self.assertTrue(os.path.exists("out.pdf"))
        os.remove("out.pdf")


if __name__ == "__main__":
    import rostest
    rostest.unitrun(PKG, 'rxdeps_exe_process', RxdepsTestCase)
