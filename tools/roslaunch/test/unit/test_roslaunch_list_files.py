# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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
import sys 
import time
import unittest
import roslib.packages

from subprocess import Popen, PIPE, check_call, call

def get_test_path():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'xml'))

class TestListFiles(unittest.TestCase):

    def setUp(self):
        pass

    def test_list_files(self):
        cmd = 'roslaunch'

        # check error behavior
        p = Popen([cmd, '--files'], stdout = PIPE)
        o, e = p.communicate()
        self.assert_(p.returncode != 0, "Should have failed w/o file argument. Code: %d" % (p.returncode))

        d = get_test_path()
        
        p = Popen([cmd, '--files', 'roslaunch', 'test-valid.xml'], stdout = PIPE)
        o, e = p.communicate()
        self.assert_(p.returncode == 0, "Return code nonzero for list files! Code: %d" % (p.returncode))
        self.assertEquals(os.path.realpath(os.path.join(d, 'test-valid.xml')), os.path.realpath(o.strip()))

        print "check 1", o
        
        p = Popen([cmd, '--files', 'roslaunch', 'test-env.xml'], stdout = PIPE)
        o, e = p.communicate()
        self.assert_(p.returncode == 0, "Return code nonzero for list files! Code: %d" % (p.returncode))
        self.assertEquals(set([os.path.realpath(os.path.join(d, 'test-env.xml')), os.path.realpath(os.path.join(d, 'test-env-include.xml'))]),
                          set([os.path.realpath(x.strip()) for x in o.split() if x.strip()]))

        print "check 2", o
