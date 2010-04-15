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

import rostest

class RoslibScriptutilTest(unittest.TestCase):
  
    def test_myargv(self):
        orig_argv = sys.argv
        try:
            from roslib.scriptutil import myargv
            args = myargv()
            self.assertEquals(args, sys.argv)
            self.assertEquals(['foo', 'bar', 'baz'], myargv(['foo','bar', 'baz']))
            self.assertEquals(['-foo', 'bar', '-baz'], myargv(['-foo','bar', '-baz']))
            
            self.assertEquals(['foo'], myargv(['foo','bar:=baz']))
            self.assertEquals(['foo'], myargv(['foo','-bar:=baz']))
        finally:
            sys.argv = orig_argv

    def test_interactive(self):
        import roslib.scriptutil
        self.failIf(roslib.scriptutil.is_interactive(), "interactive should be false by default")
        for v in [True, False]:
            roslib.scriptutil.set_interactive(v)        
            self.assertEquals(v, roslib.scriptutil.is_interactive())
        
if __name__ == '__main__':
    rostest.unitrun('test_roslib', 'test_scriptutil', RoslibScriptutilTest, coverage_packages=['roslib.scriptutil'])

