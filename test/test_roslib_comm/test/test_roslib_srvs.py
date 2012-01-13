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
import unittest

import roslib.packages

class RoslibSrvTest(unittest.TestCase):
  
    def test_SrvSpec(self):
        from roslib.msgs import MsgSpec
        from roslib.srvs import SrvSpec

        types = ['int32']
        names = ['a']
        constants = []
        text = 'int32 a'
        msg_a = MsgSpec(types, names, constants, text)

        types = ['int64']
        names = ['b']
        constants = []
        text = 'int64 b'
        msg_b = MsgSpec(types, names, constants, text)

        text = msg_a.text + '\n---\n' + msg_b.text
        spec = SrvSpec(msg_a, msg_b, text)
        self.assertEquals(msg_a, spec.request)
        self.assertEquals(msg_b, spec.response)
        self.assertEquals(text, spec.text)
        self.assertEquals('', spec.full_name)
        self.assertEquals('', spec.short_name)
        self.assertEquals('',spec.package)
        
        # tripwire
        self.assert_(repr(spec))
        self.assert_(str(spec))

        # exercise eq
        self.assertNotEquals(spec, 'spec')
        self.assert_(spec != 'spec')
        
        spec2 = SrvSpec(msg_a, msg_b, text)
        self.assertEquals(spec, spec2)
        self.failIf(spec != spec2)
        
        # - full_name
        spec2.full_name = 'something'
        self.assertNotEquals(spec, spec2)        
        spec2.full_name = ''        
        self.assertEquals(spec, spec2)
        # - short_name
        spec2.short_name = 'something'
        self.assertNotEquals(spec, spec2)        
        spec2.short_name = ''        
        self.assertEquals(spec, spec2)
        # - package
        spec2.package = 'something'
        self.assertNotEquals(spec, spec2)        
        spec2.package = ''        
        self.assertEquals(spec, spec2)
        
    def test_srv_file(self):
        from roslib.srvs import srv_file
        
        d = roslib.packages.get_pkg_dir('test_ros')
        filename = os.path.join(d, 'srv', 'AddTwoInts.srv')
        with open(filename, 'r') as f:
            text = f.read()

        self.assertEquals(filename, srv_file('test_ros', 'AddTwoInts'))
        
    def test_load_from_file(self):
        from roslib.srvs import load_from_file, set_verbose
        
        d = roslib.packages.get_pkg_dir('test_ros')
        filename = os.path.join(d, 'srv', 'AddTwoInts.srv')
        with open(filename, 'r') as f:
            text = f.read()
        
        name, spec = load_from_file(filename)
        self.assertEquals('AddTwoInts', name)
        self.assertEquals(['int64', 'int64'], spec.request.types)
        self.assertEquals(['a', 'b'], spec.request.names)
        self.assertEquals(text, spec.text)
        
        name2, spec2 = load_from_file(filename, package_context='foo')
        self.assertEquals('foo/AddTwoInts', name2)
        name2, spec2 = load_from_file(filename, package_context='foo/')
        self.assertEquals('foo/AddTwoInts', name2)
        name2, spec2 = load_from_file(filename, package_context='foo//')
        self.assertEquals('foo/AddTwoInts', name2)

        # test with verbose on
        set_verbose(True)
        name3, spec3 = load_from_file(filename)
        self.assertEquals(name, name3)
        self.assertEquals(spec, spec3)        

