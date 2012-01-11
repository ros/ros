#!/usr/bin/env python
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

import os
import sys 
import time
import unittest

import rostest

from subprocess import Popen, PIPE, check_call, call

from rosgraph.names import script_resolve_name

def get_param_server():
    import rosgraph
    return rosgraph.Master('/rosparam')

class TestRosparamOnline(unittest.TestCase):

    def setUp(self):
        self.vals = set()
        self.msgs = {}

    def callback(self, msg, val):
        self.vals.add(val)
        self.msgs[val] = msg
        
    def test_rosparam(self):
        ps = get_param_server()

        # network is initialized
        cmd = 'rosparam'
        names = ['/chatter', 'foo/chatter']

        # list
        params = ['/string', '/int', '/float',
                  '/g1/string', '/g1/int', '/g1/float',
                  '/g2/string', '/g2/int', '/g2/float',
                  ]
        # - we aren't matching against the core services as those can make the test suites brittle
        output = Popen([cmd, 'list'], stdout=PIPE).communicate()[0]
        l = set(output.split())
        for t in params:
            self.assert_(t in l)

        # get
        # - strings
        output = Popen([cmd, 'get', "string"], stdout=PIPE).communicate()[0]
        self.assertEquals('foo-value', output.strip())
        # -- pretty
        output = Popen([cmd, 'get', '-p', "string"], stdout=PIPE).communicate()[0]
        self.assertEquals('foo-value', output.strip())
        output = Popen([cmd, 'get', "/string"], stdout=PIPE).communicate()[0]
        self.assertEquals('foo-value', output.strip())
        output = Popen([cmd, 'get', "g1/string"], stdout=PIPE).communicate()[0]
        self.assertEquals('g1-foo-value', output.strip())
        output = Popen([cmd, 'get', "/g1/string"], stdout=PIPE).communicate()[0]
        self.assertEquals('g1-foo-value', output.strip())
        output = Popen([cmd, 'get', "/g2/string"], stdout=PIPE).communicate()[0]
        self.assertEquals('g2-foo-value', output.strip())
        # - ints
        output = Popen([cmd, 'get', "int"], stdout=PIPE).communicate()[0]
        self.assertEquals('1', output.strip())
        # -- pretty
        output = Popen([cmd, 'get', '-p', "int"], stdout=PIPE).communicate()[0]
        self.assertEquals('1', output.strip())
        output = Popen([cmd, 'get', "/int"], stdout=PIPE).communicate()[0]
        self.assertEquals('1', output.strip())
        output = Popen([cmd, 'get', "g1/int"], stdout=PIPE).communicate()[0]
        self.assertEquals('10', output.strip())
        output = Popen([cmd, 'get', "/g1/int"], stdout=PIPE).communicate()[0]
        self.assertEquals('10', output.strip())
        output = Popen([cmd, 'get', "/g2/int"], stdout=PIPE).communicate()[0]
        self.assertEquals('20', output.strip())
        # - floats
        output = Popen([cmd, 'get', "float"], stdout=PIPE).communicate()[0]
        self.assertEquals('1.0', output.strip())
        # -- pretty
        output = Popen([cmd, 'get', '-p', "float"], stdout=PIPE).communicate()[0]
        self.assertEquals('1.0', output.strip())
        output = Popen([cmd, 'get', "/float"], stdout=PIPE).communicate()[0]
        self.assertEquals('1.0', output.strip())
        output = Popen([cmd, 'get', "g1/float"], stdout=PIPE).communicate()[0]
        self.assertEquals('10.0', output.strip())
        output = Popen([cmd, 'get', "/g1/float"], stdout=PIPE).communicate()[0]
        self.assertEquals('10.0', output.strip())
        output = Popen([cmd, 'get', "/g2/float"], stdout=PIPE).communicate()[0]
        self.assertEquals('20.0', output.strip())
        # - dictionary
        output = Popen([cmd, 'get', "g1"], stdout=PIPE).communicate()[0]
        import yaml
        d = yaml.load(output)
        self.assertEquals(d['float'], 10.0)
        self.assertEquals(d['int'], 10.0)
        self.assertEquals(d['string'], "g1-foo-value")
        self.assertEquals(set(['float', 'int', 'string']), set(d.keys()))

        # -- don't bother parsing pretty output of dictionary, but check for no errors
        check_call([cmd, 'get', '-p', "g1"])
        # --- with verbose
        check_call([cmd, 'get', '-pv', "g1"])
        
        # set
        # - integers
        Popen([cmd, 'set', "/set/test1", "1"], stdout=PIPE).communicate()[0]
        self.assertEquals(1, ps.getParam('/set/test1'))
        # -- verbose
        Popen([cmd, 'set', '-v', "/set/test1", "1"], stdout=PIPE).communicate()[0]
        self.assertEquals(1, ps.getParam('/set/test1'))
        Popen([cmd, 'set', "set/test1", "2"], stdout=PIPE).communicate()[0]
        self.assertEquals(2, ps.getParam('/set/test1'))
        # - floats
        Popen([cmd, 'set', "/set/test2", "1.0"], stdout=PIPE).communicate()[0]
        self.assertEquals(1, ps.getParam('/set/test2'))
        Popen([cmd, 'set', "set/test2", "2.0"], stdout=PIPE).communicate()[0]
        self.assertEquals(2, ps.getParam('/set/test2'))
        # - booleans
        Popen([cmd, 'set', "/set/testbool", "true"], stdout=PIPE).communicate()[0]
        self.assertEquals(True, ps.getParam('/set/testbool'))
        Popen([cmd, 'set', "set/testbool", "false"], stdout=PIPE).communicate()[0]
        self.assertEquals(False, ps.getParam('/set/testbool'))
        # - strings
        #   TODO: test more interesting encodings, like multi-line
        Popen([cmd, 'set', "/set/teststr", "hi"], stdout=PIPE).communicate()[0]
        self.assertEquals("hi", ps.getParam('/set/teststr'))
        Popen([cmd, 'set', "set/teststr", "hello world"], stdout=PIPE).communicate()[0]
        self.assertEquals("hello world", ps.getParam('/set/teststr'))
        Popen([cmd, 'set', "set/teststr", "'true'"], stdout=PIPE).communicate()[0]
        self.assertEquals("true", ps.getParam('/set/teststr'))
        # - list
        Popen([cmd, 'set', "set/testlist", "[]"], stdout=PIPE).communicate()[0]
        self.assertEquals([], ps.getParam('/set/testlist'))
        Popen([cmd, 'set', "/set/testlist", "[1, 2, 3]"], stdout=PIPE).communicate()[0]
        self.assertEquals([1, 2, 3], ps.getParam('/set/testlist'))
        # - dictionary
        Popen([cmd, 'set', "/set/testdict", "{a: b, c: d}"], stdout=PIPE).communicate()[0]
        self.assertEquals('b', ps.getParam('/set/testdict/a'))
        self.assertEquals('d', ps.getParam('/set/testdict/c'))
        #   - empty dictionary should be a noop
        Popen([cmd, 'set', "set/testdict", "{}"], stdout=PIPE).communicate()[0]
        self.assertEquals('b', ps.getParam('/set/testdict/a'))
        self.assertEquals('d', ps.getParam('/set/testdict/c'))
        #   - this should be an update
        Popen([cmd, 'set', "/set/testdict", "{e: f, g: h}"], stdout=PIPE).communicate()[0]
        self.assertEquals('b', ps.getParam('/set/testdict/a'))
        self.assertEquals('d', ps.getParam('/set/testdict/c'))
        self.assertEquals('f', ps.getParam('/set/testdict/e'))
        self.assertEquals('h', ps.getParam('/set/testdict/g'))
        # -- verbose
        check_call([cmd, 'set', '-v', "/set/testdictverbose", "{e: f, g: h}"])
        
        # delete
        ps.setParam('/delete/me', True)
        self.assert_(ps.hasParam('/delete/me'))
        Popen([cmd, 'delete', "/delete/me"], stdout=PIPE).communicate()[0]
        self.failIf(ps.hasParam('/delete/me'))

        # TODO: dump
        # TODO: load
            
PKG = 'test_rosparam'
NAME = 'test_rosparam_command_line_online'
if __name__ == '__main__':
    rostest.run(PKG, NAME, TestRosparamOnline, sys.argv)
