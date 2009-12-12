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
#
# Revision $Id: test_rosparam_command_line_online.py 5710 2009-08-20 03:11:04Z sfkwc $

from __future__ import with_statement

PKG = 'test_rosparam'
NAME = 'test_rosparam_command_line_online'
import roslib; roslib.load_manifest(PKG)

import os
import signal
import sys 
import time
import unittest

import rostest

import cStringIO
from subprocess import Popen, PIPE, check_call, call

from roslib.scriptutil import get_param_server, script_resolve_name

from contextlib import contextmanager
@contextmanager
def fakestdout():
    realstdout = sys.stdout
    fakestdout = cStringIO.StringIO()
    sys.stdout = fakestdout
    yield fakestdout
    sys.stdout = realstdout

def tolist(b):
    return [x.strip() for x in b.getvalue().split('\n') if x.strip()]

class TestRosparam(unittest.TestCase):

    def setUp(self):
        pass

    def _check(self, expected, actual):
        """
        Make sure all elements of expected are present in actual
        """
        for t in expected:
            self.assert_(t in actual)
    def _notcheck(self, not_expected, actual):
        """
        Make sure all elements of not_expected are not present in actual
        """
        for t in not_expected:
            self.failIf(t in actual)
        
    def test_rosparam_list(self):
        from ros import rosparam
        cmd = 'rosparam'

        params = ['/string', '/int', '/float',
                  '/g1/string', '/g1/int', '/g1/float',
                  '/g2/string', '/g2/int', '/g2/float',
                  ]
        l = rosparam.list_params('')
        for t in params:
            self.assert_(t in l)

        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'list'])
            self._check(params, tolist(b))
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'list', '/'])
            self._check(params, tolist(b))
            
        # test with namespace
        g1p = [p for p in params if p.startswith('/g1/')]
        not_g1p = [p for p in params if not p.startswith('/g1/')]
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'list', '/g1'])
            self._check(g1p, tolist(b))
            self._notcheck(not_g1p, tolist(b))            
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'list', '/g1/'])
            self._check(g1p, tolist(b))
            self._notcheck(not_g1p, tolist(b))
        # test with no match        
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'list', '/not/a/namespace/'])
            self.assertEquals([], tolist(b))
            
    def test_rosparam_load(self):
        from ros import rosparam
        import roslib.packages
        f = os.path.join(roslib.packages.get_pkg_dir('test_rosparam'), 'test', 'test.yaml')
        f_ns = os.path.join(roslib.packages.get_pkg_dir('test_rosparam'), 'test', 'test_ns.yaml')
        
        cmd = 'rosparam'
        try:
            rosparam.yamlmain([cmd, 'load'])
            self.fail("command-line arg should have failed")
        except SystemExit, e:
            self.assertNotEquals(0, e.code)
        try:
            rosparam.yamlmain([cmd, 'load', 'fake-file.yaml'])
            self.fail("command-line arg should have failed")
        except SystemExit, e:
            self.assertNotEquals(0, e.code)

        ps = get_param_server()

        # load into top-level
        rosparam.yamlmain([cmd, 'load', f])
        self.assertEquals('bar', ps.getParam('/', '/foo')[2])
        # - make sure it did an overlay, not erase
        self.assertEquals('foo-value', ps.getParam('/', '/string')[2])
        
        rosparam.yamlmain([cmd, 'load', '-v', f])
        self.assertEquals('bar', ps.getParam('/', '/foo')[2])
        
        # load into namespace
        rosparam.yamlmain([cmd, 'load', f, '/rosparam_load/test'])
        self.assertEquals('bar', ps.getParam('/', '/rosparam_load/test/foo')[2])
        rosparam.yamlmain([cmd, 'load', '-v', f, '/rosparam_load/test'])
        self.assertEquals('bar', ps.getParam('/', '/rosparam_load/test/foo')[2])

        # load file with namespace spec in it
        # - load into top-level
        rosparam.yamlmain([cmd, 'load', f_ns])
        self.assertEquals('baz', ps.getParam('/', '/a/b/foo')[2])
        self.assertEquals('bar', ps.getParam('/', '/foo')[2])
        rosparam.yamlmain([cmd, 'load', '-v', f_ns])
        self.assertEquals('baz', ps.getParam('/', '/a/b/foo')[2])        
        
        # load into namespace
        rosparam.yamlmain([cmd, 'load', f_ns, '/rosparam_load/test2'])
        self.assertEquals('baz', ps.getParam('/', '/rosparam_load/test2/a/b/foo')[2])
        rosparam.yamlmain([cmd, 'load', '-v', f_ns, '/rosparam_load/test2'])
        self.assertEquals('baz', ps.getParam('/', '/rosparam_load/test2/a/b/foo')[2])
        
    def test_rosparam_get(self):
        from ros import rosparam
        cmd = 'rosparam'
        try:
            rosparam.yamlmain([cmd, 'get'])
            self.fail("command-line arg should have failed")
        except SystemExit, e:
            self.assertNotEquals(0, e.code)

        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', "string"])
            self.assertEquals('foo-value', b.getvalue().strip())
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', '-p', "string"])
            self.assertEquals('foo-value', b.getvalue().strip())
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', "/string"])
            self.assertEquals('foo-value', b.getvalue().strip())
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', "/g1/string"])
            self.assertEquals('g1-foo-value', b.getvalue().strip())
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', "g1/string"])
            self.assertEquals('g1-foo-value', b.getvalue().strip())
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', "int"])
            self.assertEquals('1', b.getvalue().strip())
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', "/int"])
            self.assertEquals('1', b.getvalue().strip())
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', '-p', "int"])
            self.assertEquals('1', b.getvalue().strip())
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', "/g1/int"])
            self.assertEquals('10', b.getvalue().strip())
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', "g1/int"])
            self.assertEquals('10', b.getvalue().strip())
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', "float"])
            self.assertEquals('1.0', b.getvalue().strip())
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', '-p', "float"])
            self.assertEquals('1.0', b.getvalue().strip())
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', '-p', "g1/float"])
            self.assertEquals('10.0', b.getvalue().strip())
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', "g1"])
            import yaml
            d = yaml.load(b.getvalue())
            self.assertEquals(d['float'], 10.0)
            self.assertEquals(d['int'], 10.0)
            self.assertEquals(d['string'], "g1-foo-value")
            self.assertEquals(set(['float', 'int', 'string']), set(d.keys()))
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', '-p', "g1"])
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'get', '-pv', "g1"])

    def test_rosparam_set(self):
        from ros import rosparam
        cmd = 'rosparam'

        ps = get_param_server()
        with fakestdout() as b:
            rosparam.yamlmain([cmd, 'set', "/rosparam_set/test1", "1"])
            self.assertEquals(1, ps.getParam('/', '/rosparam_set/test1')[2])
        with fakestdout() as b:            
            # -- verbose
            rosparam.yamlmain([cmd, 'set', '-v', "/rosparam_set/test1", "1"])
            self.assertEquals(1, ps.getParam('/', '/rosparam_set/test1')[2])
        with fakestdout() as b:            
            rosparam.yamlmain([cmd, 'set', "rosparam_set/test1", "2"])
            self.assertEquals(2, ps.getParam('/', '/rosparam_set/test1')[2])
            
        with fakestdout() as b:
            # - floats
            rosparam.yamlmain([cmd, 'set', "/rosparam_set/test2", "1.0"])
            self.assertEquals(1., ps.getParam('/', '/rosparam_set/test2')[2])
        with fakestdout() as b:
            # - floats
            rosparam.yamlmain([cmd, 'set', "/rosparam_set/test2", "2.0"])
            self.assertEquals(2., ps.getParam('/', '/rosparam_set/test2')[2])
        with fakestdout() as b:            
            # - booleans
            rosparam.yamlmain([cmd, 'set', "/rosparam_set/testbool", "true"])
            self.assertEquals(True, ps.getParam('/', '/rosparam_set/testbool')[2])
        with fakestdout() as b:            
            # - strings
            rosparam.yamlmain([cmd, 'set', "/rosparam_set/teststr", "hi"])
            self.assertEquals("hi", ps.getParam('/', '/rosparam_set/teststr')[2])
        with fakestdout() as b: 
            # - list
            rosparam.yamlmain([cmd, 'set', "/rosparam_set/testlist", "[1, 2, 3]"])
            self.assertEquals([1, 2, 3], ps.getParam('/', '/rosparam_set/testlist')[2])
        with fakestdout() as b: 
            # - dictionary
            rosparam.yamlmain([cmd, 'set', "/rosparam_set/testdict", "{a: b, c: d}"])
            self.assertEquals('b', ps.getParam('/', '/rosparam_set/testdict/a')[2])
            self.assertEquals('d', ps.getParam('/', '/rosparam_set/testdict/c')[2])
        with fakestdout() as b:             
            #   - empty dictionary should be a noop
            rosparam.yamlmain([cmd, 'set', "set/testdict", "{}"])
            self.assertEquals('b', ps.getParam('/', '/rosparam_set/testdict/a')[2])
            self.assertEquals('d', ps.getParam('/', '/rosparam_set/testdict/c')[2])
        with fakestdout() as b:                         
            #   - this should be an update
            rosparam.yamlmain([cmd, 'set', "/rosparam_set/testdict", "{e: f, g: h}"])
            self.assertEquals('b', ps.getParam('/', '/rosparam_set/testdict/a')[2])
            self.assertEquals('d', ps.getParam('/', '/rosparam_set/testdict/c')[2])
            self.assertEquals('f', ps.getParam('/', '/rosparam_set/testdict/e')[2])
            self.assertEquals('h', ps.getParam('/', '/rosparam_set/testdict/g')[2])
        with fakestdout() as b:                                     
            # -- verbose
            rosparam.yamlmain([cmd, 'set', '-v', "/rosparam_set/testdictverbose", "{e: f, g: h}"])
            self.assertEquals('f', ps.getParam('/', '/rosparam_set/testdictverbose/e')[2])
            self.assertEquals('h', ps.getParam('/', '/rosparam_set/testdictverbose/g')[2])

    def test_rosparam_delete(self):
        from ros import rosparam
        cmd = 'rosparam'
        ps = get_param_server()

        try:
            rosparam.yamlmain([cmd, 'delete'])
        except SystemExit, e:
            self.assertNotEquals(0, e.code)
        try:
            rosparam.yamlmain([cmd, 'delete', 'one', 'two'])
        except SystemExit, e:
            self.assertNotEquals(0, e.code)

        # delete
        ps.setParam('/', '/delete/me', True)
        self.assert_(ps.hasParam('/', '/delete/me')[2])
        rosparam.yamlmain([cmd, 'delete', "/delete/me"])
        self.failIf(ps.hasParam('/', '/delete/me')[2])

        ps.setParam('/', '/delete/me2', True)
        self.assert_(ps.hasParam('/', '/delete/me2')[2])
        rosparam.yamlmain([cmd, 'delete', '-v', "/delete/me2"])
        self.failIf(ps.hasParam('/', '/delete/me2')[2])

    def test_rosparam_dump(self):
        from ros import rosparam
        import roslib.packages
        f = os.path.join(roslib.packages.get_pkg_dir('test_rosparam'), 'test', 'test.yaml')
        f_out = os.path.join(roslib.packages.get_pkg_dir('test_rosparam'), 'test', 'test_dump.yaml')
        
        cmd = 'rosparam'
        ps = get_param_server()

        try:
            rosparam.yamlmain([cmd, 'dump'])
        except SystemExit, e:
            self.assertNotEquals(0, e.code)
        try:
            rosparam.yamlmain([cmd, 'dump', f_out, 'rosparam_dump', 'rosparam_dump2'])
        except SystemExit, e:
            self.assertNotEquals(0, e.code)

        rosparam.yamlmain([cmd, 'load', f, 'rosparam_dump'])
        self.assertEquals('bar', ps.getParam('/', 'rosparam_dump/foo')[2])
        
        rosparam.yamlmain([cmd, 'dump', f_out, 'rosparam_dump'])
        # yaml files should be equal
        import yaml
        with open(f_out) as b:
            with open(f) as b2:
                self.assertEquals(yaml.load(b.read()), yaml.load(b2.read()))

        rosparam.yamlmain([cmd, 'dump', '-v', f_out, 'rosparam_dump'])                
        with open(f_out) as b:
            with open(f) as b2:
                self.assertEquals(yaml.load(b.read()), yaml.load(b2.read()))

    def test_fullusage(self):
        from ros import rosparam
        try:
            rosparam._fullusage()
        except SystemExit: pass
        try:
            rosparam.yamlmain(['rosparam'])
        except SystemExit: pass
        try:
            rosparam.yamlmain(['rosparam', 'invalid'])        
        except SystemExit: pass
        
            
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestRosparam, sys.argv, coverage_packages=['rosparam'])
