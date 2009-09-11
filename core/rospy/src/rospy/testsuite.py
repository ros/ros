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
from core import *
from names import *
from names import _set_caller_id #private import
from rosutil import *

import unittest

class NamingTestCase(unittest.TestCase):
    
    def testResolveName(self):
        _set_caller_id('/callerId')
        tests = [
            #private cases
            ('~flow', '/foo', '/foo/flow'), ('~flow', '/foo/bar', '/foo/bar/flow'),
            ('~flow', '/foo/', '/foo/flow'), ('~flow/', '/foo/bar/', '/foo/bar/flow'),
            ('~flow', '/foo/bar/baz', '/foo/bar/baz/flow'), ('~flow', None, '/callerId/flow'),
            ('~flow', '/', '/flow'), ('~flow/child', '/foo/bar', '/foo/bar/flow/child'),
            #relative cases
            ('bar/flow', '/foo/', '/bar/flow'), ('baz/flow', '/foo/bar', '/foo/baz/flow'),
            ('bar/flow/', '/foo', '/bar/flow'), ('baz/flow/', '/foo/bar', '/foo/baz/flow'),
            ('bar/flow', '/', '/bar/flow'), ('blah/baz/flow', '/foo/bar/', '/foo/blah/baz/flow'),
            #global cases
            ('/bar', '/foo', '/bar'), ('/baz/flow', '/foo/bar', '/baz/flow'),
            ('/bar/', '/foo/', '/bar'), ('/baz/flow/', '/foo/bar', '/baz/flow'),
            ('/', '/foo', '/'), ('/', '/foo/bar', '/'),
            ]
        for (name, callerId, v) in tests:
            try:
                val = resolve_name(name, callerId)
                assert val == v, "Test [%s, %s]: returned[%s] != true[%s]"%(name, callerId, val, v)
            except ParameterInvalid:
                self.fail("test [%s] raised ParameterInvalid for "%t)

    def testNameContext(self):
        tests = [
            ('/foo', '/'), ('/foo/bar', '/foo/'), ('/a/b', '/a/'), ('/a/b/c', '/a/b/'), ('/', '/')
            ]
        for (t, v) in tests:
            try:
                assert namespace(t) == v, "Test [%s] != %s"%(t, v)
            except ParameterInvalid:
                self.fail("test [%s] raised ParameterInvalid for "%t)
        
        fails = [ None]
        for f in fails:
            try:
                namespace(f)
                self.fail("test [%s] did not fail"%f)
            except ParameterInvalid:
                pass
            
    def testLookup(self):
        d = {
            '/alpha' : 1,
            '/beta/alpha' : 2,
            '/charlie/beta/alpha' : 3,
            '/delta/charlie/beta/alpha' : 3,
            
            '/one/val' : 1,
            '/one/two/val' : 2,
            '/one/two/three/val' : 3,
            '/one/two/three/four/val' : 4,

            '/foo/baz' : 5,
            '/foo/bar/baz' : 6,

            '/toplevel': 100,

            '/cross1/illegal1': 400,
            '/cross2/illegal2': 401,
            }
        for (k, v) in d.iteritems():
            try:
                # k[1:] strips global quantifier as global lookups are illegal
                assert scoped_lookup('/node', k[1:], d) == v, "Context-less case failed for %s"%k
            except Exception, e:
                self.fail("Context-less case failed for %s: %s"%(k, e))
        tests = [            
            ('/beta', 'alpha', 2),
            ('/charlie/beta', 'alpha', 3),
            ('/charlie', 'beta/alpha', 3),

            ('/foo/bar/blah/', 'baz', 6),
            ('/foo/bar', 'baz', 6),
            ('/foo', 'baz', 5),

            ('/one/two/three', 'val', 3),
            ('/one/two/three/four/', 'val', 4), 
            ('/one/two/three/four/five', 'val', 4), 
            ('/one/two/three/four/five/six/', 'val', 4),
            ('/one/two/three/four/five/six/', '/one/two/three/val', 3),
            ('/one/two/three/four/five/six/', '/one/two/three/four/val', 4),
            
            ('/one', '/toplevel', 100), #for now, master should allow global names if its in-scope
            ('/one', 'toplevel', 100),
            ('/one/two', 'toplevel', 100),
            ('/one/two/three', 'toplevel', 100),

            ('/cross1', 'illegal1', 400),
            ('/cross2', 'illegal2', 401),
            ]
        for (ns, key, check) in tests:
            v = scoped_lookup(nsJoin(ns, 'node'), key, d) 
            assert v == check, "Test case failed: %s, %s, %s != %s"%(ns, key, check, v)
            
        fails = [
            ('/beta/alpha', ''),
            ('/alpha', ''),
            ('/alpha/', ''),
            ('/', 'illegal1'),
            ('/cross2', 'illegal1'),
            ('/cross2/', 'illegal1'),
            ('/cross2/', 'cross1/illegal1'),            
            ('/cross2/', '/cross1/illegal1'),            
            ('/', 'baz'),
            ]
        for (ns, key) in fails:
            try:
                scoped_lookup(nsJoin(ns, 'node'), key, d) #should not be present
                self.fail("Lookup should have failed: %s, %s"%(ns, key))
            except KeyError:
                pass
            
if __name__ == '__main__':
    unittest.main()
