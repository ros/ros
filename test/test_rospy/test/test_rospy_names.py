#!/usr/bin/env python
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
# Revision $Id$

import roslib; roslib.load_manifest('test_rospy')

import os
import sys
import struct
import unittest
import time

# test rospy.names package
class TestRospyNames(unittest.TestCase):

    def test_scoped_name(self):
        from rospy.exceptions import ROSException
        from rospy.names import scoped_name
        tests = [
            ['/', '/name', 'name'],
            ['/', '/ns/name', 'ns/name'],
            ['/ns', '/ns/name', 'ns/name'],            
            ['/ns/node', '/ns/name', 'name'],            
            ['/ns/', '/ns/name', 'ns/name'],            
            ['/ns/ns2/', '/ns/name', 'name'],            
            ]
        for caller_id, name, v in tests:
            val = scoped_name(caller_id, name)
            self.assertEquals(v, val, "failed on [%s] [%s]: %s"%(caller_id, name, val))

        fail = [
            ['name', '/name'],
            ['~name', '/name'],
            ]

        for caller_id, name in fail:
            try:
                scoped_name(caller_id, name)
                self.fail("should have failed on %s, %s"%(caller_id, name))
            except ROSException: pass
            
    def test_load_mappings(self):
        from rospy.names import get_mappings
        # get_mappings is initialized statically, so can't test anything other than it is empty
        self.assertEquals({}, get_mappings())
        
    def test_canonicalize_name(self):
        from rospy.names import canonicalize_name
        tests = [
            ('', ''),
            ('/', '/'),
            ('foo', 'foo'),          
            ('/foo', '/foo'),          
            ('/foo/', '/foo'),
            ('/foo/bar', '/foo/bar'),
            ('/foo/bar/', '/foo/bar'),
            ('/foo/bar//', '/foo/bar'),
            ('/foo//bar', '/foo/bar'),
            ('//foo/bar', '/foo/bar'),
            ('foo/bar', 'foo/bar'),
            ('foo//bar', 'foo/bar'),
            ('foo/bar/', 'foo/bar'),
            ('/foo/bar', '/foo/bar'),
            ]
        for t, v in tests:
            self.assertEquals(v, canonicalize_name(t))
            
    def test_ANYTYPE(self):
        from rospy.names import TOPIC_ANYTYPE, SERVICE_ANYTYPE
        self.assertEquals("*", TOPIC_ANYTYPE)
        self.assertEquals("*", SERVICE_ANYTYPE)

    def test_resolve_name(self):
        from rospy.names import resolve_name
        # TODO: test with remappings
        tests = [
            ('', '/', '/'),
            ('', None, '/'), #caller_id defaults to /
            ('', '/node', '/'),
            ('', '/ns1/node', '/ns1/'),

            ('foo', '', '/foo'),
            ('foo', None, '/foo'),
            ('foo/', '', '/foo'),
            ('/foo', '', '/foo'),
            ('/foo/', '', '/foo'),
            ('/foo', '/', '/foo'),
            ('/foo', None, '/foo'),
            ('/foo/', '/', '/foo'),
            ('/foo', '/bar', '/foo'),
            ('/foo/', '/bar', '/foo'),

            ('foo', '/ns1/ns2', '/ns1/foo'),
            ('foo', '/ns1/ns2/', '/ns1/foo'),
            ('foo', '/ns1/ns2/ns3/', '/ns1/ns2/foo'),
            ('foo/', '/ns1/ns2', '/ns1/foo'),
            ('/foo', '/ns1/ns2', '/foo'),
            ('foo/bar', '/ns1/ns2', '/ns1/foo/bar'),
            ('foo//bar', '/ns1/ns2', '/ns1/foo/bar'),
            ('foo/bar', '/ns1/ns2/ns3', '/ns1/ns2/foo/bar'),
            ('foo//bar//', '/ns1/ns2/ns3', '/ns1/ns2/foo/bar'),
            
            ('~foo', '/', '/foo'),            
            ('~foo', '/node', '/node/foo'),            
            ('~foo', '/ns1/ns2', '/ns1/ns2/foo'),            
            ('~foo/', '/ns1/ns2', '/ns1/ns2/foo'),            
            ('~foo/bar', '/ns1/ns2', '/ns1/ns2/foo/bar'),

            ]
        remap = False
        for name, caller_id, v in tests:
            self.assertEquals(v, resolve_name(name, caller_id, remap))

    def test_validators(self):
        from rospy.names import empty_or_valid_name, ParameterInvalid
        validator = empty_or_valid_name('param_name')
        tests = [
            ('', '/', ''),
            ('', '/node', ''),
            ('name', '/node', '/name'),            
            ('/name', '/node', '/name'),            
            ('~name', '/node', '/node/name'),
            # unicode tests
            (u'~name', '/node', u'/node/name'),            
            ]
        for name, caller_id, v in tests:
            self.assertEquals(v, validator(name, caller_id))
        invalid = [
            (1, '/node'),            
            (None, '/node'),            
            ]
        for name, caller_id in invalid:
            try:
                validator(name, caller_id)
                self.fail("empty_or_valid_name should have failed on : [%s], [%s]"%(name, caller_id))
            except ParameterInvalid: pass

    def test_valid_name(self):
        # test with resolution
        from rospy.names import valid_name_validator_resolved, valid_name, ParameterInvalid
        validator = valid_name('param_name', True)
        tests = [
            ('name', '/node', '/name'),            
            ('/name', '/node', '/name'),            
            ('~name', '/node', '/node/name'),
            # test unicode
            (u'~name', '/node', u'/node/name'),                        
            ]
        for name, caller_id, v in tests:
            self.assertEquals(v, valid_name_validator_resolved('p', name, caller_id), "failed on %s %s"%(name, caller_id))
            self.assertEquals(v, validator(name, caller_id))

        # kwc: valid_name is currently very soft in the failures it
        # checks as it is targetted at catching parameter
        # misalignment. I would like to make it more strict in the
        # future.
        invalid = [
            (1, '/node'),            
            (None, '/node'),            
            ('localhost:123', '/node'),            
            ('Bob Barker', '/node'),
            # unicode
            (u'Bob Barker', '/node'),            
            ]
        for name, caller_id in invalid:
            try:
                valid_name_validator_resolved('p', name, caller_id)
                self.fail("valid_name_validator_unresolved should have failed on : [%s], [%s]"%(name, caller_id))
            except ParameterInvalid: pass
            try:
                validator(name, caller_id)
                self.fail("valid_name_validator_unresolved should have failed on : [%s], [%s]"%(name, caller_id))
            except ParameterInvalid: pass

        from rospy.names import valid_name_validator_unresolved
        validator = valid_name('param_name', False)
        tests = [
            ('name', '/node', 'name'),            
            ('/name', '/node', '/name'),            
            ('~name', '/node', '~name'),
            # unicode
            (u'~name', '/node', u'~name'),            
            ]
        for name, caller_id, v in tests:
            self.assertEquals(v, valid_name_validator_unresolved('p', name, caller_id), "failed on [%s] [%s]"%(name, caller_id))
            self.assertEquals(v, validator(name, caller_id))
            
        for name, caller_id in invalid:
            try:
                valid_name_validator_unresolved('p', name, caller_id)
                self.fail("valid_name_validator_unresolved should have failed on : [%s], [%s]"%(name, caller_id))
            except ParameterInvalid: pass
            try:
                validator(name, caller_id)
                self.fail("valid_name_validator_unresolved should have failed on : [%s], [%s]"%(name, caller_id))
            except ParameterInvalid: pass

    def test_global_name(self):
        from rospy.names import global_name, ParameterInvalid
        validator = global_name('param_name')
        tests = [
            ('/', '/node', '/'),            
            ('/name', '/node', '/name'),
            # unicode
            (u'/name', '/node', u'/name'),            
            ]
        for name, caller_id, v in tests:
            self.assertEquals(v, validator(name, caller_id))
        invalid = [
            (1, '/node'),            
            (None, '/node'),            
            ('name', '/node'),   
            ('~name', '/node'),            
            ]
        for name, caller_id in invalid:
            try:
                validator(name, caller_id)
                self.fail("global_name should have failed on : [%s], [%s]"%(name, caller_id))
            except ParameterInvalid: pass

    def test_caller_id(self):
        from rospy.names import get_caller_id, _set_caller_id, get_namespace
        try:
            self.assertEquals('/unnamed', get_caller_id())
            self.assertEquals('/', get_namespace())
            _set_caller_id('/foo')
            self.assertEquals('/foo', get_caller_id())
            self.assertEquals('/', get_namespace())
            _set_caller_id('/foo/bar')
            self.assertEquals('/foo/bar', get_caller_id())
            self.assertEquals('/foo/', get_namespace())
        finally:
            _set_caller_id('/unnamed')
        
    def test_valid_type_name(self):
        from rospy.names import valid_type_name, ParameterInvalid
        validator = valid_type_name('param_name')
        tests = [
            ('foo/Type', '/node'),            
            ('*', '/node'),
            # unicode
            (u'foo/Type', '/node'),                        
            ]
        for t, caller_id in tests:
            self.assertEquals(t, validator(t, caller_id))
        invalid = [
            (1, '/node'),            
            (None, '/node'),            
            ('name', '/node'),
            ]
        for t, caller_id in invalid:
            try:
                validator(t, caller_id)
                self.fail("valid_type_name should have failed on : [%s], [%s]"%(t, caller_id))
            except ParameterInvalid: pass
            
if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_rospy', sys.argv[0], TestRospyNames, coverage_packages=['rospy.names'])
