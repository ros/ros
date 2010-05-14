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
import sys
import unittest

import roslib.names
import rostest

class NamesTest(unittest.TestCase):
  
  def test_get_ros_namespace(self):
    if 'ROS_NAMESPACE' is os.environ:
      rosns = os.environ['ROS_NAMESPACE']
      del os.environ['ROS_NAMESPACE']
    else:
      rosns = None
    self.assertEquals('/', roslib.names.get_ros_namespace())
    self.assertEquals('/', roslib.names.get_ros_namespace(env={}))

    os.environ['ROS_NAMESPACE'] = 'unresolved'
    self.assertEquals('/unresolved/', roslib.names.get_ros_namespace())
    self.assertEquals('/unresolved/', roslib.names.get_ros_namespace(env={'ROS_NAMESPACE': 'unresolved'}))
    
    os.environ['ROS_NAMESPACE'] = '/resolved/'
    self.assertEquals('/resolved/', roslib.names.get_ros_namespace())
    self.assertEquals('/resolved/', roslib.names.get_ros_namespace(env={'ROS_NAMESPACE': '/resolved'}))
    
    del os.environ['ROS_NAMESPACE']
    
    # restore
    if rosns:
      os.environ['ROS_NAMESPACE'] = rosns

  def test_make_global_ns(self):
    from roslib.names import make_global_ns

    for n in ['~foo']:
      try:
        make_global_ns(n)
        self.fail("make_global_ns should fail on %s"%n)
      except ValueError: pass

    self.assertEquals('/foo/', make_global_ns('foo'))
    self.assertEquals('/', make_global_ns(''))
    self.assertEquals('/foo/', make_global_ns('/foo'))
    self.assertEquals('/foo/', make_global_ns('/foo/'))    
    self.assertEquals('/foo/bar/', make_global_ns('/foo/bar'))
    self.assertEquals('/foo/bar/', make_global_ns('/foo/bar/'))             

  def test_make_caller_id(self):
    from roslib.names import make_caller_id
    if 'ROS_NAMESPACE' is os.environ:
      rosns = os.environ['ROS_NAMESPACE']
      del os.environ['ROS_NAMESPACE']
    else:
      rosns = None

    for n in ['~name']:
      try:
        make_caller_id('~name') # illegal
        self.fail("make_caller_id should fail on %s"%n)
      except ValueError: pass
    
    self.assertEquals('/node/', make_caller_id('node'))
    self.assertEquals('/bar/node/', make_caller_id('bar/node'))
    self.assertEquals('/bar/node/', make_caller_id('/bar/node'))        

    os.environ['ROS_NAMESPACE'] = '/test/'
    self.assertEquals('/test/node/', make_caller_id('node'))
    self.assertEquals('/test/bar/node/', make_caller_id('bar/node'))
    self.assertEquals('/bar/node/', make_caller_id('/bar/node'))        
    
    # restore
    if rosns:
      os.environ['ROS_NAMESPACE'] = rosns

  
  def test_is_global(self):
    try:
      roslib.names.is_global(None)
      self.fail("is_global should raise exception on invalid param")
    except: pass
    tests = ['/', '/global', '/global2']
    for t in tests:
      self.assert_(roslib.names.is_global(t))
    fails = ['', 'not_global', 'not/global']
    for t in fails:
      self.failIf(roslib.names.is_global(t))
    
  def test_is_private(self):
    try:
      roslib.names.is_private(None)
      self.fail("is_private should raise exception on invalid param")
    except: pass
    tests = ['~name', '~name/sub']
    for t in tests:
      self.assert_(roslib.names.is_private(t))
    fails = ['', 'not_private', 'not/private', 'not/~private', '/not/~private']
    for t in fails:
      self.failIf(roslib.names.is_private(t))
      
  def test_namespace(self):
    from roslib.names import namespace
    try:
      namespace(1)
      self.fail("1")
    except TypeError: pass
    try:
      namespace(None)
      self.fail("None")
    except ValueError: pass
    self.assertEquals('/', namespace(''))
    self.assertEquals('/', namespace('/'))
    self.assertEquals('/', namespace('/foo'))
    self.assertEquals('/', namespace('/foo/'))      
    self.assertEquals('/foo/', namespace('/foo/bar'))
    self.assertEquals('/foo/', namespace('/foo/bar/'))      
    self.assertEquals('/foo/bar/', namespace('/foo/bar/baz'))
    self.assertEquals('/foo/bar/', namespace('/foo/bar/baz/'))

    # unicode tests
    self.assertEquals(u'/', namespace(u''))
    self.assertEquals(u'/', namespace(u'/'))    
    self.assertEquals(u'/foo/bar/', namespace(u'/foo/bar/baz/'))

  def test_nsjoin(self):
    from roslib.names import ns_join

    # private and global names cannot be joined
    self.assertEquals('~name', ns_join('/foo', '~name'))
    self.assertEquals('/name', ns_join('/foo', '/name'))
    self.assertEquals('~name', ns_join('~', '~name'))
    self.assertEquals('/name', ns_join('/', '/name'))

    # ns can be '~' or '/'
    self.assertEquals('~name', ns_join('~', 'name'))
    self.assertEquals('/name', ns_join('/', 'name'))

    self.assertEquals('/ns/name', ns_join('/ns', 'name'))
    self.assertEquals('/ns/name', ns_join('/ns/', 'name'))    
    self.assertEquals('/ns/ns2/name', ns_join('/ns', 'ns2/name'))
    self.assertEquals('/ns/ns2/name', ns_join('/ns/', 'ns2/name'))

    # allow ns to be empty
    self.assertEquals('name', ns_join('', 'name'))
    

  def test_load_mappings(self):
    from roslib.names import load_mappings
    self.assertEquals({}, load_mappings([]))
    self.assertEquals({}, load_mappings(['foo']))
    self.assertEquals({}, load_mappings([':=']))
    self.assertEquals({}, load_mappings([':=:=']))
    self.assertEquals({}, load_mappings(['f:=']))
    self.assertEquals({}, load_mappings([':=b']))
    self.assertEquals({}, load_mappings(['foo:=bar:=baz']))
    # should ignore node param assignments
    self.assertEquals({}, load_mappings(['_foo:=bar']))        
    
    self.assertEquals({'foo': 'bar'}, load_mappings(['foo:=bar']))
    # should allow double-underscore names
    self.assertEquals({'__foo': 'bar'}, load_mappings(['__foo:=bar']))
    self.assertEquals({'foo': 'bar'}, load_mappings(['./f', '-x', '--blah', 'foo:=bar']))
    self.assertEquals({'a': '1', 'b': '2', 'c': '3'}, load_mappings(['c:=3', 'c:=', ':=3', 'a:=1', 'b:=2']))

  def test_resource_name(self):
    from roslib.names import resource_name
    self.assertEquals('foo/bar', resource_name('foo', 'bar'))
    self.assertEquals('bar', resource_name('foo', 'bar', my_pkg='foo'))
    self.assertEquals('foo/bar', resource_name('foo', 'bar', my_pkg='bar'))
    self.assertEquals('foo/bar', resource_name('foo', 'bar', my_pkg=''))
    self.assertEquals('foo/bar', resource_name('foo', 'bar', my_pkg=None))        

  def test_resource_name_base(self):
    from roslib.names import resource_name_base
    self.assertEquals('', resource_name_base('')) 
    self.assertEquals('bar', resource_name_base('bar'))    
    self.assertEquals('bar', resource_name_base('foo/bar'))
    self.assertEquals('bar', resource_name_base('/bar'))
    self.assertEquals('', resource_name_base('foo/'))    

  def test_resource_name_package(self):
    from roslib.names import resource_name_package
    self.assertEquals(None, resource_name_package(''))
    self.assertEquals(None, resource_name_package('foo'))    
    self.assertEquals('foo', resource_name_package('foo/'))
    self.assertEquals('foo', resource_name_package('foo/bar'))    

  def test_package_resource_name(self):
    from roslib.names import package_resource_name
    self.assertEquals(('', ''), package_resource_name(''))
    self.assertEquals(('', 'foo'), package_resource_name('foo'))
    self.assertEquals(('foo', 'bar'), package_resource_name('foo/bar'))
    self.assertEquals(('foo', ''), package_resource_name('foo/'))
    try:
      # only allowed single separator
      package_resource_name("foo/bar/baz")
      self.fail("should have raised ValueError")
    except ValueError:
      pass
      

  def test_is_legal_resource_name(self):
    from roslib.names import is_legal_resource_name
    failures = [None, '', 'hello\n', '\t', 'foo++', 'foo-bar', '#foo', 
                ' name', 'name ',
                '~name', '/name',
                '1name', 'foo\\']
    for f in failures:
      self.failIf(is_legal_resource_name(f), f)
    tests = ['f', 'f1', 'f_', 'foo', 'foo_bar', 'foo/bar', 'roslib/Log']
    for t in tests:
      self.assert_(is_legal_resource_name(t), t)

  def test_is_legal_name(self):
    from roslib.names import is_legal_name
    failures = [None,
                'foo++', 'foo-bar', '#foo',
                'hello\n', '\t', ' name', 'name ',
                'f//b',
                '1name', 'foo\\']
    for f in failures:
      self.failIf(is_legal_name(f), f)
    tests = ['',
             'f', 'f1', 'f_', 'f/', 'foo', 'foo_bar', 'foo/bar', 'foo/bar/baz',
             '~f', '~a/b/c',
             '/a/b/c/d', '/']
    for t in tests:
      self.assert_(is_legal_name(t), "[%s]"%t)

  def test_is_legal_base_name(self):
    from roslib.names import is_legal_base_name
    failures = [None, '', 'hello\n', '\t', 'foo++', 'foo-bar', '#foo',
                'f/', 'foo/bar', '/', '/a',
                'f//b',
                '~f', '~a/b/c',                
                ' name', 'name ',
                '1name', 'foo\\']
    for f in failures:
      self.failIf(is_legal_base_name(f), f)
    tests = ['f', 'f1', 'f_', 'foo', 'foo_bar']
    for t in tests:
      self.assert_(is_legal_base_name(t), "[%s]"%t)

  def test_is_legal_resource_base_name(self):
    from roslib.names import is_legal_resource_base_name
    failures = [None, '', 'hello\n', '\t', 'foo++', 'foo-bar', '#foo',
                'f/', 'foo/bar', '/', '/a',
                'f//b',
                '~f', '~a/b/c',                
                ' name', 'name ',
                '1name', 'foo\\']
    for f in failures:
      self.failIf(is_legal_resource_base_name(f), f)
    tests = ['f', 'f1', 'f_', 'foo', 'foo_bar']
    for t in tests:
      self.assert_(is_legal_resource_base_name(t), "[%s]"%t)
      
if __name__ == '__main__':
  rostest.unitrun('test_roslib', 'test_names', NamesTest, coverage_packages=['roslib.names'])

