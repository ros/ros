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
  
def test_get_ros_namespace():
    if 'ROS_NAMESPACE' in os.environ:
        rosns = os.environ['ROS_NAMESPACE']
        del os.environ['ROS_NAMESPACE']
    else:
        rosns = None
    sysargv = sys.argv

    from rosgraph.names import get_ros_namespace
    try:
        sys.argv = []
        assert '/' == get_ros_namespace()
        assert '/' == get_ros_namespace(argv=[])
        assert '/' == get_ros_namespace(env={})
        assert '/' == get_ros_namespace(env={}, argv=[])

        os.environ['ROS_NAMESPACE'] = 'unresolved'
        assert '/unresolved/' == get_ros_namespace()
        assert '/unresolved/' == get_ros_namespace(env={'ROS_NAMESPACE': 'unresolved'})
        sys.argv = ['foo', '__ns:=unresolved_override']
        assert '/unresolved_override/' == get_ros_namespace(env={'ROS_NAMESPACE': 'unresolved'})
        assert '/override2/' == get_ros_namespace(env={'ROS_NAMESPACE': 'unresolved'}, argv=['foo', '__ns:=override2'])

        sys.argv = []
        os.environ['ROS_NAMESPACE'] = '/resolved/'
        assert '/resolved/' == get_ros_namespace()
        assert '/resolved/' == get_ros_namespace(env={'ROS_NAMESPACE': '/resolved'})

        del os.environ['ROS_NAMESPACE']

        sys.argv = ['foo', '__ns:=unresolved_ns']
        assert '/unresolved_ns/' == get_ros_namespace()
        assert '/unresolved_ns2/' == get_ros_namespace(argv=['foo', '__ns:=unresolved_ns2'])
        sys.argv = ['foo', '__ns:=/resolved_ns/']
        assert '/resolved_ns/' == get_ros_namespace()
        assert '/resolved_ns2/' == get_ros_namespace(argv=['foo', '__ns:=resolved_ns2'])
    finally:
        sys.argv = sysargv

        # restore
        if rosns:
            os.environ['ROS_NAMESPACE'] = rosns

def test_make_global_ns():
    from rosgraph.names import make_global_ns

    for n in ['~foo']:
        try:
            make_global_ns(n)
            assert False, "make_global_ns should fail on %s"%n
        except ValueError: pass

    assert '/foo/' == make_global_ns('foo')
    assert '/' == make_global_ns('')
    assert '/foo/' == make_global_ns('/foo')
    assert '/foo/' == make_global_ns('/foo/')
    assert '/foo/bar/' == make_global_ns('/foo/bar')
    assert '/foo/bar/' == make_global_ns('/foo/bar/')

def test_make_caller_id():
    from rosgraph.names import make_caller_id
    if 'ROS_NAMESPACE' is os.environ:
        rosns = os.environ['ROS_NAMESPACE']
        del os.environ['ROS_NAMESPACE']
    else:
        rosns = None

    for n in ['~name']:
        try:
            make_caller_id('~name') # illegal
            assert False, "make_caller_id should fail on %s"%n
        except ValueError:
            pass
    
    assert '/node/' == make_caller_id('node')
    assert '/bar/node/' == make_caller_id('bar/node')
    assert '/bar/node/' == make_caller_id('/bar/node')

    os.environ['ROS_NAMESPACE'] = '/test/'
    assert '/test/node/' == make_caller_id('node')
    assert '/test/bar/node/' == make_caller_id('bar/node')
    assert '/bar/node/' == make_caller_id('/bar/node')
    
    # restore
    if rosns:
        os.environ['ROS_NAMESPACE'] = rosns
  
def test_is_global():
    from rosgraph.names import is_global
    try:
        is_global(None)
        assert False, "is_global should raise exception on invalid param"
    except: pass
    tests = ['/', '/global', '/global2']
    for t in tests:
        assert is_global(t)
    fails = ['', 'not_global', 'not/global']
    for t in fails:
        assert not is_global(t)
    
def test_is_private():
    from rosgraph.names import is_private
    try:
        is_private(None)
        assert False, "is_private should raise exception on invalid param"
    except: pass
    tests = ['~name', '~name/sub']
    for t in tests:
        assert is_private(t)
    fails = ['', 'not_private', 'not/private', 'not/~private', '/not/~private']
    for t in fails:
        assert not is_private(t)
      
def test_namespace():
    from rosgraph.names import namespace
    try:
        namespace(1)
        assert False, "1"
    except TypeError: pass
    try:
        namespace(None)
        assert False, "None"
    except ValueError: pass
    assert '/'== namespace('')
    assert '/'== namespace('/')
    assert '/'== namespace('/foo')
    assert '/'== namespace('/foo/')
    assert '/foo/'== namespace('/foo/bar')
    assert '/foo/'== namespace('/foo/bar/')
    assert '/foo/bar/'== namespace('/foo/bar/baz')
    assert '/foo/bar/'== namespace('/foo/bar/baz/')

    # unicode tests
    assert u'/'== namespace(u'')
    assert u'/'== namespace(u'/')
    assert u'/foo/bar/'== namespace(u'/foo/bar/baz/')

def test_nsjoin():
    from rosgraph.names import ns_join

    # private and global names cannot be joined
    assert '~name' == ns_join('/foo', '~name')
    assert '/name' == ns_join('/foo', '/name')
    assert '~name' == ns_join('~', '~name')
    assert '/name' == ns_join('/', '/name')

    # ns can be '~' or '/'
    assert '~name' == ns_join('~', 'name')
    assert '/name' == ns_join('/', 'name')

    assert '/ns/name' == ns_join('/ns', 'name')
    assert '/ns/name' == ns_join('/ns/', 'name')
    assert '/ns/ns2/name' == ns_join('/ns', 'ns2/name')
    assert '/ns/ns2/name' == ns_join('/ns/', 'ns2/name')

    # allow ns to be empty
    assert 'name' == ns_join('', 'name')
    

def test_load_mappings():
    from rosgraph.names import load_mappings
    assert {} == load_mappings([])
    assert {} == load_mappings(['foo'])
    assert {} == load_mappings([':='])
    assert {} == load_mappings([':=:='])
    assert {} == load_mappings(['f:='])
    assert {} == load_mappings([':=b'])
    assert {} == load_mappings(['foo:=bar:=baz'])
    # should ignore node param assignments
    assert {} == load_mappings(['_foo:=bar'])
    
    assert {'foo': 'bar'} == load_mappings(['foo:=bar'])
    # should allow double-underscore names
    assert {'__foo': 'bar'} == load_mappings(['__foo:=bar'])
    assert {'foo': 'bar'} == load_mappings(['./f', '-x', '--blah', 'foo:=bar'])
    assert {'a': '1', 'b': '2', 'c': '3'} == load_mappings(['c:=3', 'c:=', ':=3', 'a:=1', 'b:=2'])

def test_is_legal_name():
    from rosgraph.names import is_legal_name
    failures = [None,
                'foo++', 'foo-bar', '#foo',
                'hello\n', '\t', ' name', 'name ',
                'f//b',
                '1name', 'foo\\']
    for f in failures:
        assert not is_legal_name(f), f
    tests = ['',
             'f', 'f1', 'f_', 'f/', 'foo', 'foo_bar', 'foo/bar', 'foo/bar/baz',
             '~f', '~a/b/c',
             '~/f',
             '/a/b/c/d', '/']
    for t in tests:
        assert is_legal_name(t), "[%s]"%t

def test_is_legal_base_name():
    from rosgraph.names import is_legal_base_name
    failures = [None, '', 'hello\n', '\t', 'foo++', 'foo-bar', '#foo',
                'f/', 'foo/bar', '/', '/a',
                'f//b',
                '~f', '~a/b/c',                
                ' name', 'name ',
                '1name', 'foo\\']
    for f in failures:
        assert not is_legal_base_name(f), f
    tests = ['f', 'f1', 'f_', 'foo', 'foo_bar']
    for t in tests:
        assert is_legal_base_name(t), "[%s]"%t

def test_resolve_name():
      from rosgraph.names import resolve_name
      # TODO: test with remappings
      tests = [
          ('', '/', '/'),
          ('', '/node', '/'),
          ('', '/ns1/node', '/ns1/'),

          ('foo', '', '/foo'),
          ('foo/', '', '/foo'),
          ('/foo', '', '/foo'),
          ('/foo/', '', '/foo'),
          ('/foo', '/', '/foo'),
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

          # #3044
          ('~/foo', '/', '/foo'),            
          ('~/foo', '/node', '/node/foo'),            
          ('~/foo', '/ns1/ns2', '/ns1/ns2/foo'),            
          ('~/foo/', '/ns1/ns2', '/ns1/ns2/foo'),            
          ('~/foo/bar', '/ns1/ns2', '/ns1/ns2/foo/bar'),

          ]
      for name, node_name, v in tests:
          assert v == resolve_name(name, node_name)

def test_anonymous_name():
    from rosgraph.names import anonymous_name, is_legal_name
    val = anonymous_name('foo')
    assert 'foo' in val
    assert 'foo' != val
    assert val != anonymous_name('foo')
    assert not '/' in val
    assert is_legal_name(val)
    
    
def test_script_resolve_name():
    from rosgraph.names import script_resolve_name, get_ros_namespace, ns_join

    assert '/global' == script_resolve_name('/myscript', '/global')
    val = script_resolve_name('/myscript', '')    
    assert get_ros_namespace() == val, val
    val = script_resolve_name('/myscript', 'foo')    
    assert ns_join(get_ros_namespace(), 'foo') == val, val
    assert '/myscript/private' == script_resolve_name('/myscript', '~private')    
    
def test_canonicalize_name():
    from rosgraph.names import canonicalize_name
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
        assert v == canonicalize_name(t)
