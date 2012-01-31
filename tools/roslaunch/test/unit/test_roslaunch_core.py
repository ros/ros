# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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
import rospkg
import xmlrpclib


import roslaunch.core

def test_Executable():
    from roslaunch.core import Executable, PHASE_SETUP
    e = Executable('cmd', ('arg1', 'arg2'), PHASE_SETUP)
    assert e.command == 'cmd'
    assert e.args == ('arg1', 'arg2')
    assert e.phase == PHASE_SETUP
    assert 'cmd' in str(e)
    assert 'arg2' in str(e)
    assert 'cmd' in repr(e)
    assert 'arg2' in repr(e)
    
def test__xml_escape():
    # this is a really bad xml escaper
    from roslaunch.core import _xml_escape
    assert _xml_escape("&<foo>") == "&amp;&lt;foo&gt;"
    
def test_RosbinExecutable():
    from roslaunch.core import RosbinExecutable, PHASE_SETUP, PHASE_RUN
    e = RosbinExecutable('cmd', ('arg1', 'arg2'), PHASE_SETUP)
    assert e.command == 'cmd'
    assert e.args == ('arg1', 'arg2')
    assert e.phase == PHASE_SETUP
    assert 'cmd' in str(e)
    assert 'arg2' in str(e)
    assert 'cmd' in repr(e)
    assert 'arg2' in repr(e)
    assert 'ros/bin' in str(e)
    assert 'ros/bin' in repr(e)    

    e = RosbinExecutable('cmd', ('arg1', 'arg2'))
    assert e.phase == PHASE_RUN
    
def test_generate_run_id():
    from roslaunch.core import generate_run_id
    assert generate_run_id() 
    
def test_Node():
    from roslaunch.core import Node
    n = Node('package', 'node_type')
    assert n.package == 'package'
    assert n.type == 'node_type'
    assert n.xmltype() == 'node'
    assert n.xmlattrs() == [('pkg', 'package'), ('type', 'node_type'), ('machine', None), ('ns', '/'), ('args', ''), ('output', None), ('cwd', None), ('respawn', False), ('name', None), ('launch-prefix', None), ('required', False)], n.xmlattrs()
    assert n.output == None

    #tripwire for now
    n.to_xml()
    val = n.to_remote_xml()
    assert 'machine' not in val
    
    # test bad constructors
    try:
        n = Node('package', 'node_type', cwd='foo')
        assert False
    except ValueError:
        pass
    try:
        n = Node('package', 'node_type', output='foo')
        assert False
    except ValueError:
        pass
    try:
        n = Node('package', '')
        assert False
    except ValueError:
        pass
    try:
        n = Node('', 'node_type')
        assert False
    except ValueError:
        pass
    try:
        n = Node('package', 'node_type', name='ns/node')
        assert False
    except ValueError:
        pass
    try:
        n = Node('package', 'node_type', respawn=True, required=True)
        assert False
    except ValueError:
        pass
    

def test_Param():
    from roslaunch.core import Param
    p = Param('key/', 'value')

    assert p.key == 'key'
    assert p.value == 'value'
    assert p == Param('key', 'value')
    assert p != Param('key2', 'value')
    assert p != 1
    
    assert 'key' in str(p)
    assert 'key' in repr(p)    
    assert 'value' in str(p)
    assert 'value' in repr(p)    
    
def test_local_machine():
    from roslaunch.core import local_machine
    m = local_machine()
    assert m is local_machine()
    assert m == local_machine()
    assert m.env_loader == None
    assert m.name == ''
    assert m.address == 'localhost'
    assert m.ssh_port == 22
    
def test_Machine():
    from roslaunch.core import Machine
    m = Machine('foo', 'localhost')
    assert 'foo' in str(m)
    assert m.name == 'foo'
    assert m.env_loader == None
    assert m.assignable == True
    assert m == m
    assert not m.__eq__(1)
    assert not m.config_equals(1)
    assert m == Machine('foo', 'localhost')
    assert m.config_equals(Machine('foo', 'localhost'))
    assert m.config_key() == Machine('foo', 'localhost').config_key()
    assert m.config_equals(Machine('foo', 'localhost'))
    assert m.config_key() == Machine('foo', 'localhost', ssh_port=22).config_key()    
    assert m.config_equals(Machine('foo', 'localhost', ssh_port=22))
    assert m.config_key() == Machine('foo', 'localhost', assignable=False).config_key()
    assert m.config_equals(Machine('foo', 'localhost', assignable=False))
    
    # original test suite
    m = Machine('name1', '1.2.3.4')
    str(m), m.config_key() #test for error
    assert m == m
    assert m == Machine('name1', '1.2.3.4')
    # verify that config_equals is not tied to name or assignable, but is tied to other properties
    assert m.config_equals(Machine('name1b', '1.2.3.4'))
    assert m.config_equals(Machine('name1c', '1.2.3.4', assignable=False))
    assert not m.config_equals(Machine('name1f', '2.2.3.4'))
        
    assert m.name == 'name1'
    assert m.address == '1.2.3.4'
    assert m.assignable == True
    assert m.ssh_port == 22   
    assert m.env_loader == None
    for p in ['user', 'password']:
        assert getattr(m, p) is None

    m = Machine('name2', '2.2.3.4', assignable=False)
    assert not m.assignable
    str(m), m.config_key() #test for error
    assert m == m
    assert m == Machine('name2', '2.2.3.4', assignable=False)
    assert m.config_equals(Machine('name2b', '2.2.3.4', assignable=False))
    assert m.config_equals(Machine('name2c', '2.2.3.4', assignable=True))

    m = Machine('name3', '3.3.3.4')
    str(m) #test for error
    assert m == m
    assert m == Machine('name3', '3.3.3.4')
    assert m.config_equals(Machine('name3b', '3.3.3.4'))
            
    m = Machine('name4', '4.4.3.4', user='user4')
    assert m.user == 'user4'
    str(m), m.config_key() #test for error
    assert m == m
    assert m == Machine('name4', '4.4.3.4', user='user4')
    assert m.config_equals(Machine('name4b', '4.4.3.4', user='user4'))
    assert not m.config_equals(Machine('name4b', '4.4.3.4', user='user4b'))
            
    m = Machine('name5', '5.5.3.4', password='password5')
    assert m.password == 'password5'
    str(m), m.config_key() #test for error
    assert m == m
    assert m == Machine('name5', '5.5.3.4', password='password5')
    assert m.config_equals(Machine('name5b', '5.5.3.4', password='password5'))
    assert not m.config_equals(Machine('name5c', '5.5.3.4', password='password5c'))

    m = Machine('name6', '6.6.3.4', ssh_port=23)
    assert m.ssh_port == 23
    str(m) #test for error
    m.config_key()
    assert m == m
    assert m == Machine('name6', '6.6.3.4', ssh_port=23)
    assert m.config_equals(Machine('name6b', '6.6.3.4', ssh_port=23))
    assert not m.config_equals(Machine('name6c', '6.6.3.4', ssh_port=24))

    m = Machine('name6', '6.6.3.4', env_loader='/opt/ros/fuerte/setup.sh')
    assert m.env_loader == '/opt/ros/fuerte/setup.sh'
    str(m) #test for error
    m.config_key()
    assert m == m
    assert m == Machine('name6', '6.6.3.4', env_loader='/opt/ros/fuerte/setup.sh')
    assert m.config_equals(Machine('name6b', '6.6.3.4', env_loader='/opt/ros/fuerte/setup.sh'))
    assert not m.config_equals(Machine('name6c', '6.6.3.4', env_loader='/opt/ros/groovy/setup.sh'))

def test_Master():
    from roslaunch.core import Master
    # can't verify value of is_running for actual master
    m = Master(uri='http://localhost:11311')
    assert m .type == Master.ROSMASTER
    m.get_host() == 'localhost'
    m.get_port() == 11311
    assert m.is_running() in [True, False]
    assert isinstance(m.get(), xmlrpclib.ServerProxy)
    assert isinstance(m.get_multi(), xmlrpclib.MultiCall)
    
    m = Master(uri='http://badhostname:11312')
    m.get_host() == 'badhostname'
    m.get_port() == 11312
    assert m.is_running() == False

    
def test_Test():
    from roslaunch.core import Test, TEST_TIME_LIMIT_DEFAULT
    t = Test('test_name', 'package', 'node_type')
    assert t.xmltype() == 'test'
    assert t.xmlattrs() == [('pkg', 'package'), ('type', 'node_type'), ('machine', None), ('ns', '/'), ('args', ''), ('output', 'log'), ('cwd', None), ('name', None), ('launch-prefix', None), ('required', False), ('test-name', 'test_name')]

    assert t.output == 'log'
    assert t.test_name == 'test_name'
    assert t.package == 'package'
    assert t.type == 'node_type'
    assert t.name == None
    assert t.namespace == '/'
    assert t.machine_name == None
    assert t.args == ''
    assert t.remap_args == [], t.remap_args
    assert t.env_args == []
    assert t.time_limit == TEST_TIME_LIMIT_DEFAULT
    assert t.cwd is None
    assert t.launch_prefix is None
    assert t.retry == 0
    assert t.filename == '<unknown>'
    
    
    t = Test('test_name', 'package', 'node_type', 'name', 
             '/namespace', 'machine_name', 'arg1 arg2', 
             remap_args=[('from', 'to')], env_args=[('ENV', 'val')], time_limit=1.0, cwd='ros_home',
             launch_prefix='foo', retry=1, filename="filename.txt")

    xmlattrs = t.xmlattrs()
    assert ('time-limit', 1.0) in xmlattrs
    assert ('retry', '1') in xmlattrs
    
    assert t.launch_prefix == 'foo'
    assert t.remap_args == [('from', 'to')]
    assert t.env_args == [('ENV', 'val')]
    assert t.name == 'name'
    assert t.namespace == '/namespace/', t.namespace
    assert t.machine_name == 'machine_name'
    assert t.args == 'arg1 arg2'
    assert t.filename == 'filename.txt'
    assert t.time_limit == 1.0
    assert t.cwd == 'ROS_HOME'
    assert t.retry == 1

    try:
        t = Test('test_name', 'package', 'node_type', time_limit=-1.0)
        assert False
    except ValueError:
        pass
    try:
        t = Test('test_name', 'package', 'node_type', time_limit=True)
        assert False
    except ValueError:
        pass
