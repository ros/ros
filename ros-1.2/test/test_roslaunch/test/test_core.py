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

PKG = 'test_roslaunch'
NAME = 'test_core'

import roslib; roslib.load_manifest(PKG)

import os
import sys
import unittest

_lastmsg = None
def printlog_cb(msg):
    global _lastmsg 
    _lastmsg = msg
def printlog_cb_exception(msg):
    raise Exception(msg)
    
## Test roslaunch.core
class TestCore(unittest.TestCase):
    def test_xml_escape(self):
        from roslaunch.core import _xml_escape
        self.assertEquals('', _xml_escape(''))
        self.assertEquals(' ', _xml_escape(' '))        
        self.assertEquals('&quot;', _xml_escape('"'))
        self.assertEquals('&quot;hello world&quot;', _xml_escape('"hello world"'))
        self.assertEquals('&gt;', _xml_escape('>'))
        self.assertEquals('&lt;', _xml_escape('<'))
        self.assertEquals('&amp;', _xml_escape('&'))
        self.assertEquals('&amp;amp;', _xml_escape('&amp;'))        
        self.assertEquals('&amp;&quot;&gt;&lt;&quot;', _xml_escape('&"><"'))
        
        
    def test_child_mode(self):
        from roslaunch.core import is_child_mode, set_child_mode
        self.failIf(is_child_mode())
        set_child_mode(True)
        self.assert_(is_child_mode())        
        set_child_mode(False)
        self.failIf(is_child_mode())

    def test_printlog(self):
        from roslaunch.core import add_printlog_handler, add_printerrlog_handler, printlog, printlog_bold, printerrlog
        add_printlog_handler(printlog_cb)
        add_printlog_handler(printlog_cb_exception)        
        add_printerrlog_handler(printlog_cb)
        add_printerrlog_handler(printlog_cb_exception)        
            
        #can't really test functionality, just make sure it doesn't crash
        global _lastmsg
        _lastmsg = None
        printlog('foo')
        self.assertEquals('foo', _lastmsg)
        printlog_bold('bar')
        self.assertEquals('bar', _lastmsg)        
        printerrlog('baz')
        self.assertEquals('baz', _lastmsg)        
        
    def test_setup_env(self):
        from roslaunch.core import setup_env, Node, Machine
        ros_root = '/ros/root1'
        rpp = '/rpp1'
        master_uri = 'http://masteruri:1234'

        n = Node('nodepkg','nodetype')
        m = Machine('name1', ros_root, rpp, '1.2.3.4')
        d = setup_env(n, m, master_uri)
        self.assertEquals(d['ROS_MASTER_URI'], master_uri)
        self.assertEquals(d['ROS_ROOT'], ros_root)
        self.assertEquals(d['PYTHONPATH'], os.path.join(ros_root, 'core', 'roslib', 'src'))
        self.assertEquals(d['ROS_PACKAGE_PATH'], rpp)
        for k in ['ROS_IP', 'ROS_NAMESPACE']:
            if k in d:
                self.fail('%s should not be set: %s'%(k,d[k]))

        # don't set ROS_ROOT and ROS_PACKAGE_PATH. ROS_ROOT should default, ROS_PACKAGE_PATH does not
        m = Machine('name1', '', '', '1.2.3.4')
        d = setup_env(n, m, master_uri)
        val = os.environ['ROS_ROOT']
        self.assertEquals(d['ROS_ROOT'], val)
        self.assertEquals(d['PYTHONPATH'], os.path.join(val, 'core', 'roslib', 'src'))        
        self.failIf('ROS_PACKAGE_PATH' in d, 'ROS_PACKAGE_PATH should not be set: %s'%d)

        # test ROS_IP
        m = Machine('name1', ros_root, rpp, '1.2.3.4', ros_ip="4.5.6.7")
        n = Node('nodepkg','nodetype', namespace="/ns1")
        d = setup_env(n, m, master_uri)
        self.assertEquals(d['ROS_NAMESPACE'], "/ns1")
        # test stripping
        n = Node('nodepkg','nodetype', namespace="/ns2/")
        d = setup_env(n, m, master_uri)
        self.assertEquals(d['ROS_NAMESPACE'], "/ns2")

        # test ROS_NAMESPACE
        m = Machine('name1', ros_root, rpp, '1.2.3.4', ros_ip="4.5.6.7")
        d = setup_env(n, m, master_uri)
        self.assertEquals(d['ROS_IP'], "4.5.6.7")

        # test node.env_args
        n = Node('nodepkg','nodetype', env_args=[('NENV1', 'val1'), ('NENV2', 'val2'), ('ROS_ROOT', '/new/root')])        
        d = setup_env(n, m, master_uri)
        self.assertEquals(d['ROS_ROOT'], "/new/root")
        self.assertEquals(d['NENV1'], "val1")
        self.assertEquals(d['NENV2'], "val2")

        # test machine.env_args
        m = Machine('name1', ros_root, rpp, '1.2.3.4', ros_ip="4.5.6.7", env_args=[('MENV1', 'val1'), ('MENV2', 'val2'), ('ROS_ROOT', '/new/root2')])        
        n = Node('nodepkg','nodetype')
        d = setup_env(n, m, master_uri)
        self.assertEquals(d['ROS_ROOT'], "/new/root2")
        self.assertEquals(d['MENV1'], "val1")
        self.assertEquals(d['MENV2'], "val2")

        # test node.env_args precedence
        m = Machine('name1', ros_root, rpp, '1.2.3.4', ros_ip="4.5.6.7", env_args=[('MENV1', 'val1'), ('MENV2', 'val2')])
        n = Node('nodepkg','nodetype', env_args=[('MENV1', 'nodeval1')])
        d = setup_env(n, m, master_uri)
        self.assertEquals(d['MENV1'], "nodeval1")
        self.assertEquals(d['MENV2'], "val2")
        

        
    def test_Machine(self):
        from roslaunch.core import Machine
        m = Machine('name1', '/ros/root1', '/rpp1', '1.2.3.4')
        str(m), m.config_key() #test for error
        self.assertEquals(m, m)
        self.assertEquals(m, Machine('name1', '/ros/root1', '/rpp1', '1.2.3.4'))
        # verify that config_equals is not tied to name or assignable, but is tied to other properties
        self.assert_(m.config_equals(Machine('name1b', '/ros/root1', '/rpp1', '1.2.3.4')))
        self.assert_(m.config_equals(Machine('name1c', '/ros/root1', '/rpp1', '1.2.3.4', assignable=False)))
        self.failIf(m.config_equals(Machine('name1d', '/ros/root2', '/rpp1', '1.2.3.4')))
        self.failIf(m.config_equals(Machine('name1e', '/ros/root1', '/rpp2', '1.2.3.4')))
        self.failIf(m.config_equals(Machine('name1f', '/ros/root1', '/rpp1', '2.2.3.4')))
        
        self.assertEquals(m.name, 'name1')
        self.assertEquals(m.ros_root, '/ros/root1')
        self.assertEquals(m.ros_package_path, '/rpp1')
        self.assertEquals(m.address, '1.2.3.4')
        self.assertEquals(m.assignable, True)
        self.assertEquals(m.ssh_port, 22)        
        self.assertEquals(m.env_args, [])        
        for p in ['ros_ip', 'user', 'password']:
            self.assertEquals(getattr(m, p), None)

        m = Machine('name2', '/ros/root2', '/rpp2', '2.2.3.4', assignable=False)
        self.assertEquals(m.assignable, False)
        str(m), m.config_key() #test for error
        self.assertEquals(m, m)
        self.assertEquals(m, Machine('name2', '/ros/root2', '/rpp2', '2.2.3.4', assignable=False))
        self.assert_(m.config_equals(Machine('name2b', '/ros/root2', '/rpp2', '2.2.3.4', assignable=False)))
        self.assert_(m.config_equals(Machine('name2c', '/ros/root2', '/rpp2', '2.2.3.4', assignable=True)))

        m = Machine('name3', '/ros/root3', '/rpp3', '3.3.3.4', ros_ip='4.5.6.7')
        self.assertEquals(m.ros_ip, '4.5.6.7')
        str(m) #test for error
        self.assertEquals(m, m)
        self.assertEquals(m, Machine('name3', '/ros/root3', '/rpp3', '3.3.3.4', ros_ip='4.5.6.7'))
        self.assert_(m.config_equals(Machine('name3b', '/ros/root3', '/rpp3', '3.3.3.4', ros_ip='4.5.6.7')))
            
        m = Machine('name4', '/ros/root4', '/rpp4', '4.4.3.4', user='user4')
        self.assertEquals(m.user, 'user4')
        str(m), m.config_key() #test for error
        self.assertEquals(m, m)
        self.assertEquals(m, Machine('name4', '/ros/root4', '/rpp4', '4.4.3.4', user='user4'))
        self.assert_(m.config_equals(Machine('name4b', '/ros/root4', '/rpp4', '4.4.3.4', user='user4')))
        self.failIf(m.config_equals(Machine('name4b', '/ros/root4', '/rpp4', '4.4.3.4', user='user4b')))
            
        m = Machine('name5', '/ros/root5', '/rpp5', '5.5.3.4', password='password5')
        self.assertEquals(m.password, 'password5')
        str(m), m.config_key() #test for error
        self.assertEquals(m, m)
        self.assertEquals(m, Machine('name5', '/ros/root5', '/rpp5', '5.5.3.4', password='password5'))
        self.assert_(m.config_equals(Machine('name5b', '/ros/root5', '/rpp5', '5.5.3.4', password='password5')))
        self.failIf(m.config_equals(Machine('name5c', '/ros/root5', '/rpp5', '5.5.3.4', password='password5c')))

        m = Machine('name6', '/ros/root6', '/rpp6', '6.6.3.4', ssh_port=23)
        self.assertEquals(m.ssh_port, 23)
        str(m) #test for error
        m.config_key()
        self.assertEquals(m, m)
        self.assertEquals(m, Machine('name6', '/ros/root6', '/rpp6', '6.6.3.4', ssh_port=23))
        self.assert_(m.config_equals(Machine('name6b', '/ros/root6', '/rpp6', '6.6.3.4', ssh_port=23)))
        self.failIf(m.config_equals(Machine('name6c', '/ros/root6', '/rpp6', '6.6.3.4', ssh_port=24)))

        m = Machine('name7', '/ros/root7', '/rpp7', '7.7.3.4', env_args=[('ARG', 'val'), ('ARG2', 'val')])
        self.assertEquals(m.env_args, [('ARG', 'val'), ('ARG2', 'val')])
        str(m), m.config_key() #test for error
        self.assertEquals(m, m)
        self.assertEquals(m, Machine('name7', '/ros/root7', '/rpp7', '7.7.3.4', env_args=[('ARG', 'val'), ('ARG2', 'val')]))
        self.assert_(m.config_equals(Machine('name7b', '/ros/root7', '/rpp7', '7.7.3.4', env_args=[('ARG', 'val'), ('ARG2', 'val')])))
        self.assert_(m.config_equals(Machine('name7b', '/ros/root7', '/rpp7', '7.7.3.4', env_args=[('ARG2', 'val'), ('ARG', 'val')])))
        self.failIf(m.config_equals(Machine('name7c', '/ros/root7', '/rpp7', '7.7.3.4', env_args=[('ARGc', 'valc'), ('ARG2', 'val')])))
            
    def test_local_machine(self):
        try:
            env_copy = os.environ.copy()
            if 'ROS_IP' in os.environ:
                del os.environ['ROS_IP']
            if 'ROS_HOSTNAME' in os.environ:
                del os.environ['ROS_HOSTNAME']

            from roslaunch.core import local_machine
            lm = local_machine()
            self.failIf(lm is None)
            #singleton
            self.assert_(lm == local_machine())

            #verify properties
            self.assertEquals(lm.ros_root, roslib.rosenv.get_ros_root())
            self.assertEquals(lm.ros_package_path, roslib.rosenv.get_ros_package_path())        
        
            # #1051 important test: this caused regressions up the tree
            self.assertEquals(lm.ros_ip, None)
        
            self.assertEquals(lm.name, '')
            self.assertEquals(lm.assignable, True)
            self.assertEquals(lm.env_args, [])        
            self.assertEquals(lm.user, None)
            self.assertEquals(lm.password, None)
        finally:
            os.environ = env_copy

        
    def test_Master(self):
        from roslaunch.core import Master
        old_env = os.environ.get('ROS_MASTER_URI', None)
        try:
            os.environ['ROS_MASTER_URI'] = 'http://foo:789'
            m = Master()
            self.assertEquals(m.type, Master.ROSMASTER)
            self.assertEquals(m.uri, 'http://foo:789')
            self.assertEquals(m.auto, Master.AUTO_NO)
            self.assertEquals(m.log_output, False)
            self.assertEquals(m, m)
            self.assertEquals(m, Master())
            
            m = Master(Master.ROSMASTER, 'http://foo:1234', Master.AUTO_START)
            self.assertEquals(m.type, Master.ROSMASTER)
            self.assertEquals(m.uri, 'http://foo:1234')
            self.assertEquals(m.auto, Master.AUTO_START)
            self.assertEquals(m.log_output, False)
            self.assertEquals(m, m)
            self.assertEquals(m, Master(Master.ROSMASTER, 'http://foo:1234', Master.AUTO_START))

            import xmlrpclib
            self.assert_(isinstance(m.get(), xmlrpclib.ServerProxy))
            m.set_port(567)
            self.assertEquals(m.uri, 'http://foo:567/')
            self.failIf(m.is_running())

            try:
                m = Master(Master.ROSMASTER, 'http://foo:1234', False)
                self.fail("should have failed on invalid auto value")
            except: pass
            try:
                m = Master(Master.ROSMASTER, 'http://foo:1234', 123)
                self.fail("should have failed on invalid auto value")
            except: pass
            
        finally:
            if old_env is None:
                del os.environ['ROS_MASTER_URI']
            else:
                os.environ['ROS_MASTER_URI'] = old_env

    def test_Executable(self):
        from roslaunch.core import Executable, PHASE_SETUP, PHASE_RUN, PHASE_TEARDOWN

        e = Executable('ls', ['-alF'])
        self.assertEquals('ls', e.command)
        self.assertEquals(['-alF'], e.args)
        self.assertEquals(PHASE_RUN, e.phase)
        self.assertEquals('ls -alF', str(e))
        self.assertEquals('ls -alF', repr(e))        

        e = Executable('ls', ['-alF', 'b*'], PHASE_TEARDOWN)
        self.assertEquals('ls', e.command)
        self.assertEquals(['-alF', 'b*'], e.args)
        self.assertEquals(PHASE_TEARDOWN, e.phase)
        self.assertEquals('ls -alF b*', str(e))
        self.assertEquals('ls -alF b*', repr(e))        

    def test_RosbinExecutable(self):
        from roslaunch.core import RosbinExecutable, PHASE_SETUP, PHASE_RUN, PHASE_TEARDOWN

        e = RosbinExecutable('ls', ['-alF'])
        self.assertEquals('ls', e.command)
        self.assertEquals(['-alF'], e.args)
        self.assertEquals(PHASE_RUN, e.phase)
        self.assertEquals('ros/bin/ls -alF', str(e))
        self.assertEquals('ros/bin/ls -alF', repr(e))        

        e = RosbinExecutable('ls', ['-alF', 'b*'], PHASE_TEARDOWN)
        self.assertEquals('ls', e.command)
        self.assertEquals(['-alF', 'b*'], e.args)
        self.assertEquals(PHASE_TEARDOWN, e.phase)
        self.assertEquals('ros/bin/ls -alF b*', str(e))
        self.assertEquals('ros/bin/ls -alF b*', repr(e))        
        
if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_roslaunch', NAME, TestCore, coverage_packages=['roslaunch.core'])
    
