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
# Revision $Id$

PKG = 'test_roslaunch'
NAME = 'test_roslaunch_node_args'

import roslib; roslib.load_manifest(PKG)

import os
import sys
import unittest
    
## Test roslaunch.node_args
class TestNodeArgs(unittest.TestCase):

    def test_create_local_process_env(self):
        
        # this is almost entirely identical to the setup_env test in test_core, with some additional
        # higher level checks
        
        from roslaunch.core import Node, Machine
        from roslaunch.node_args import create_local_process_env
        ros_root = '/ros/root1'
        rpp = '/rpp1'
        master_uri = 'http://masteruri:1234'

        n = Node('nodepkg','nodetype')
        m = Machine('name1', ros_root, rpp, '1.2.3.4')
        d = create_local_process_env(n, m, master_uri)
        self.assertEquals(d['ROS_MASTER_URI'], master_uri)
        self.assertEquals(d['ROS_ROOT'], ros_root)
        self.assertEquals(d['PYTHONPATH'], os.path.join(ros_root, 'core', 'roslib', 'src'))
        self.assertEquals(d['ROS_PACKAGE_PATH'], rpp)
        for k in ['ROS_IP', 'ROS_NAMESPACE']:
            if k in d:
                self.fail('%s should not be set: %s'%(k,d[k]))

        # test that it inherits local environment
        env = os.environ.copy()
        env['PATH'] = '/some/path'
        env['FOO'] = 'foo'
        d = create_local_process_env(n, m, master_uri, env=env)
        self.assertEquals(d['FOO'], 'foo')

        # test that it defaults to os.environ
        os.environ['FOO'] = 'bar'
        d = create_local_process_env(n, m, master_uri)
        self.assertEquals(d['FOO'], 'bar')

        # test that our ROS_ROOT and PYTHONPATH override env
        env = os.environ.copy()
        env['ROS_ROOT'] = '/not/ros/root/'
        env['PYTHONPATH'] = '/some/path'
        d = create_local_process_env(n, m, master_uri, env=env)
        self.assertEquals(d['ROS_ROOT'], ros_root)
        self.assertEquals(d['PYTHONPATH'], os.path.join(ros_root, 'core', 'roslib', 'src'))

        # - make sure it didn't pollute original env
        self.assertEquals(env['ROS_ROOT'], '/not/ros/root/')
        self.assertEquals(env['PYTHONPATH'], '/some/path')
        
        # don't set ROS_ROOT and ROS_PACKAGE_PATH. ROS_ROOT should default, ROS_PACKAGE_PATH does not
        m = Machine('name1', '', '', '1.2.3.4')
        d = create_local_process_env(n, m, master_uri)
        val = os.environ['ROS_ROOT']
        self.assertEquals(d['ROS_ROOT'], val)
        self.assertEquals(d['PYTHONPATH'], os.path.join(val, 'core', 'roslib', 'src'))        
        self.failIf('ROS_PACKAGE_PATH' in d, 'ROS_PACKAGE_PATH should not be set: %s'%d)

        # test ROS_IP
        m = Machine('name1', ros_root, rpp, '1.2.3.4', ros_ip="4.5.6.7")
        n = Node('nodepkg','nodetype', namespace="/ns1")
        d = create_local_process_env(n, m, master_uri)
        self.assertEquals(d['ROS_NAMESPACE'], "/ns1")
        # test stripping
        n = Node('nodepkg','nodetype', namespace="/ns2/")
        d = create_local_process_env(n, m, master_uri)
        self.assertEquals(d['ROS_NAMESPACE'], "/ns2")

        # test ROS_NAMESPACE
        m = Machine('name1', ros_root, rpp, '1.2.3.4', ros_ip="4.5.6.7")
        d = create_local_process_env(n, m, master_uri)
        self.assertEquals(d['ROS_IP'], "4.5.6.7")

        # test node.env_args
        n = Node('nodepkg','nodetype', env_args=[('NENV1', 'val1'), ('NENV2', 'val2'), ('ROS_ROOT', '/new/root')])        
        d = create_local_process_env(n, m, master_uri)
        self.assertEquals(d['ROS_ROOT'], "/new/root")
        self.assertEquals(d['NENV1'], "val1")
        self.assertEquals(d['NENV2'], "val2")

        # test machine.env_args
        m = Machine('name1', ros_root, rpp, '1.2.3.4', ros_ip="4.5.6.7", env_args=[('MENV1', 'val1'), ('MENV2', 'val2'), ('ROS_ROOT', '/new/root2')])        
        n = Node('nodepkg','nodetype')
        d = create_local_process_env(n, m, master_uri)
        self.assertEquals(d['ROS_ROOT'], "/new/root2")
        self.assertEquals(d['MENV1'], "val1")
        self.assertEquals(d['MENV2'], "val2")

        # test node.env_args precedence
        m = Machine('name1', ros_root, rpp, '1.2.3.4', ros_ip="4.5.6.7", env_args=[('MENV1', 'val1'), ('MENV2', 'val2')])
        n = Node('nodepkg','nodetype', env_args=[('MENV1', 'nodeval1')])
        d = create_local_process_env(n, m, master_uri)
        self.assertEquals(d['MENV1'], "nodeval1")
        self.assertEquals(d['MENV2'], "val2")
        
if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_roslaunch', NAME, TestNodeArgs, coverage_packages=['roslaunch.node_args'])
    
