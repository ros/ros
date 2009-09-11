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
# Revision $Id: tcpros.py 1636 2009-07-28 20:58:29Z sfkwc $

PKG = 'test_roslaunch'
NAME = 'test_nodeprocess'

import roslib; roslib.load_manifest(PKG)

import os
import sys
import unittest
    
## Test roslaunch.nodeprocess
class TestNodeprocess(unittest.TestCase):

    def test_create_master_process(self):
        from roslaunch.core import Node, Machine, Master, RLException
        from roslaunch.nodeprocess import create_master_process, LocalProcess
        
        ros_root = '/ros/root'
        port = 1234
        type = Master.ZENMASTER
        run_id = 'foo'

        # test invalid params
        try:
            create_master_process(run_id, type, ros_root, -1)
            self.fail("shoud have thrown RLException")
        except RLException: pass
        try:
            create_master_process(run_id, type, ros_root, 10000000)
            self.fail("shoud have thrown RLException")
        except RLException: pass
        try:
            create_master_process(run_id, 'foo', ros_root, port)
            self.fail("shoud have thrown RLException")
        except RLException: pass

        # test valid params
        p = create_master_process(run_id, type, ros_root, port)
        self.assert_(isinstance(p, LocalProcess))
        self.assertEquals(p.args[0], os.path.join(ros_root, 'bin', 'zenmaster'))
        idx = p.args.index('-p')
        self.failIf(idx < 1)
        self.assertEquals(p.args[idx+1], str(port))
        self.assert_('--core' in p.args)

        self.assertEquals(p.package, 'rospy')
        self.failIf(p.log_output)
        
        p = create_master_process(run_id, type, ros_root, port, log_output=True)
        self.assert_(p.log_output)
        
        # verify that 'botherder' still works, though deprecated
        p = create_master_process(run_id, Master.BOTHERDER, ros_root, port)
        self.assertEquals(p.args[0], os.path.join(ros_root, 'bin', 'botherder'))
        self.assertEquals(p.args[1], str(port))
        
        # TODO: have to think more as to the correct environment for the master process
        
        
    def test_create_node_process(self):
        from roslaunch.core import Node, Machine, RLException
        from roslaunch.nodeprocess import create_node_process, LocalProcess, NodeParamsException
        # have to use real ROS configuration for these tests
        ros_root = os.environ['ROS_ROOT']
        rpp = os.environ.get('ROS_PACKAGE_PATH', None)
        master_uri = 'http://masteruri:1234'
        m = Machine('name1', ros_root, rpp, '1.2.3.4')

        run_id = 'id'
        
        # test invalid params
        n = Node('not_a_real_package','not_a_node')
        n.machine = m
        try: # should fail b/c node cannot be found
            create_node_process(run_id, n, master_uri)
        except NodeParamsException:
            pass
        
        # have to specify a real node
        n = Node('rospy_tutorials','talker.py')

        n.machine = None
        try: # should fail b/c n.machine is not set
            create_node_process(run_id, n, master_uri)
        except RLException:
            pass

        # basic integration test
        n.machine = m
        p = create_node_process(run_id, n, master_uri)
        self.assert_(isinstance(p, LocalProcess))

        # repeat some setup_local_process_env tests
        d = p.env
        self.assertEquals(d['ROS_MASTER_URI'], master_uri)
        self.assertEquals(d['ROS_ROOT'], ros_root)
        self.assertEquals(d['PYTHONPATH'], os.path.join(ros_root, 'core', 'roslib', 'src'))
        if rpp:
            self.assertEquals(d['ROS_PACKAGE_PATH'], rpp)
        for k in ['ROS_IP', 'ROS_NAMESPACE']:
            if k in d:
                self.fail('%s should not be set: %s'%(k,d[k]))

        # test package and name
        self.assertEquals(p.package, 'rospy_tutorials')
        # - no 'correct' full answer here 
        self.assert_(p.name.startswith('talker'))

        # test log_output
        n.output = 'log'
        self.assert_(create_node_process(run_id, n, master_uri).log_output)
        n.output = 'screen'
        self.failIf(create_node_process(run_id, n, master_uri).log_output)

        # test respawn
        n.respawn = True
        self.assert_(create_node_process(run_id, n, master_uri).respawn)
        n.respawn = False
        self.failIf(create_node_process(run_id, n, master_uri).respawn)        

        # test cwd
        n.cwd = None
        self.assertEquals(create_node_process(run_id, n, master_uri).cwd, None)
        n.cwd = 'ros-root'
        self.assertEquals(create_node_process(run_id, n, master_uri).cwd, 'ros-root')
        n.cwd = 'node'                
        self.assertEquals(create_node_process(run_id, n, master_uri).cwd, 'node')

        # test args

        # - simplest test (no args)
        n.args = ''
        p = create_node_process(run_id, n, master_uri)
        # - the first arg should be the path to the node executable
        cmd = roslib.packages.find_node('rospy_tutorials', 'talker.py', ros_root, rpp)
        self.assertEquals(p.args[0], cmd)

        # - test basic args
        n.args = "arg1 arg2 arg3"
        p = create_node_process(run_id, n, master_uri)
        self.assertEquals(p.args[0], cmd)
        for a in "arg1 arg2 arg3".split():
            self.assert_(a in p.args)
            
        # - test remap args
        n.remap_args = [('KEY1', 'VAL1'), ('KEY2', 'VAL2')]
        p = create_node_process(run_id, n, master_uri)
        self.assert_('KEY1:=VAL1' in p.args)        
        self.assert_('KEY2:=VAL2' in p.args)
        
        # - test __name
        n = Node('rospy_tutorials','talker.py')
        n.name = 'fooname'
        n.machine = m
        self.assert_('__name:=fooname' in create_node_process(run_id, n, master_uri).args)
        
        # - test substitution args
        os.environ['SUB_TEST'] = 'subtest'
        os.environ['SUB_TEST2'] = 'subtest2'
        n.args = 'foo $(env SUB_TEST) $(env SUB_TEST2)'
        p = create_node_process(run_id, n, master_uri)        
        self.failIf('SUB_TEST' in p.args)
        self.assert_('foo' in p.args)
        self.assert_('subtest' in p.args)
        self.assert_('subtest2' in p.args)        

    
    def test_setup_local_process_env(self):
        
        # this is almost entirely identical to the setup_env test in test_core, with some additional
        # higher level checks
        
        from roslaunch.core import Node, Machine
        from roslaunch.nodeprocess import setup_local_process_env
        ros_root = '/ros/root1'
        rpp = '/rpp1'
        master_uri = 'http://masteruri:1234'

        n = Node('nodepkg','nodetype')
        m = Machine('name1', ros_root, rpp, '1.2.3.4')
        d = setup_local_process_env(n, m, master_uri)
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
        d = setup_local_process_env(n, m, master_uri, env=env)
        self.assertEquals(d['FOO'], 'foo')

        # test that it defaults to os.environ
        os.environ['FOO'] = 'bar'
        d = setup_local_process_env(n, m, master_uri)
        self.assertEquals(d['FOO'], 'bar')

        # test that our ROS_ROOT and PYTHONPATH override env
        env = os.environ.copy()
        env['ROS_ROOT'] = '/not/ros/root/'
        env['PYTHONPATH'] = '/some/path'
        d = setup_local_process_env(n, m, master_uri, env=env)
        self.assertEquals(d['ROS_ROOT'], ros_root)
        self.assertEquals(d['PYTHONPATH'], os.path.join(ros_root, 'core', 'roslib', 'src'))

        # - make sure it didn't pollute original env
        self.assertEquals(env['ROS_ROOT'], '/not/ros/root/')
        self.assertEquals(env['PYTHONPATH'], '/some/path')
        
        # don't set ROS_ROOT and ROS_PACKAGE_PATH. ROS_ROOT should default, ROS_PACKAGE_PATH does not
        m = Machine('name1', '', '', '1.2.3.4')
        d = setup_local_process_env(n, m, master_uri)
        val = os.environ['ROS_ROOT']
        self.assertEquals(d['ROS_ROOT'], val)
        self.assertEquals(d['PYTHONPATH'], os.path.join(val, 'core', 'roslib', 'src'))        
        self.failIf('ROS_PACKAGE_PATH' in d, 'ROS_PACKAGE_PATH should not be set: %s'%d)

        # test ROS_IP
        m = Machine('name1', ros_root, rpp, '1.2.3.4', ros_ip="4.5.6.7")
        n = Node('nodepkg','nodetype', namespace="/ns1")
        d = setup_local_process_env(n, m, master_uri)
        self.assertEquals(d['ROS_NAMESPACE'], "/ns1")
        # test stripping
        n = Node('nodepkg','nodetype', namespace="/ns2/")
        d = setup_local_process_env(n, m, master_uri)
        self.assertEquals(d['ROS_NAMESPACE'], "/ns2")

        # test ROS_NAMESPACE
        m = Machine('name1', ros_root, rpp, '1.2.3.4', ros_ip="4.5.6.7")
        d = setup_local_process_env(n, m, master_uri)
        self.assertEquals(d['ROS_IP'], "4.5.6.7")

        # test node.env_args
        n = Node('nodepkg','nodetype', env_args=[('NENV1', 'val1'), ('NENV2', 'val2'), ('ROS_ROOT', '/new/root')])        
        d = setup_local_process_env(n, m, master_uri)
        self.assertEquals(d['ROS_ROOT'], "/new/root")
        self.assertEquals(d['NENV1'], "val1")
        self.assertEquals(d['NENV2'], "val2")

        # test machine.env_args
        m = Machine('name1', ros_root, rpp, '1.2.3.4', ros_ip="4.5.6.7", env_args=[('MENV1', 'val1'), ('MENV2', 'val2'), ('ROS_ROOT', '/new/root2')])        
        n = Node('nodepkg','nodetype')
        d = setup_local_process_env(n, m, master_uri)
        self.assertEquals(d['ROS_ROOT'], "/new/root2")
        self.assertEquals(d['MENV1'], "val1")
        self.assertEquals(d['MENV2'], "val2")

        # test node.env_args precedence
        m = Machine('name1', ros_root, rpp, '1.2.3.4', ros_ip="4.5.6.7", env_args=[('MENV1', 'val1'), ('MENV2', 'val2')])
        n = Node('nodepkg','nodetype', env_args=[('MENV1', 'nodeval1')])
        d = setup_local_process_env(n, m, master_uri)
        self.assertEquals(d['MENV1'], "nodeval1")
        self.assertEquals(d['MENV2'], "val2")
        
if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_roslaunch', NAME, TestNodeprocess, coverage_packages=['roslaunch.nodeprocess'])
    
