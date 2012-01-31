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

import os
import sys
import unittest

import roslaunch.loader 
import roslaunch.xmlloader 

def get_test_path():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'xml'))

# path to example.launch directory
def get_example_path():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'resources'))

## Fake RosLaunch object
class RosLaunchMock(object):
    def __init__(self):
        self.nodes_core = []
        self.nodes = []
        self.tests = []
        self.params = []
        self.executables = []        
        self.clear_params = []        
        self.machines = []
        self.master = None
        self.config_errors = []        
        self.roslaunch_files = []
    def set_master(self, m):
        self.master = m
    def add_machine(self, m, verbose=True):
        self.machines.append(m)
    def add_roslaunch_file(self, f):
        self.roslaunch_files.append(f)
    def add_node(self, n, core=False, verbose=True):
        if not core:
            self.nodes.append(n)
        else:
            self.nodes_core.append(n)

    def add_config_error(self, msg):
        self.config_errors.append(msg)
        
    def add_test(self, t, verbose=True):
        self.tests.append(t)        
    def add_executable(self, t):
        self.executables.append(t)        

    def add_param(self, p, filename=None, verbose=True):
        matches = [x for x in self.params if x.key == p.key]
        for m in matches:
            self.params.remove(m)
        self.params.append(p)        
    def add_clear_param(self, param):
        self.clear_params.append(param)


## Test Roslaunch XML parser
class TestXmlLoader(unittest.TestCase):

    def setUp(self):
        self.xml_dir = get_test_path()
        
    def _load(self, test_file):
        loader = roslaunch.xmlloader.XmlLoader()
        mock = RosLaunchMock()
        self.assert_(os.path.exists(test_file), "cannot locate test file %s"%test_file)
        loader.load(test_file, mock)
        return mock
        
    def _load_valid_nodes(self, tests):
        mock = self._load(os.path.join(self.xml_dir, 'test-node-valid.xml'))
        nodes = [n for n in mock.nodes if n.type in tests]
        self.assertEquals(len(tests), len(nodes))
        return nodes

    def _load_valid_rostests(self, tests):
        mock = self._load(os.path.join(self.xml_dir, 'test-test-valid.xml'))
        nodes = [n for n in mock.tests if n.type in tests]
        self.assertEquals(len(tests), len(nodes))
        return nodes
    
    def _load_valid_machines(self, tests):
        mock = self._load(os.path.join(self.xml_dir, 'test-machine-valid.xml'))
        machines = [m for m in mock.machines if m.name in tests]
        self.assertEquals(len(tests), len(machines))
        return machines

    def test_load_string(self):
        # make sure load_string isn't broken
        loader = roslaunch.xmlloader.XmlLoader()
        mock = RosLaunchMock()

        # test with no <launch /> element
        # #1582: test error message as well
        try:
            loader.load_string('<foo />', mock)
            self.fail("no root lauch element passed")
        except Exception, e:
            self.assertEquals(str(e), "Invalid roslaunch XML syntax: no root <launch> tag")
        
        f = open(os.path.join(self.xml_dir, 'test-node-valid.xml'), 'r')
        try:
            s = f.read()
        finally:
            f.close()
        loader.load_string(s, mock)
        # sanity check
        self.assert_(mock.nodes)
        self.assert_([n for n in mock.nodes if n.type == 'test_base'])

        # check exception case
        f = open(os.path.join(self.xml_dir, 'invalid-xml.xml'), 'r')
        try:
            s = f.read()
        finally:
            f.close()
        try:
            loader.load_string(s, mock)            
            self.fail('load_string should have thrown an exception')
        except roslaunch.xmlloader.XmlParseException:
            pass

    def test_load(self):
        # make sure load isn't broken
        loader = roslaunch.xmlloader.XmlLoader()
        
        # test against empty data
        loader.load(os.path.join(self.xml_dir, 'test-valid.xml'), RosLaunchMock())

        # sanity check with real data
        mock = RosLaunchMock()

        loader.load(os.path.join(self.xml_dir, 'test-node-valid.xml'), mock)
        self.assert_(mock.nodes)
        self.assert_([n for n in mock.nodes if n.type == 'test_base'])

        # check exception case
        try:
            loader.load(os.path.join(self.xml_dir, 'invalid-xml.xml'), mock)            
            self.fail('load_string should have thrown an exception')
        except roslaunch.xmlloader.XmlParseException:
            pass

    def test_params_invalid(self):
        tests = ['test-params-invalid-%s.xml'%i for i in range(1, 6)]
        loader = roslaunch.xmlloader.XmlLoader()
        for filename in tests:
            filename = os.path.join(self.xml_dir, filename)
            try:
                self.assert_(os.path.exists(filename))
                loader.load(filename, RosLaunchMock())
                self.fail("xmlloader did not throw an xmlparseexception for [%s]"%filename)
            except roslaunch.xmlloader.XmlParseException, e:
                pass
            except roslaunch.loader.LoadException, e:
                pass

    def test_params(self):
        mock = self._load(os.path.join(self.xml_dir, 'test-params-valid.xml'))

        param_d = {}
        for p in mock.params:
            param_d[p.key] = p.value

        self.assertEquals('pass', param_d['/override'])
        self.assertEquals('bar2', param_d['/somestring1'])
        self.assertEquals('10', param_d['/somestring2'])
        self.assertEquals(1, param_d['/someinteger1'])
        self.assertEquals(2, param_d['/someinteger2'])
        self.assertAlmostEquals(3.14159, param_d['/somefloat1'], 2)
        self.assertAlmostEquals(5.0, param_d['/somefloat2'], 1)
        self.assertEquals("a child namespace parameter 1", param_d['/wg/wgchildparam'], p.value)
        self.assertEquals("a child namespace parameter 2", param_d['/wg2/wg2childparam1'], p.value)
        self.assertEquals("a child namespace parameter 3", param_d['/wg2/wg2childparam2'], p.value)

        import xmlrpclib
        f = open(os.path.join(get_example_path(), 'example.launch'))
        try:
            contents = f.read()
        finally:
            f.close()
        p = [p for p in mock.params if p.key == '/configfile'][0]
        self.assertEquals(contents, p.value, 1)
        p = [p for p in mock.params if p.key == '/binaryfile'][0]
        self.assertEquals(xmlrpclib.Binary(contents), p.value, 1)

        f = open(os.path.join(get_example_path(), 'example.launch'))
        try:
            contents = f.read()
        finally:
            f.close()
        p = [p for p in mock.params if p.key == '/commandoutput'][0]
        self.assertEquals(contents, p.value, 1)
        
        
    def test_rosparam_valid(self):
        mock = self._load(os.path.join(self.xml_dir, 'test-rosparam-valid.xml'))

        for prefix in ['', '/rosparam', '/node_rosparam']:
            p = [p for p in mock.params if p.key == prefix+'/string1'][0]
            self.assertEquals('bar', p.value)
            p = [p for p in mock.params if p.key == prefix+'/robots/childparam'][0]
            self.assertEquals('a child namespace parameter', p.value)

        p = [p for p in mock.params if p.key == '/node_rosparam/string1'][0]
        self.assertEquals('bar', p.value)
        p = [p for p in mock.params if p.key == '/node_rosparam/robots/childparam'][0]
        self.assertEquals('a child namespace parameter', p.value)
        
        exes = [e for e in mock.executables if e.command == 'rosparam']
        self.assertEquals(len(exes), 2, "expected 2 rosparam exes, got %s"%len(exes))
        from roslaunch.core import PHASE_SETUP
        for e in exes:
            self.assertEquals(PHASE_SETUP, e.phase)
            args = e.args
            self.assertEquals(3, len(args), "expected 3 args, got %s"%str(args))
            rp_cmd, rp_file, rp_ctx = args
            self.failIf('$(find' in rp_file, "file attribute was not evaluated")
            self.assertEquals('dump', rp_cmd)
                
            # verify that the context is passed in correctly
            if rp_file.endswith('dump.yaml'):
                self.assertEquals('/', rp_ctx)
            elif rp_file.endswith('dump2.yaml'):
                self.assertEquals('/rosparam/', rp_ctx)

        # test inline yaml examples
        p = [p for p in mock.params if p.key == '/inline_str'][0]
        self.assertEquals('value1', p.value)
        p = [p for p in mock.params if p.key == '/inline_list'][0]
        self.assertEquals([1, 2, 3, 4], p.value)
        p = [p for p in mock.params if p.key == '/inline_dict/key1'][0]
        self.assertEquals('value1', p.value)
        p = [p for p in mock.params if p.key == '/inline_dict/key2'][0]
        self.assertEquals('value2', p.value)
        p = [p for p in mock.params if p.key == '/inline_dict2/key3'][0]
        self.assertEquals('value3', p.value)
        p = [p for p in mock.params if p.key == '/inline_dict2/key4'][0]
        self.assertEquals('value4', p.value)

        # verify that later tags override 
        # - key2 is overriden
        self.assertEquals(1, len([p for p in mock.params if p.key == '/override/key1']))
        p = [p for p in mock.params if p.key == '/override/key1'][0]
        self.assertEquals('override1', p.value)
        # - key2 is not overriden
        p = [p for p in mock.params if p.key == '/override/key2'][0]
        self.assertEquals('value2', p.value)

        # verify that 'param' attribute is not required
        p = [p for p in mock.params if p.key == '/noparam1'][0]
        self.assertEquals('value1', p.value)
        p = [p for p in mock.params if p.key == '/noparam2'][0]
        self.assertEquals('value2', p.value)

        # #3580: test degree/rad conversions
        import math
        p = [p for p in mock.params if p.key == '/inline_degrees0'][0]
        self.assertAlmostEquals(0, p.value)
        p = [p for p in mock.params if p.key == '/inline_degrees180'][0]
        self.assertAlmostEquals(p.value, math.pi)
        p = [p for p in mock.params if p.key == '/inline_degrees360'][0]
        self.assertAlmostEquals(p.value, 2 * math.pi)

        p = [p for p in mock.params if p.key == '/dict_degrees/deg0'][0]
        self.assertAlmostEquals(0, p.value)
        p = [p for p in mock.params if p.key == '/dict_degrees/deg180'][0]
        self.assertAlmostEquals(p.value, math.pi)
        p = [p for p in mock.params if p.key == '/dict_degrees/deg360'][0]
        self.assertAlmostEquals(p.value, 2 * math.pi)

        p = [p for p in mock.params if p.key == '/inline_rad0'][0]
        self.assertAlmostEquals(0, p.value)
        p = [p for p in mock.params if p.key == '/inline_radpi'][0]
        self.assertAlmostEquals(p.value, math.pi)
        p = [p for p in mock.params if p.key == '/inline_rad2pi'][0]
        self.assertAlmostEquals(p.value, 2 * math.pi)

        p = [p for p in mock.params if p.key == '/dict_rad/rad0'][0]
        self.assertAlmostEquals(0, p.value)
        p = [p for p in mock.params if p.key == '/dict_rad/radpi'][0]
        self.assertAlmostEquals(p.value, math.pi)
        p = [p for p in mock.params if p.key == '/dict_rad/rad2pi'][0]
        self.assertAlmostEquals(p.value, 2 * math.pi)
                    
        # rosparam file also contains empty params
        mock = self._load(os.path.join(self.xml_dir, 'test-rosparam-empty.xml'))
        self.assertEquals([], mock.params)

    def test_rosparam_invalid(self):
        tests = ['test-rosparam-invalid-%s.xml'%i for i in range(1, 6)]
        loader = roslaunch.xmlloader.XmlLoader()
        for filename in tests:
            filename = os.path.join(self.xml_dir, filename)
            try:
                self.assert_(os.path.exists(filename))
                loader.load(filename, RosLaunchMock())
                self.fail("xmlloader did not throw an xmlloadexception for [%s]"%filename)
            except roslaunch.loader.LoadException, e:
                pass
        
    def test_node_valid(self):
        nodes = self._load_valid_nodes([])

    def test_rostest_valid(self):
        nodes = self._load_valid_rostests([])
        
    def test_node_rosparam_invalid(self):
        tests = ['test-node-rosparam-invalid-name.xml']
        loader = roslaunch.xmlloader.XmlLoader()
        for filename in tests:
            filename = os.path.join(self.xml_dir, filename)
            try:
                self.assert_(os.path.exists(filename))
                loader.load(filename, RosLaunchMock())
                self.fail("xmlloader did not throw an xmlparseexception for [%s]"%filename)
            except roslaunch.xmlloader.XmlParseException, e:
                pass

    def test_node_rosparam(self):
        from roslaunch.core import PHASE_SETUP

        tests = [("test-node-rosparam-load.xml", "test_node_rosparam_load"),
                 ("test-node-rosparam-dump.xml","test_node_rosparam_dump"),
                 ("test-node-rosparam-delete.xml","test_node_rosparam_delete"),
                 ("test-node-rosparam-load-multi.xml", "test_node_rosparam_multi"),
                 ("test-node-rosparam-load-param.xml", "test_node_rosparam_load_param"),
                 ("test-node-rosparam-load-ns.xml", "test_node_rosparam_load_ns")]
        for f, test in tests:
                 
            mock = self._load(os.path.join(self.xml_dir, f))
            nodes = [n for n in mock.nodes if n.type == test]
            self.assertEquals(1, len(nodes))
            n = nodes[0]
                          
            exes = [e for e in mock.executables if e.command == 'rosparam']

            if n.type == "test_node_rosparam_load":
                self.assertEquals(0, len(exes))
                p = [p for p in mock.params if p.key == '/rosparam_load/string1'][0]
                self.assertEquals('bar', p.value)
                p = [p for p in mock.params if p.key == '/rosparam_load/robots/childparam'][0]
                self.assertEquals('a child namespace parameter', p.value)
            elif n.type == "test_node_rosparam_delete":
                self.assertEquals(1, len(exes))
                self.assert_(len(exes[0].args) == 2, "invalid arg: %s"%(str(exes[0].args)))
                rp_cmd, rp_param = exes[0].args   
                self.assertEquals("delete", rp_cmd)
                self.assertEquals("/ns1/rosparam_delete/ns2/param", rp_param)
            elif n.type == "test_node_rosparam_dump":
                self.assertEquals(1, len(exes))
                rp_cmd, rp_file, rp_ctx = exes[0].args                
                self.assertEquals("dump", rp_cmd)
                self.assertEquals("dump.yaml", rp_file)
                self.assertEquals('/rosparam_dump/', rp_ctx)
            elif n.type == "test_node_rosparam_load_ns":
                self.assertEquals(0, len(exes))
                p = [p for p in mock.params if p.key == '/load_ns/subns/string1'][0]
                self.assertEquals('bar', p.value)
                p = [p for p in mock.params if p.key == '/load_ns/subns/robots/childparam'][0]
                self.assertEquals('a child namespace parameter', p.value)
            elif n.type == "test_node_rosparam_load_param":
                self.assertEquals(0, len(exes))
                p = [p for p in mock.params if p.key == '/load_param/param/string1'][0]
                self.assertEquals('bar', p.value)
                p = [p for p in mock.params if p.key == '/load_param/param/robots/childparam'][0]
                self.assertEquals('a child namespace parameter', p.value)
            elif n.type == "test_node_rosparam_multi":
                self.assertEquals(1, len(exes))
                e = exes[0]
                rp_cmd, rp_file, rp_ctx = e.args
                self.assertEquals("dump", rp_cmd)
                self.assertEquals("mdump.yaml", rp_file)
                self.assertEquals('/rosparam_multi/', rp_ctx)

                # test two other rosparam tags
                p = [p for p in mock.params if p.key == '/rosparam_multi/string1'][0]
                self.assertEquals('bar', p.value)
                p = [p for p in mock.params if p.key == '/rosparam_multi/robots/childparam'][0]
                self.assertEquals('a child namespace parameter', p.value)

                p = [p for p in mock.params if p.key == '/rosparam_multi/msubns/string1'][0]
                self.assertEquals('bar', p.value)
                p = [p for p in mock.params if p.key == '/rosparam_multi/msubns/robots/childparam'][0]
                self.assertEquals('a child namespace parameter', p.value)
                
    ## test that ~params in groups get applied to later members of group
    def test_local_param_group(self):
        mock = self._load(os.path.join(self.xml_dir, 'test-local-param-group.xml'))
        correct = [
            u'/group1/g1node1/gparam1',
            u'/group1/g1node2/gparam1',
            u'/group1/g1node2/gparam2',
            u'/node1/param1',
            ]
        p_names = [p.key for p in mock.params]
        self.assertEquals(set([]), set(correct) ^ set(p_names), "%s does not match %s"%(p_names, correct))

    def test_node_param(self):
        mock = self._load(os.path.join(self.xml_dir, 'test-node-valid.xml'))
        tests = [('/test_private_param1/foo1', 'bar1'),
                 ('/ns_test/test_private_param2/foo2', 'bar2'),
                 ('/test_private_param3/foo3', 'bar3'), ]
        for k, v in tests:
            p = [p for p in mock.params if p.key == k]
            self.assertEquals(1, len(p), "%s not present in parameters: %s"%(k, mock.params))
            self.assertEquals(v, p[0].value)
        node_types = [n.type for n in mock.nodes]
        
    def test_roslaunch_files(self):
        f = os.path.join(self.xml_dir, 'test-env.xml')
        f2 = os.path.join(self.xml_dir, 'test-env-include.xml')
        mock = self._load(f)
        mock_files = [os.path.realpath(x) for x in mock.roslaunch_files]
        actual_files = [os.path.realpath(x) for x in [f, f2]]
        self.assertEquals(len(set(actual_files)), len(set(mock_files)))

    def test_launch_prefix(self):
        nodes = self._load_valid_nodes(['test_launch_prefix'])
        self.assertEquals(1, len(nodes))
        self.assertEquals('xterm -e gdb --args', nodes[0].launch_prefix)
        nodes = self._load_valid_nodes(['test_base'])
        self.assertEquals(1, len(nodes))
        self.assertEquals(None, nodes[0].launch_prefix)

    def test_respawn(self):
        tests = ["test_respawn_true", "test_respawn_TRUE",
                 "test_respawn_false", "test_respawn_FALSE",
                 "test_respawn_none",]
        respawn_nodes = self._load_valid_nodes(tests)
        self.assertEquals(len(tests), len(respawn_nodes))
        for n in respawn_nodes:
            if n.type in ['test_respawn_true', 'test_respawn_TRUE']:
                self.assertEquals(True, n.respawn, "respawn for [%s] should be True"%n.type)
            else:
                self.assertEquals(False, n.respawn, "respawn for [%s] should be False"%n.type)

    def test_env_and_include(self):
        mock = self._load(os.path.join(self.xml_dir, 'test-env.xml'))
        expected = ['test_none', 'test_one', 'test_one_two', 'test_one_two_priv', 'test_one_two_include',]
        self.assertEquals(set(expected), set([n.type for n in mock.nodes]))
        for n in mock.nodes:
            if n.type == 'test_none':
                self.assertEquals([], n.env_args)
            elif n.type == 'test_one':
                self.assertEquals([("ONE", "one")], n.env_args)
            elif n.type == 'test_one_two':
                self.assert_(("ONE", "one") in n.env_args)
                self.assert_(("TWO", "two") in n.env_args)
            elif n.type == 'test_one_two_priv':
                self.assert_(("ONE", "one") in n.env_args)
                self.assert_(("TWO", "two") in n.env_args)
                self.assert_(("PRIVATE_TWO", "private_two") in n.env_args)
            elif n.type == 'test_one_two_include':
                self.assert_(("ONE", "one") in n.env_args)
                self.assert_(("TWO", "two") in n.env_args)
                self.assert_(("INCLUDE", "include") in n.env_args)
                
    def test_clear_params(self):
        true_tests = ["/test_clear_params1/","/test_clear_params2/",
                      "/clear_params_ns/test_clear_params_ns/",
                      "/group_test/","/embed_group_test/embedded_group/",
                      "/include_test/",
                      ]
        mock = self._load(os.path.join(self.xml_dir, 'test-clear-params.xml'))
        self.assertEquals(len(true_tests), len(mock.clear_params), "clear params did not match expected true: %s"%(str(mock.clear_params)))
        for t in true_tests:
            self.assert_(t in mock.clear_params, "%s was not marked for clear: %s"%(t, mock.clear_params))
                
    def test_clear_params_invalid(self):
        tests = ['test-clear-params-invalid-1.xml', 'test-clear-params-invalid-2.xml',
                 'test-clear-params-invalid-3.xml','test-clear-params-invalid-4.xml',]
        loader = roslaunch.xmlloader.XmlLoader()
        for filename in tests:
            filename = os.path.join(self.xml_dir, filename)
            try:
                self.assert_(os.path.exists(filename))
                loader.load(filename, RosLaunchMock())
                self.fail("xmlloader did not throw an xmlparseexception for [%s]"%filename)
            except roslaunch.xmlloader.XmlParseException, e:
                pass

    def _subtest_node_base(self, nodes):
        node = nodes[0]
        self.assertEquals("package", node.package)
        self.assertEquals("test_base", node.type)        

    def test_node_base(self):
        self._subtest_node_base(self._load_valid_nodes(['test_base']))
        tests = self._load_valid_rostests(['test_base'])
        self._subtest_node_base(tests)
        self.assertEquals('test1', tests[0].test_name)
        self.assertEquals(roslaunch.core.TEST_TIME_LIMIT_DEFAULT, tests[0].time_limit)
        
    def _subtest_node_args(self, nodes):
        for n in nodes:
            if n.type == 'test_args':
                self.assertEquals("args test", n.args)
            elif n.type == 'test_args_empty':
                self.assertEquals("", n.args)                

    def test_node_args(self):
        self._subtest_node_args(self._load_valid_nodes(['test_args', 'test_args_empty']))
        tests = self._load_valid_rostests(['test_args', 'test_args_empty'])
        self._subtest_node_args(tests)
        for n in tests:
            if n.type == 'test_args':
                self.assertEquals("test2", n.test_name)
            elif n.type == 'test_args_empty':
                self.assertEquals("test3", n.test_name)                
        
    def test_rostest_time_limit(self):
        tests = self._load_valid_rostests(['test_time_limit_int_1', 'test_time_limit_float_10_1'])
        for n in tests:
            if n.type == 'test_time_limit_int_1':
                self.assertAlmostEquals(1.0, n.time_limit, 3)
            elif n.type == 'test_time_limit_float_10_1':
                self.assertAlmostEquals(10.1, n.time_limit, 3)

    def test_rostest_retry(self):
        n = self._load_valid_rostests(['test_retry'])[0]
        self.assertEquals(2, n.retry)
                
    def test_node_cwd(self):
        nodes = self._load_valid_nodes(['test_base', 'test_cwd_2', 'test_cwd_3', 'test_cwd_4'])
        for n in nodes:
            if n.type == 'test_base':
                self.assertEquals(None, n.cwd)
            elif n.type == 'test_cwd_2':
                self.assertEquals("node", n.cwd)  
            elif n.type in ['test_cwd_3', 'test_cwd_4']:
                self.assertEquals("ROS_HOME", n.cwd)  

    def test_node_output(self):
        nodes = self._load_valid_nodes(['test_output_log', 'test_output_screen'])
        for n in nodes:
            if n.type == 'test_output_log':
                self.assertEquals("log", n.output)
            elif n.type == 'test_output_screen':
                self.assertEquals("screen", n.output) 

    def test_node_required(self):
        nodes = self._load_valid_nodes(['test_base',
                                        'test_required_true_1',
                                        'test_required_true_2',
                                        'test_required_false_1',
                                        'test_required_false_2',                                        
                                        ])
        for n in nodes:
            if n.type.startswith('test_required_true'):
                self.assertEquals(True, n.required)
            else:
                self.assertEquals(False, n.required) 

    def test_node_machine(self):
        nodes = self._load_valid_nodes(['test_machine'])
        node = nodes[0]
        self.assertEquals("machine_test", node.machine_name)

    def test_node_ns(self):
        nodes = self._load_valid_nodes(['test_ns1', 'test_ns2','test_ns3'])
        for n in nodes:
            if n.type == 'test_ns1':
                self.assertEquals("/ns_test1/", n.namespace)
            elif n.type == 'test_ns2':
                self.assertEquals("/ns_test2/child2/", n.namespace) 
            elif n.type == 'test_ns3':
                self.assertEquals("/ns_test3/child3/", n.namespace) 

    def test_machines(self):
        tests = ['test-machine-invalid.xml', \
                 'test-machine-invalid-4.xml', 'test-machine-invalid-5.xml',
                 'test-machine-invalid-6.xml', 'test-machine-invalid-7.xml',
                 'test-machine-invalid-8.xml', 'test-machine-invalid-9.xml', 
                 'test-machine-invalid-10.xml', 'test-machine-invalid-11.xml',
                 'test-machine-invalid-12.xml', 
                 ]
        loader = roslaunch.xmlloader.XmlLoader()
        for filename in tests:
            filename = os.path.join(self.xml_dir, filename)
            try:
                self.assert_(os.path.exists(filename))
                loader.load(filename, RosLaunchMock())
                self.fail("xmlloader did not throw an xmlparseexception for [%s]"%filename)
            except roslaunch.xmlloader.XmlParseException, e:
                pass

        machines = self._load_valid_machines(['machine1', 'machine6', 'machine7', 'machine8', 'machine9'])
        for m in machines:
            if m.name == 'machine1':
                self.assertEquals(m.address, 'address1')
            elif m.name == 'machine7':
                self.assertEquals(m.timeout, 10.0)
            elif m.name == 'machine8':
                self.assertEquals(m.timeout, 1.)
            elif m.name == 'machine9':
                self.assertEquals(m.env_loader, '/opt/ros/fuerte/env.sh')

                
    def test_node_subst(self):
        test_file =os.path.join(self.xml_dir, 'test-node-substitution.xml')
        keys = ['PACKAGE', 'TYPE', 'OUTPUT', 'RESPAWN']
        for k in keys:
            if k in os.environ:
                del os.environ[k]
        import random
        r = random.randint(0, 100000)
        if r%2:
            output = 'screen'
            respawn = 'true' 
        else:
            output = 'log'
            respawn = 'false'
        # append all but one required key and make sure we fail
        for k in keys[:-1]:
            if k in ['PACKAGE', 'TYPE']:
                os.environ[k] = "%s-%s"%(k.lower(), r)
            elif k == 'OUTPUT':
                os.environ['OUTPUT'] = output
            try:
                mock = self._load(test_file)
                self.fail("xml loader should have thrown an exception due to missing environment var")
            except roslaunch.xmlloader.XmlParseException, e:
                pass

        # load the last required env var
        os.environ['RESPAWN'] = respawn
        mock = self._load(test_file)
        self.assertEquals(1, len(mock.nodes), "should only be one test node")
        n = mock.nodes[0]
        self.assertEquals(n.package, 'package-%s'%r)
        self.assertEquals(n.type, 'type-%s'%r)
        self.assertEquals(n.output, output)
        if respawn == 'true':
            self.assert_(n.respawn)
        else:
            self.failIf(n.respawn)            

    def test_machine_subst(self):
        test_file = os.path.join(self.xml_dir, 'test-machine-substitution.xml')
        keys = ['NAME', 'ADDRESS'] 
        for k in keys:
            if k in os.environ:
                del os.environ[k]
        import random
        r = random.randint(0, 100000)
        # append all but one required key and make sure we fail
        for k in keys[:-1]:
            os.environ[k] = "%s-%s"%(k.lower(), r)
            try:
                mock = self._load(test_file)
                self.fail("xml loader should have thrown an exception due to missing environment var")
            except roslaunch.xmlloader.XmlParseException, e:
                pass

        os.environ['NAME'] = "name-foo"
        os.environ['ADDRESS'] = "address-foo"
        # load the last required env var
        mock = self._load(test_file)
        self.assertEquals(1, len(mock.machines), "should only be one test machine")
        m = mock.machines[0]
        self.assertEquals(m.name, 'name-foo')
        self.assertEquals(m.address, 'address-foo')

    def test_env(self):
        nodes = self._load_valid_nodes(['test_env', 'test_env_empty'])
        for n in nodes:
            if n.type == 'test_env':
                self.assert_(("env1", "env1 value1") in n.env_args)
                self.assert_(("env2", "env2 value2") in n.env_args)
                self.assertEquals(2, len(n.env_args))
            elif n.type == 'test_env_empty':
                self.assert_(("env1", "") in n.env_args)

    def test_remap(self):
        loader = roslaunch.xmlloader.XmlLoader()
        mock = RosLaunchMock()
        loader.load(os.path.join(self.xml_dir, 'test-remap-valid.xml'), mock)
        names = ["node%s"%i for i in xrange(1, 7)]
        nodes = [n for n in mock.nodes if n.type in names]
        for n in nodes:
            if n.type == 'node1':
                self.assertEquals([['foo', 'bar']], n.remap_args)
            elif n.type == 'node2':
                self.assertEquals([['foo', 'baz']], n.remap_args)
            elif n.type == 'node3':
                self.assertEquals([['foo', 'bar']], n.remap_args)
            elif n.type == 'node4':
                self.assertEquals([['foo', 'far']], n.remap_args)
            elif n.type == 'node5':
                self.assertEquals([['foo', 'fad'], ['a', 'b'], ['c', 'd']], n.remap_args)
            elif n.type == 'node6':
                self.assertEquals([['foo', 'far'], ['old1', 'new1'], ['old2', 'new2'], ['old3', 'new3']], n.remap_args)
                 
    def test_substitution(self):
        mock = self._load(os.path.join(self.xml_dir, 'test-substitution.xml'))
        # for now this is mostly a trip wire test due to #1776 
        for p in mock.params:
            self.assert_('$' not in p.key)
            self.assert_('$' not in p.value)            
        for n in mock.nodes:
            self.assert_('$' not in n.package)
            self.assert_('$' not in n.type)
            for e in n.env_args:
                self.assert_('$' not in e[0])
                self.assert_('$' not in e[1])
            for r in n.remap_args:
                self.assert_('$' not in r[0])
                self.assert_('$' not in r[1])
            for a in n.args:
                self.assert_('$' not in a)                        
    
    def test_node_invalid(self):
        tests = ['test-node-invalid-type.xml','test-node-invalid-type-2.xml',
                 'test-node-invalid-pkg.xml','test-node-invalid-pkg-2.xml',
                 # 1 and 2 have been disabled for now until we re-remove ability to have unnamed nodes with params
                 'test-node-invalid-name-1.xml',
                 'test-node-invalid-name-2.xml',
                 'test-node-invalid-name-3.xml',
                 'test-node-invalid-machine.xml',
                 'test-node-invalid-respawn.xml',
                 'test-node-invalid-respawn-required.xml',                 
                 'test-node-invalid-required-1.xml',
                 'test-node-invalid-required-2.xml',                  
                 'test-node-invalid-ns.xml','test-node-invalid-ns-2.xml',                 
                 'test-node-invalid-env-name.xml','test-node-invalid-env-name-2.xml',
                 'test-node-invalid-env-value.xml',
                 'test-node-invalid-output.xml',
                 'test-node-invalid-cwd.xml',                 
                 'test-node-invalid-exception.xml',

                 # rostest <test> tests
                 'test-test-invalid-reqd-1.xml',
                 'test-test-invalid-reqd-2.xml',                 
                 'test-test-invalid-respawn.xml',
                 'test-test-invalid-output.xml',
                 'test-test-invalid-time-limit-1.xml',
                 'test-test-invalid-time-limit-2.xml',
                 'test-test-invalid-retry.xml',
                 ]
        loader = roslaunch.xmlloader.XmlLoader()
        for filename in tests:
            filename = os.path.join(self.xml_dir, filename)
            try:
                self.assert_(os.path.exists(filename))
                loader.load(filename, RosLaunchMock())
                self.fail("xmlloader did not throw an xmlparseexception for [%s]"%filename)
            except roslaunch.xmlloader.XmlParseException, e:
                pass

    def test_remap_invalid(self):
        tests = ['test-remap-invalid-1.xml',
                 'test-remap-invalid-2.xml',
                 'test-remap-invalid-3.xml',
                 'test-remap-invalid-4.xml',                                  
                 'test-remap-invalid-name-from.xml',                 
                 'test-remap-invalid-name-to.xml',                 
                 ]
        loader = roslaunch.xmlloader.XmlLoader()
        for filename in tests:
            filename = os.path.join(self.xml_dir, filename)
            try:
                self.assert_(os.path.exists(filename))
                loader.load(filename, RosLaunchMock())
                self.fail("xmlloader did not throw an xmlparseexception for [%s]"%filename)
            except roslaunch.xmlloader.XmlParseException, e:
                pass

    def test_if_unless(self):
        mock = RosLaunchMock()
        loader = roslaunch.xmlloader.XmlLoader()
        filename = os.path.join(self.xml_dir, 'test-if-unless.xml')
        loader.load(filename, mock, argv=[])

        param_d = {}
        for p in mock.params:
            param_d[p.key] = p.value

        keys = ['group_if', 'group_unless', 'param_if', 'param_unless']
        for k in keys:
            self.assert_('/'+k+'_pass' in param_d, param_d)
            self.failIf('/'+k+'_fail' in param_d, k)

        n = mock.nodes[0]
        for k in ['if', 'unless']:
            self.assert_(['from_%s_pass'%k, 'to_%s_pass'%k] in n.remap_args)
            self.failIf(['from_%s_fail'%k, 'to_%s_fail'%k] in n.remap_args)            
        
    def test_if_unless_invalid(self):
        mock = RosLaunchMock()
        loader = roslaunch.xmlloader.XmlLoader()
        filename = os.path.join(self.xml_dir, 'test-if-unless-invalid-both.xml')
        # this should raise, not sure XmlParseException is what we want as it destroys semantic info
        try:
            loader.load(filename, mock, argv=[])
            self.fail("should have raised with invalid if and unless spec")
        except roslaunch.xmlloader.XmlParseException, e:
            self.assert_('unless' in str(e))
            self.assert_('if' in str(e))

    def test_arg_invalid(self):
        mock = RosLaunchMock()
        loader = roslaunch.xmlloader.XmlLoader()
        filename = os.path.join(self.xml_dir, 'test-arg.xml')
        # this should raise, not sure XmlParseException is what we want as it destroys semantic info
        try:
            loader.load(filename, mock, argv=[])
            self.fail("should have raised with missing arg")
        except roslaunch.xmlloader.XmlParseException, e:
            self.assert_('required' in str(e))

        # test with invalid $(arg unknown)
        filename = os.path.join(self.xml_dir, 'test-arg-invalid-sub.xml')
        try:
            loader.load(filename, mock, argv=[])
            self.fail("should have raised with unknown arg")
        except roslaunch.xmlloader.XmlParseException, e:
            self.assert_('missing' in str(e))

        # test with invalid $(arg unknown)
        filename = os.path.join(self.xml_dir, 'test-arg-invalid-redecl.xml')
        try:
            loader.load(filename, mock, argv=[])
            self.fail("should have raised with multiple decl")
        except roslaunch.xmlloader.XmlParseException, e:
            self.assert_('grounded' in str(e))
            
                    
    def test_arg(self):
        loader = roslaunch.xmlloader.XmlLoader()
        filename = os.path.join(self.xml_dir, 'test-arg.xml')

        mock = RosLaunchMock()
        loader.load(filename, mock, argv=["required:=test_arg", "if_test:=0"])

        param_d = {}
        for p in mock.params:
            param_d[p.key] = p.value

        self.assertEquals(param_d['/p1_test'], 'test_arg')
        self.assertEquals(param_d['/p2_test'], 'not_set')
        self.assertEquals(param_d['/p3_test'], 'set')
        self.assertEquals(param_d['/succeed'], 'yes')                
        self.assertEquals(param_d['/if_test'], 'not_ran')                
        self.assertEquals(param_d['/if_param'], False)   
        self.assertEquals(param_d['/int_param'], 1234)   
        self.assertAlmostEquals(param_d['/float_param'], 3.)   
        self.failIf('/fail' in param_d)

        # context tests
        #  - args are scoped to their context, and thus can be rebound in a sibling context
        self.assertEquals(param_d['/context1'], 'group1')
        self.assertEquals(param_d['/context2'], 'group2')        
        
        # include tests
        self.assertEquals(param_d['/include_test/p1_test'], 'required1')
        self.assertEquals(param_d['/include_test/p2_test'], 'not_set')
        self.assertEquals(param_d['/include_test/p3_test'], 'set')
        self.assertEquals(param_d['/include_test/p4_test'], 'initial')
        
        self.assertEquals(param_d['/include2/include_test/p1_test'], 'required2')
        self.assertEquals(param_d['/include2/include_test/p2_test'], 'optional2')
        self.assertEquals(param_d['/include2/include_test/p3_test'], 'set')
        self.assertEquals(param_d['/include2/include_test/p4_test'], 'new2')
            
        self.assert_('/include3/include_test/p1_test' not in param_d)
        self.assert_('/include3/include_test/p2_test' not in param_d)
        self.assert_('/include3/include_test/p3_test' not in param_d)
        self.assert_('/include3/include_test/p4_test' not in param_d)

        # test again with optional value set
        mock = RosLaunchMock()
        loader.load(filename, mock, argv=["required:=test_arg", "optional:=test_arg2", "if_test:=1"])

        param_d = {}
        for p in mock.params:
            param_d[p.key] = p.value
            
        self.assertEquals(param_d['/p1_test'], 'test_arg')
        self.assertEquals(param_d['/p2_test'], 'test_arg2')
        self.assertEquals(param_d['/p3_test'], 'set')
        self.assertEquals(param_d['/context1'], 'group1')
        self.assertEquals(param_d['/context2'], 'group2')                
        self.assertEquals(param_d['/succeed'], 'yes')                
        self.assertEquals(param_d['/if_test'], 'ran')   
        self.assertEquals(param_d['/if_param'], True)   
        self.failIf('/fail' in param_d)

        # include tests
        self.assertEquals(param_d['/include_test/p1_test'], 'required1')
        self.assertEquals(param_d['/include_test/p2_test'], 'not_set')
        self.assertEquals(param_d['/include_test/p3_test'], 'set')
        self.assertEquals(param_d['/include_test/p4_test'], 'initial')
        
        self.assertEquals(param_d['/include2/include_test/p1_test'], 'required2')
        self.assertEquals(param_d['/include2/include_test/p2_test'], 'optional2')
        self.assertEquals(param_d['/include2/include_test/p3_test'], 'set')
        self.assertEquals(param_d['/include2/include_test/p4_test'], 'new2')
        
        self.assertEquals(param_d['/include3/include_test/p1_test'], 'required3')
        self.assertEquals(param_d['/include3/include_test/p2_test'], 'optional3')
        self.assertEquals(param_d['/include3/include_test/p3_test'], 'set')
        self.assertEquals(param_d['/include3/include_test/p4_test'], 'new3')
        

