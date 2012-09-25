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

import os
import sys
import string
import xmlrpclib

import rospy
from rostest import *

singletest = None
#singletest = 'testSourceKillFlow'

#TODO: test single source to multiple sinks
#TODO: test multiple sources to single sink

# test_string_in
# test_string_out

# test_primitives_in
# test_primitives_out

# test_arrays_in
# test_arrays_out

# test_header_in
# test_header_out

_required_publications  = 'test_string_out', 'test_primitives_out', 'test_arrays_out', 'test_header_out'
_required_subscriptions = 'test_string_in', 'test_primitives_in', 'test_arrays_in', 'test_header_in'

## Expects a single test node to be running with name 'test_node' and subscribed to 'test_string'
class SlaveTestCase(TestRosClient):

    def setUp(self):
        super(SlaveTestCase, self).setUp()
        # retrieve handle on node
        self.caller_id = rospy.get_caller_id()
        self.node_api = self.apiSuccess(self.master.lookupNode(self.caller_id, 'test_node'))
        self.assert_(self.node_api.startswith('http'))
        self.node = xmlrpclib.ServerProxy(self.node_api)
        
    def testGetPid(self):
        pid = self.apiSuccess(self.node.getPid(self.caller_id))
        self.assert_(pid > 0)
        
    def testGetPublications(self):
        publications = self.apiSuccess(self.node.getPublications(self.caller_id))
        self.assert_(publications is not None)
        expected = [rospy.resolve_name(t) for t in _required_publications]
        missing = set(expected) - set(publications) 
        self.failIf(len(missing), 'missing required topics: %s'%(','.join(missing)))

    def _subTestSourceRequestFlow(self, testName, protocols, testEval):
        master = self.master
        tests = [
            [['testSourceRequestFlow-%s-nodeA','testSourceRequestFlow-%s-nodeB',],
             ['node', 'testSourceRequestFlow-%s-nodeA.out', 'testSourceRequestFlow-%s-nodeB.in']],
            [['g1.testSourceRequestFlow-%s-nodeA','g1.testSourceRequestFlow-%s-nodeB',],
             ['g1.node', 'testSourceRequestFlow-%s-nodeA.out', 'testSourceRequestFlow-%s-nodeB.in']],
            [['g1.g2.g3.testSourceRequestFlow-%s-nodeA','g1.g2.testSourceRequestFlow-%s-nodeB',],
             ['g1.g2.node', 'g3.testSourceRequestFlow-%s-nodeA.out', 'testSourceRequestFlow-%s-nodeB.in']],
            [['g1.g2.testSourceRequestFlow-%s-nodeA','g1.g2.g3.testSourceRequestFlow-%s-nodeB',],
             ['g1.g2.node', 'testSourceRequestFlow-%s-nodeA.out', 'g3.testSourceRequestFlow-%s-nodeB.in']],
            ]
        sources = {}
        #start the nodes
        # - save the source as we will be making calls on it
        pkg, node = testNode
        for test in tests:
            sourceName, sinkName = [val%testName for val in test[0]]
            port = apiSuccess(master.addNode('', '', sourceName, pkg, node, TEST_MACHINE, 0))
            apiSuccess(master.addNode('', '', sinkName, pkg, node, TEST_MACHINE, 0))
            sourceUri = 'http://%s:%s/'%(testNodeAddr[0], port)
            sources[sourceName] = xmlrpclib.ServerProxy(sourceUri)

        for test in tests:
            sourceName, sinkName = [val%testName for val in test[0]]
            source = sources[sourceName]
            callerId = test[1][0]
            sourceLocator, sinkLocator = [val%testName for val in test[1][1:]]
            args = source.sourceRequestFlow(callerId, sourceLocator, sinkLocator, protocols)
            testEval(args)
        #TODO: test locator name resolution            
    
    def testSourceRequestFlow_TCPROS1(self):
        def testEval(args):
            protocol = apiSuccess(args)
            assert type(protocol) == list
            assert string.upper(protocol[0]) == 'TCPROS', "source should have returned TCPROS as the desired protocol"
            assert len(protocol) == 3, "TCPROS parameter spec should be length 3"
        protocols = [['TCPROS']]
        self._subTestSourceRequestFlow('TCPROS1', protocols, testEval)

    def testSourceRequestFlow_TCPROS2(self):
        def testEval(args):
            protocol = apiSuccess(args)
            assert type(protocol) == list            
            assert string.upper(protocol[0]) == 'TCPROS', "source should have returned TCPROS as the desired protocol"
            assert len(protocol) == 3, "TCPROS parameter spec should be length 3"
        protocols = [['Fake1', 123, 132], ['Fake2', 1.0], ['Fake3'], ['Fake4', 'string'], ['Fake5', ['a', 'list'], ['a', 'nother', 'list']], ['TCPROS'], ['Fakelast', 'fake'] ]
        self._subTestSourceRequestFlow('TCPROS2', protocols, testEval)

    def testSourceRequestFlow_Fail(self):
        protocols = [['Fake1', 123, 132], ['Fake2', 1.0], ['Fake3'], ['Fake4', 'string'], ['Fake5', ['a', 'list'], ['a', 'nother', 'list']], ['Fakelast', 'fake'] ]
        self._subTestSourceRequestFlow('Fail', protocols, apiFail)

    def testSourceRequestFlow_Errors(self):
        slave = self.slave
        master = self.master
        #test that malformed locators return error codes
        apiError(slave.sourceRequestFlow('', '', ''))
        apiError(slave.sourceRequestFlow('', 'good.locator', 'badlocator'))
        apiError(slave.sourceRequestFlow('', 'badlocator', 'good.locator'))
                
# sourceKillFlow(callerId, sourceLocator, sinkLocator)
#
# * called by master
# * returns int

    def testSourceKillFlow(self):
        slave = self.slave
        master = self.master
        #test that malformed locators return error codes
        apiError(slave.sourceKillFlow('', '', ''))
        apiError(slave.sourceKillFlow('', 'good.locator', 'badlocator'))
        apiError(slave.sourceKillFlow('', 'badlocator', 'good.locator'))
        
        tests = [
            [['testSourceKillFlow-nodeA','testSourceKillFlow-nodeB',],
             ['node', 'testSourceKillFlow-nodeA.out', 'testSourceKillFlow-nodeB.in']],
            [['g1.testSourceKillFlow-nodeA','g1.testSourceKillFlow-nodeB',],
             ['g1.node', 'testSourceKillFlow-nodeA.out', 'testSourceKillFlow-nodeB.in']],
            [['g1.g2.g3.testSourceKillFlow-nodeA','g1.g2.g3.testSourceKillFlow-nodeB',],
             ['g1.g2.node', 'g3.testSourceKillFlow-nodeA.out', 'g3.testSourceKillFlow-nodeB.in']],
            [['g1.g2.testSourceKillFlow-nodeA','g1.g2.testSourceKillFlow-nodeB',],
             ['g1.g2.node', 'testSourceKillFlow-nodeA.out', 'testSourceKillFlow-nodeB.in']],
            [['a1.g2.g3.testSourceKillFlow-nodeA','a1.g2.testSourceKillFlow-nodeB',],
             ['a1.node', 'g2.g3.testSourceKillFlow-nodeA.out', 'g2.testSourceKillFlow-nodeB.in']],
            [['a1.g2.testSourceKillFlow-nodeA','a1.g2.g3.testSourceKillFlow-nodeB',],
             ['a1.node', 'g2.testSourceKillFlow-nodeA.out', 'g2.g3.testSourceKillFlow-nodeB.in']],

            ]
        sources = {}
        #start the flows
        # - save the source as we will be making calls on it
        pkg, node = testNode
        for test in tests:
            sourceName, sinkName = test[0]
            # - start the source/sink nodes
            port = apiSuccess(master.addNode('', '', sourceName, pkg, node, TEST_MACHINE, 0))
            apiSuccess(master.addNode('', '', sinkName, pkg, node, TEST_MACHINE, 0))
            sourceUri = 'http://%s:%s/'%(testNodeAddr[0], port)
            sources[sourceName] = xmlrpclib.ServerProxy(sourceUri)
            # - start the flow
            callerId, sourceLocator, sinkLocator = test[1]
            apiSuccess(master.connectFlow(callerId, sourceLocator, sinkLocator, 1))
            
        #attempt to kill flow with 1 wrong endpoint
        for test in tests:
            source = sources[test[0][0]]
            callerId, sourceLocator, sinkLocator = test[1]
            #sourceKill flow does a silent succeed if there is no flow to sink, but returns
            #0 flows killed
            val = apiSuccess(source.sourceKillFlow(callerId, sourceLocator, 'fake.sink'))
            assert val == 0, "flowsKilled should be 0 for non-existent flow [fakesink]"
            #sourceKill flow fails if source param does not refer to it
            apiError(source.sourceKillFlow(callerId, 'fake.source', sinkLocator))
            
        #attempt to kill flow with wrong context
        for test in tests:
            source = sources[test[0][0]]
            callerId, sourceL, sinkL = test[1]
            for sub_test in tests:
                sub_callerId, sub_source, sub_sink = test[1]
                if sub_callerId == callerId and sub_source == sourceL and sub_sink == sinkL:
                    continue
                val = apiSuccess(source.sourceKillFlow(sub_callerId, sub_source, sub_sink))
                assert val == 0, "flowsKilled should be 0 for non-existent flow [different context: %s,%s,%s on %s,%s,%s]"%(sub_callerId, sub_source, sub_sink, callerId, sourceL, sinkL)
                
        # try to kill all flows on the source that we started
        for test in tests:
            source = sources[test[0][0]]
            callerId, sourceLocator, sinkLocator = test[1]
            val = apiSuccess(source.sourceKillFlow(callerId, sourceLocator, sinkLocator))
            assert type(val) == int
            assert val == 1, "Source did not report 1 flow killed: %s, %s"%(val, getLastMsg())
        #TODO: test locator name resolution            
    
    sinkTests = [
        [['testSinkConnectFlow-nodeA','testSinkConnectFlow-nodeB',],
         ['node', 'testSinkConnectFlow-nodeA.out', 'testSinkConnectFlow-nodeB.in']],
        [['g1.testSinkConnectFlow-nodeA','g1.testSinkConnectFlow-nodeB',],
         ['g1.node', 'testSinkConnectFlow-nodeA.out', 'testSinkConnectFlow-nodeB.in']],
        [['g1.g2.g3.testSinkConnectFlow-nodeA','g1.g2.testSinkConnectFlow-nodeB',],
         ['g1.g2.node', 'g3.testSinkConnectFlow-nodeA.out', 'testSinkConnectFlow-nodeB.in']],
        [['g1.g2.testSinkConnectFlow-nodeA','g1.g2.g3.testSinkConnectFlow-nodeB',],
         ['g1.g2.node', 'testSinkConnectFlow-nodeA.out', 'g3.testSinkConnectFlow-nodeB.in']],
        #separate subgraphs
        [['a1.a2.testSinkConnectFlow-nodeA','b1.b2.testSinkConnectFlow-nodeB',],
         ['node', 'a1.a2.testSinkConnectFlow-nodeA.out', 'b1.b2.testSinkConnectFlow-nodeB.in']],
        # '.locator' resolution
        [['c1.node.testSinkConnectFlow-nodeA','c1.node2.testSinkConnectFlow-nodeB',],
         ['c1.node2', 'node.testSinkConnectFlow-nodeA.out', '.testSinkConnectFlow-nodeB.in']],

        ]

    def _sink_StartNodes(self, tests):
        """
        Test subroutine to startup all the nodes specified in the tests
        """
        master = self.master
        sourceUris = {}
        sinks = {}
        #start the nodes
        # - save the sink as we will be making calls on it
        pkg, node = testNode
        for test in tests:
            sourceName, sinkName = test[0]
            sourcePort = apiSuccess(master.addNode('', '', sourceName, pkg, node, TEST_MACHINE, 0))
            sinkPort = apiSuccess(master.addNode('', '', sinkName, pkg, node, TEST_MACHINE, 0))
            sourceUri = 'http://%s:%s/'%(testNodeAddr[0], sourcePort)
            sinkUri = 'http://%s:%s/'%(testNodeAddr[0], sinkPort)
            sourceUris[sourceName] = sourceUri
            sinks[sinkName] = xmlrpclib.ServerProxy(sinkUri)
        return sourceUris, sinks

    def _sink_StartFlows(self, tests, sourceUris, sinks):
        """
        subroutine to connect the flows specified in the tests, purely
        using the sink API.

        In the future it would be nice to verify that the flow truly
        exists, but for now trust what the sink says
        """
        reliable = 1 #don't have UDP yet
        for test in tests:
            sourceName, sinkName = test[0]
            sourceUri = sourceUris[sourceName]
            sink = sinks[sinkName]
            callerId, sourceLocator, sinkLocator = test[1]
            print "Testing", test
            val = apiSuccess(sink.sinkConnectFlow(callerId, sourceLocator, sinkLocator, sourceUri, reliable))
            assert type(val) == int

    def _sink_KillFlows(self, tests, sinks):
        """
        subroutine to kill the flows specified in the tests, purely
        using the sink API (i.e. source will still be running)

        In the future it would be nice to verify that the flow was
        killed, but for now trust what the sink says"""
        reliable = 1 #don't have UDP yet
        for test in tests:
            sourceName, sinkName = test[0]
            sink = sinks[sinkName]
            callerId, sourceLocator, sinkLocator = test[1]
            print "Testing", test
            assert 1 == apiSuccess(sink.sinkKillFlow(callerId, sourceLocator, sinkLocator)),\
                   "sink did not report 1 flow killed: %s, %s"%(getLastVal(), getLastMsg())
    
    #sinkConnectFlow(sourceLocator, sinkLocator, sourceXmlRpcURI, reliable)
    # * API call on the sink of a flow. Called by master.
    # * master requests that sink negotiate with the source to establish the flow
    # * returns meaningless int
    # * synchronous call, blocks until success/fail

    def testSinkConnectFlow(self):
        master = self.master
        sourceUris, sinks = self._sink_StartNodes(self.sinkTests)
        self._sink_StartFlows(self.sinkTests, sourceUris, sinks)

# sinkKillFlow(sourceLocator, sinkLocator)
# * called by master
    def testSinkKillFlow(self):
        slave = self.slave
        master = self.master
        apiError(slave.sinkKillFlow('', '', ''))
        
        #startup the standard sink tests
        sourceUris, sinks = self._sink_StartNodes(self.sinkTests)
        self._sink_StartFlows(self.sinkTests, sourceUris, sinks)
        self._sink_KillFlows(self.sinkTests, sinks)        
    
    def testMisc(self):
        slave = self.slave
        assert slave is not None, "slave is None"
        #getMasterUri
        masterUri = apiSuccess(slave.getMasterUri(''))
        assert getMasterUri() == masterUri or getMasterUriAlt() == masterUri, masterUri
        assert masterUri == apiSuccess(slave.getMasterUri('a.different.id'))
        #getPid
        pid = apiSuccess(slave.getPid(''))
        assert pid == apiSuccess(slave.getPid('a.different.id'))
        #callerId must be string
        apiError(slave.getPid(0))
        apiError(slave.getMasterUri(0))        
        try:
            slave.shutdown('some.id')
        except:
            pass
        time.sleep(0.1)
        try:
            code, status, val = slave.getPid('')
            assert code < 1, "Slave is still running after shutdown"
        except:
            pass
            
