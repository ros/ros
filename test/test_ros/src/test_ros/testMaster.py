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
"""
testMaster: ROS integration test cases for master XML-RPC API

To run, invoke nodes/testMaster
"""

import os, sys, getopt, traceback, logging, socket
import datetime, xmlrpclib, math, random
import unittest
import rospy
from test_ros.rostest import *
from test_ros.testSlave import msMain

MYPKG = 'test_ros'

HAS_PARAM = True

singletest = 'testGetFlowNames'
singletest = None
#singletest = 'testDotLocalNames'

def verifyNodeAddress(master, callerId, name, machine, addr, port):
    if not name:
        raise Exception("name is None")
    rmachine, raddr, rport = apiSuccess(master.getNodeAddress(callerId, name))
    if machine:
        assert rmachine == machine, "Node [%s] is running on '%s' instead of '%s'"%(name, rmachine, machine)
    if port:
        assert rport == port, "Node [%s] is running on '%s' instead of '%s'"%(name, rport, port)
    else:
        assert rport, "Node [%s] does not have a registered port"%name
    if addr:
        if addr == 'localhost':
            addr = socket.gethostname()
        if raddr == 'localhost':
            raddr = socket.gethostname() 
        assert socket.gethostbyname(raddr) == socket.gethostbyname(addr), "%s!=%s"%(socket.gethostbyname(raddr), socket.gethostbyname(addr))
    #ping the node
    apiSuccess(xmlrpclib.ServerProxy("http://%s:%s/"%(raddr, rport)).getPid(''))

def testGraphState(master, graphNodes, graphFlows):
    graph = apiSuccess(master.getGraph(''))
    diff = set(graph[0]) ^ set(graphNodes)
    assert not diff, "Graph nodes %s does not match expected %s: %s"%(graph[0], graphNodes, diff)
    # stringify for comparison
    expectedFlows = ["%s:%s:1"%f for f in graphFlows] # :1 = connected
    print graph[1]
    remoteFlows = ["%s:%s:%s"%(src,snk,c) for (src,snk,c) in graph[1]]
    if expectedFlows or remoteFlows:
        #assert set(expectedFlows) ^ set(remoteFlows), "Graph flows [%s] does not match expected [%s]"%(graph[1], graphFlows)
        diff = set(expectedFlows) ^ set(remoteFlows)
        assert not diff, "Graph flows %s does not match expected %s: %s"%(expectedFlows, remoteFlows, diff)

def testParamState(master, myState):
    callerId = 'master' #validate from root 
    for (k, v) in myState.iteritems():
        if HAS_PARAM:
            assert apiSuccess(master.hasParam(callerId, k))
        print "verifying parameter %s"%k
        v2 = apiSuccess(master.getParam(callerId, k))
        if isinstance(v2, xmlrpclib.DateTime):
            assert xmlrpclib.DateTime(v) == v2, "[%s]: %s != %s, %s"%(k, v, v2, v2.__class__)
        else:
            assert v == v2, "[%s]: %s != %s, %s"%(k, v, v2, v2.__class__)
    paramNames = myState.keys()
    remoteParamNames = apiSuccess(master.getParamNames(callerId))
    assert not set(paramNames) ^ set(remoteParamNames), "parameter server keys do not match local" 

class ParamServerTestCase(ROSGraphTestCase):
    """Parameter Server API Test Cases"""
    
    def setUp(self):
        super(ParamServerTestCase, self).setUp()
    
    def tearDown(self):
        super(ParamServerTestCase, self).tearDown()        

    def _testSetParam(self, ctx, myState, testVals, master):
        for type, vals in testVals:
            try:
                if ctx:
                    callerId = "%s.node"%ctx
                else:
                    callerId = "node"
                count = 0
                for val in vals:
                    key = "%s-%s"%(type,count)
                    print "master.setParam(%s,%s,%s)"%(callerId, key, val)
                    master.setParam(callerId, key, val)
                    if HAS_PARAM:                    
                        assert apiSuccess(master.hasParam(callerId, key))
                    if ctx:
                        trueKey = "%s.%s"%(ctx, key)
                    else:
                        trueKey = key
                    myState[trueKey] = val
                    count += 1
            except:
                assert "getParam failed on type[%s], val[%s]"%(type,val)
        testParamState(master, myState)

    def testParamValues(self):
        """testParamValues: test storage of all XML-RPC compatible types"""
        from xmlrpclib import Binary
        testVals = [
            ['int', [0, 1024, 2147483647, -2147483647]],
            ['boolean', [True, False]],
            ['string', ['', '\0', 'x', 'hello', ''.join([chr(n) for n in xrange(0, 255)])]],
            ['double', [0.0, math.pi, -math.pi, 3.4028235e+38, -3.4028235e+38]],
            #TODO: microseconds?
            ['datetime', [datetime.datetime(2005, 12, 6, 12, 13, 14), datetime.datetime(1492, 12, 6, 12, 13, 14)]],
            ['base64', [Binary(''), Binary('\0'), Binary(''.join([chr(n) for n in xrange(0, 255)]))]],
            ['struct', [{ "a": 2, "b": 4},
                        {"a" : "b", "c" : "d"},
                        {"a" : {"b" : { "c" : "d"}}}]],
            ['array', [[], [1, 2, 3], ['a', 'b', 'c'], [0.0, 0.1, 0.2, 2.0, 2.1, -4.0],
                       [1, 'a', math.pi], [[1, 2, 3], ['a', 'b', 'c'], [1.0, 2.1, 3.2]]]
             ],
            ]
        master = self.master

        print "Putting parameters onto the server"
        # put our params into the parameter server
        contexts = ['', 'scope1', 'scope2', 'scope.subscope1', 'scope.sub1.sub2']
        myState = {}
        for ctx in contexts:
            self._testSetParam(ctx, myState, testVals, master)

        print "Deleting all of our parameters"
        # delete all of our parameters
        paramKeys = myState.keys()
        for key in paramKeys:
            apiSuccess(master.deleteParam('', key))
            del myState[key]
        testParamState(master, myState)

    def testEncapsulation(self):
        """testEncapsulation: test encapsulation: setting same parameter at different levels"""
        master = self.master
        myState = {}
        testParamState(master, myState)

        testContexts = ['', 'en', 'en.sub1', 'en.sub2', 'en.sub1.sub2']
        for c in testContexts:
            testKey = 'param1'
            testVal = random.randint(-1000000, 100000)
            if c:
                callerId = "%s.node"%c
                trueKey = "%s.%s"%(c,testKey)
            else:
                callerId ="node"
                trueKey = testKey
            master.setParam(callerId, testKey, testVal)
            myState[trueKey] = testVal
            # make sure the master has the parameter under both keys and that they are equal
            v1 = apiSuccess(master.getParam('', trueKey))
            v2 = apiSuccess(master.getParam(callerId, testKey))
            assert v1 == v2, "[%s]: %s vs. [%s,%s]: %s"%(trueKey, v1, callerId, testKey, v2)
            if HAS_PARAM:
                assert apiSuccess(master.hasParam(callerId, testKey)), testKey
                assert apiSuccess(master.hasParam('node', trueKey)), trueKey

        testParamState(master, myState)

    def testDotLocalNames(self):
        master = self.master
        myState = {}
        testParamState(master, myState)

        testContexts = ['', 'sub1', 'sub1.sub2', 'sub1.sub2.sub3']
        for c in testContexts:
            if c:
                callerId = "%s.node"%c
            else:
                callerId = "node"
            testKey = ".param1"
            testVal = random.randint(-1000000, 100000)            
            master.setParam(callerId, testKey, testVal)
            trueKey = callerId+testKey
            myState[trueKey] = testVal

            v1 = apiSuccess(master.getParam('node', trueKey))
            v2 = apiSuccess(master.getParam(callerId, testKey))
            assert v1 == v2, "[%s]: %s vs. [%s,%s]: %s"%(trueKey, v1, callerId, testKey, v2)
            if HAS_PARAM:
                assert apiSuccess(master.hasParam(callerId, testKey)), testKey
                assert apiSuccess(master.hasParam('node', trueKey)), trueKey

            #test setting a local param on a different node
            testKey = "altnode.param2"
            testVal = random.randint(-1000000, 100000)            
            master.setParam(callerId, testKey, testVal)
            if c:
                trueKey = "%s.%s"%(c,testKey)
                altCallerId = "%s.altnode"%c
            else:
                trueKey = testKey
                altCallerId = "altnode"
            myState[trueKey] = testVal

            v1 = apiSuccess(master.getParam(altCallerId, ".param2"))
            v2 = apiSuccess(master.getParam(callerId, testKey))
            assert v1 == v2
            if HAS_PARAM:
                assert apiSuccess(master.hasParam(callerId, testKey)), testKey
                assert apiSuccess(master.hasParam(altCallerId, ".param2"))

        testParamState(master, myState)

    def testScopeUp(self):
        """testScopeUp: test that parameter server can chain up scopes to find/delete parameters"""
        master = self.master
        myState = {}
        testParamState(master, myState)
        
        testVal = random.randint(-1000000, 100000)
        master.setParam('', 'uparam1', testVal)
        myState['uparam1'] = testVal 
        assert testVal == apiSuccess(master.getParam('node', 'uparam1'))
        assert testVal == apiSuccess(master.getParam('uptest.node', 'uparam1'))
        assert testVal == apiSuccess(master.getParam('uptest.sub1.node', 'uparam1'))
        assert testVal == apiSuccess(master.getParam('uptest.sub1.sub2.node', 'uparam1'))

        testVal = random.randint(-1000000, 100000)
        master.setParam('uptest2.sub1.node', 'uparam2', testVal)
        myState['uptest2.sub1.uparam2'] = testVal 
        assert testVal == apiSuccess(master.getParam('uptest2.sub1.node', 'uparam2'))
        assert testVal == apiSuccess(master.getParam('uptest2.sub1.sub2.node', 'uparam2'))
        assert testVal == apiSuccess(master.getParam('uptest2.sub1.sub2.sub3.node', 'uparam2'))
        testParamState(master, myState)

        #verify upwards deletion
        apiSuccess(master.deleteParam('uptest.sub1.sub2.node', 'uparam1'))
        del myState['uparam1']
        testParamState(master, myState)        
        apiSuccess(master.deleteParam('uptest2.sub1.sub2.sub3.node', 'uparam2'))
        del myState['uptest2.sub1.uparam2']
        testParamState(master, myState)        
        
    def testScopeDown(self):
        """testScopeDown: test scoping rules for sub contexts"""
        master = self.master
        myState = {}
        testParamState(master, myState)

        # test that parameter server down not chain down scopes
        testVal = random.randint(-1000000, 100000)
        master.setParam('down.one.two.three.node', 'dparam1', testVal)
        myState['down.one.two.three.dparam1'] = testVal
        if HAS_PARAM:
            assert not apiSuccess(master.hasParam('down.one', 'dparam1')) 
            assert not apiSuccess(master.hasParam('down.one.two', 'dparam1'))
        apiError(master.getParam('down.one.node', 'dparam1')) 
        apiError(master.getParam('down.one.two.node', 'dparam1'))

        # test that parameter server allows setting of parameters further down (1)
        testVal = random.randint(-1000000, 100000)
        master.setParam('node', 'down2.dparam2', testVal)
        myState['down2.dparam2'] = testVal         
        assert testVal == apiSuccess(master.getParam('down2.node', 'dparam2'))
        assert testVal == apiSuccess(master.getParam('', 'down2.dparam2'))
        if HAS_PARAM:
            assert not apiSuccess(master.hasParam('down2.node', 'down2.dparam2'))
        apiError(master.getParam('down2.node', 'down2.dparam2'))
        testParamState(master, myState)
        
        # test that parameter server allows setting of parameters further down (2)
        testVal = random.randint(-1000000, 100000)
        master.setParam('node', 'down3.sub.dparam3', testVal)
        myState['down3.sub.dparam3'] = testVal
        assert testVal == apiSuccess(master.getParam('down3.sub.node', 'dparam3'))
        assert testVal == apiSuccess(master.getParam('down3.node', 'sub.dparam3'))
        assert testVal == apiSuccess(master.getParam('', 'down3.sub.dparam3'))
        assert testVal == apiSuccess(master.getParam('down3.sub.sub2.node', 'dparam3'))
        if HAS_PARAM:        
            assert not apiSuccess(master.hasParam('down3.sub.node', 'sub.dparam3'))
            assert not apiSuccess(master.hasParam('down3.sub.node', 'down3.sub.dparam3'))
        apiError(master.getParam('down3.sub.node', 'sub.dparam3'))
        apiError(master.getParam('down3.sub.node', 'down3.sub.dparam3'))
        testParamState(master, myState)

        # test downwards deletion
        master.setParam('node', 'down4.sub.dparam4A', testVal)
        apiSuccess(master.deleteParam('down4.sub.node', 'dparam4A'))
        if HAS_PARAM:        
            assert not apiSuccess(master.hasParam('down4.sub', 'dparam4A'))
        master.setParam('node', 'down4.sub.dparam4B', testVal)        
        apiSuccess(master.deleteParam('down4.node', 'sub.dparam4B'))
        if HAS_PARAM:        
            assert not apiSuccess(master.hasParam('down4.sub', 'dparam4B'))
        master.setParam('node', 'down4.sub.dparam4C', testVal)
        apiSuccess(master.deleteParam('', 'down4.sub.dparam4C'))
        if HAS_PARAM:        
            assert not apiSuccess(master.hasParam('down4.sub.node', 'dparam4C'))
        testParamState(master, myState)
        
class MasterTestCase(ROSGraphTestCase):
    """Master API Test Cases -- those not covered by ParamServer and AddKillNode"""

    def setUp(self):
        super(MasterTestCase, self).setUp()
    
    def tearDown(self):
        super(MasterTestCase, self).tearDown() 

    def _verifyFlowNameState(self, master, state):
        flows = apiSuccess(master.getFlowNames('node1', ''))
        assert len(flows) == len(state.values()), "Master reported a different number of flows"
        for val in state.itervalues():
            assert val in flows, "flows does not contain %s : %s"%(val, flows)

    def testPromoteFlow(self):
        master = self.master
        state = {}
        callerId = 'node1'
        type = 'common_flows/String'
        #setup node1 with outflows outflow1..3 and inflow1..3
        apiSuccess(master.registerNode(callerId, 'node1', 'localhost', 1234, []))
        for i in range(1, 4):
            for dir in ['inflow', 'outflow']:
                locator = 'node1:%s%s'%(dir, i)
                apiSuccess(master.registerFlow(callerId, 'node1', locator, dir, type))
                state[locator] = [locator, dir, type, 0]
        
        #test promote of a non-existent flow
        apiError(master.promoteFlow(callerId, 'node1:notAFlow', 'node1:newNotAFlow'))
        # - :outflow1 should resolve to parent, which is a non-existent flow
        apiError(master.promoteFlow(callerId, ':outflow1', 'node1:newNotAFlow'))        

        ## Play with Outflows
        #successfully promote outflow1 to global namespace
        dir = 'outflow'
        apiSuccess(master.promoteFlow(callerId, 'node1:outflow1', ':outflow1'))
        state[':outflow1'] = [':outflow1', dir, type, 1]        
        # - verify with .local syntax
        apiSuccess(master.promoteFlow(callerId, '.:outflow2', ':outflow2'))        
        state[':outflow2'] = [':outflow2', dir, type, 1]

        ## Play with Inflows
        #successfully promote inflow1 to the same namespace
        dir = 'inflow'
        tests = [
            ['node1:inflow1',  'node1:newInflow1'],
            ['node1:inflow2', 'sibling:inflow2'],
            ['node1:inflow3', 'node1:subgraph1:inflow3']
            ]
        for source, target in tests:
            apiSuccess(master.promoteFlow(callerId, source, target))
            state[target] = [target, dir, type, 1]
        
        self._verifyFlowNameState(master, state)

        #TODO: test promotion to an already promoted/registered flow name
        apiError(master.promoteFlow(callerId, 'node1:outflow1', 'node1:outflow2'))
        apiError(master.promoteFlow(callerId, 'node1:outflow2', ':outflow1'))
        apiError(master.promoteFlow(callerId, 'node1:inflow1', 'sibling:inflow2'))        

        #TODO: test promote of a promoted flow
        #TODO: test promote with both :flow and .:flow
        #TODO: test name resolution with subgraphs and callerIds better
    
    def testRegisterFlow(self):
        master = self.master
        state = {}
        type = 'common_flows/String'
        for callerId in ['node1', 'subgraph.node1', 'grandparent.parent.node1']:
            #setup node1 with outflows outflow1..3 and inflow1..3
            apiSuccess(master.registerNode(callerId, 'node1', 'localhost', 1234, []))
            for i in range(1, 4):
                for dir in ['inflow', 'outflow']:
                    if dir == 'inflow':
                        locator = 'node1:%s%s'%(dir, i)
                    else:
                        locator = '.:%s%s'%(dir, i) #test .local resolution
                    apiSuccess(master.registerFlow(callerId, 'node1', locator, dir, type))
                    realLocator = '%s:%s%s'%(callerId, dir, i)
                    state[realLocator] = [realLocator, dir, type, 0]
            #test register on a non-existent node
            apiError(master.registerFlow(callerId, 'notNode', 'notANode:outflow', 'outflow', type))
            #test register on bad locators
            apiError(master.registerFlow(callerId, 'node1', '', 'outflow', type))
            apiError(master.registerFlow(callerId, 'node1', 'badflow1', 'outflow', type))            
            #test register on a bad direction
            apiError(master.registerFlow(callerId, 'node1', 'node1:badflow2', 'spiralflow', type))
            #test register on invalid types
            apiError(master.registerFlow(callerId, 'node1', 'node1:badflow3', 'outflow', ''))
            apiError(master.registerFlow(callerId, 'node1', 'node1:badflow4', 'outflow', 'faketype'))

            #test register to an already promoted/registered flow name
            apiError(master.registerFlow(callerId, 'node1', 'node1:outflow1', 'outflow', type))
            apiSuccess(master.promoteFlow(callerId, 'node1:outflow1', 'node1:newOutflow1'))
            state['%s:newOutflow1'%callerId] = ['node1:newOutflow1', dir, type, 1]            
            apiError(master.registerFlow(callerId, 'node1', 'node1:newOutflow1', 'outflow', type))

            self._verifyFlowNameState(master, state)

    def testUnregisterFlow(self):
        master = self.master
        state = {}
        type = 'common_flows/String'
        for callerId in ['node1', 'subgraph.node1', 'grandparent.parent.node1']:
            #setup node1 with outflows outflow1..3 and inflow1..3
            apiSuccess(master.registerNode(callerId, 'node1', 'localhost', 1234, []))
            for i in range(1, 4):
                for dir in ['inflow', 'outflow']:
                    if dir == 'inflow':
                        locator = rlocator = 'node1:%s%s'%(dir, i)
                    else:
                        locator = '.:%s%s'%(dir, i) #test .local resolution
                        rlocator = 'node1:%s%s'%(dir, i) #test .local resolution
                    apiSuccess(master.registerFlow(callerId, 'node1', locator, dir, type))
                    realLocator = '%s:%s%s'%(callerId, dir, i)
                    state[realLocator] = [realLocator, dir, type, 0]

            self._verifyFlowNameState(master, state)
            
            #test bad unregisters
            apiError(master.unregisterFlow(callerId, 'notANode:outflow'))
            apiError(master.unregisterFlow(callerId, ''))
            apiError(master.unregisterFlow(callerId.replace('node1', 'node2'), 'outflow1'))

            apiSuccess(master.unregisterFlow(callerId, 'node1:outflow1'))
            del state['%s:outflow1'%callerId]
            apiSuccess(master.unregisterFlow(callerId, '.:outflow2'))
            del state['%s:outflow2'%callerId]            
            apiSuccess(master.unregisterFlow(callerId.replace('node1', 'node2'), 'node1:outflow3'))
            del state['%s:outflow3'%callerId]             
            apiSuccess(master.unregisterFlow('master', '%s:inflow1'%callerId))
            del state['%s:inflow1'%callerId]

            self._verifyFlowNameState(master, state)
            
            #test register to an already unregistered flow
            apiError(master.unregisterFlow(callerId, 'node1:outflow1'))

            #test transitive unregisters
            apiSuccess(master.promoteFlow(callerId, 'node1:inflow3', ':inflow3A'))
            apiSuccess(master.promoteFlow(callerId, ':inflow3A', ':inflow3B'))
            # - should unregister both 3A and 3B
            apiSuccess(master.unregisterFlow(callerId, 'node1:inflow3'))
            del state['%s:inflow3'%callerId]
            
            self._verifyFlowNameState(master, state)
    
    def testGetFlowNames(self):
        master = self.master
        pkg,node = testNode
        subgraph = 'sub1.sub2'
        name = 'testGFN_A'
        port = apiSuccess(master.addNode('caller', subgraph, name, pkg, node, TEST_MACHINE, 0))        
        #testNode must have :in and :out
        #:in and :out
        testFlowNames = ["%s:%s"%(name, v) for v in ["in", "out"]]
        print "FLOW NAMES", apiSuccess(master.getFlowNames('master', ''))        
        flows = apiSuccess(master.getFlowNames('%s.caller'%subgraph, ''))
        assert not set([x[0] for x in flows]) ^ set(testFlowNames), "%s vs. %s"%([x[0] for x in flows], testFlowNames)
        
        inDirs = [x[1] for x in flows if x[0].endswith(':in')]
        outDirs = [x[1] for x in flows if x[0].endswith(':out')]        
        assert not filter(lambda x: x != "inflow", inDirs), inDirs
        assert not filter(lambda x: x != "outflow", outDirs), outDirs
        assert not filter(lambda x: x != "common_flows/String", [x[2] for x in flows]) #all flow types should be String
        assert not filter(lambda x: x, [x[3] for x in flows]) #promoted flag should all be zero

        #test callerId scoping and subgraph parameter
        testFlowNames = ["%s.%s:%s"%(subgraph, name, v) for v in ["in", "out"]]        
        flows = apiSuccess(master.getFlowNames('caller', subgraph))
        assert not set([x[0] for x in flows]) ^ set(testFlowNames), "%s vs. %s"%([x[0] for x in flows], testFlowNames)
        flows = apiSuccess(master.getFlowNames('caller', "%s.%s"%(subgraph, name)))
        assert not set([x[0] for x in flows]) ^ set(testFlowNames), "%s vs. %s"%([x[0] for x in flows], testFlowNames)

        testFlowNames = ["sub2.%s:%s"%(name, v) for v in ["in", "out"]]        
        flows = apiSuccess(master.getFlowNames('sub1.caller', 'sub2'))
        assert not set([x[0] for x in flows]) ^ set(testFlowNames), "%s vs. %s"%([x[0] for x in flows], testFlowNames)
        

        testFlowNames = ["%s.%s:%s"%(subgraph, name, v) for v in ["in", "out"]]        
        flows = apiSuccess(master.getFlowNames('caller', ''))
        flowNames = [x[0] for x in flows]
        assert not set(flowNames) ^ set(testFlowNames)

    
    def testGetNodeAddress(self):
        def validate(val, callerId, name, port):
            assert len(val) == 3, "getNodeAddress did not return 3-element list for value field"
            assert type(val[0]) == str and type(val[1]) == str and type(val[2]) == int,\
                   "getNodeAddress did not return (str, str, int) for value field"
            verifyNodeAddress(master, callerId, name, TEST_MACHINE, 'localhost', port)
        
        #NOTE: this does not do full coverage on rospy, as many of the branch
        #conditions in rospy are inaccessible via external call (they required
        #corrupted data to be inserted into the master, which registerNode prevents)

        master = self.master
        #test that invalid case still meets type spec
        code, msg, val = master.getNodeAddress('', 'testGetNodeAddress-fake')
        assert code == -1, "getNodeAddress did not return failure code 0 for non-existent node"
        assert len(val) == 3, "getNodeAddress did not return 3-element list for value field in error case"
        assert type(val[0]) == str and type(val[1]) == str and type(val[2]) == int,\
               "getNodeAddress did not return (str, str, int) for value field in error case"

        #start a node to test valid case against
        name = 'testGetNodeAddress-1'
        port = 7981
        pkg, node = testNode
        apiSuccess(master.addNode('', '', name, pkg, node, TEST_MACHINE, port))
        val = apiSuccess(master.getNodeAddress('', name))
        validate(val, '', name, port)

        #verify Graph Resource Name scoping rules
        name = 'testGetNodeAddress-2'
        port = 7982
        apiSuccess(master.addNode('', 'gna1.gna2', name, pkg, node, TEST_MACHINE, port))
        #make sure we have a node
        tests = [
            #test exact name matches
            ['gna1.gna2.node', name],
            ['gna1.node', 'gna2.%s'%name],
            ['', 'gna1.gna2.%s'%name],
            #make sure that gNA searches upwards 
            ['gna1.gna2.gna3.node', name],
            ]
        for test in tests:
            callerId, name = test
            validate(apiSuccess(master.getNodeAddress(callerId, name)), callerId, name, port)
        
        #make sure that it gNA doesn't search upwards when subcontext is specified
        val = apiError(master.getNodeAddress('gna1.gna2', 'gna3.%s'%name))

    def _verifyNodeDead(self, port):
        testUri = "http://localhost:%s/"%port
        try:
            xmlrpclib.ServerProxy(testUri).getPid('node')
            self.fail("test node is still running")
        except:
            pass
        
    def testAddKillNode(self):
        """testAddKillNode: test adding then killing nodes"""
        #TODO: more test cases
        master = self.master
        pkg,node = testNode        
        apiError(master.killNode('node','nonexistentNode'))
        name = 'testAddKillA'
        port = apiSuccess(master.addNode('node', 'subgraph', name, pkg, node, TEST_MACHINE, 0))
        # - doesn't traverse across        
        apiError(master.killNode('different.subgraph.node', name))
        # - doesn't traverse down
        apiError(master.killNode('node', name))
        # - doesn't traverse up        
        apiError(master.killNode('subgraph.sub2.node', name))
        # - kill it
        apiSuccess(master.killNode('subgraph.node', name))

        # - push on the name resolution a bit
        tests = [['node', '', 'testAddKillB'],
                 ['node', 'g1.g2', 'testAddKillB'],
                 ['node', 'g1', 'testAddKillB'],
                 ['g1.g2.node', 'g3', 'testAddKillB'],
                 ]
        for callerId, subcontext, nodeName in tests:
            port = apiSuccess(master.addNode(callerId, subcontext, nodeName, pkg, node, TEST_MACHINE, 0))
            if subcontext:
                name = "%s.%s"%(subcontext, nodeName)
            else:
                name = nodeName
            apiSuccess(master.killNode(callerId, name))
            self._verifyNodeDead(port)

    def testAddNode(self):
        """testAddNode: test master.addNode(callerId, subcontext, name, pkg, pkgNode, machine, port)"""
        master = self.master
        graphNodes = ['master']
        graphFlows = []
        # Failure cases
        pkg, node = testNode
        errors = [
            # - subcontext
            ['', 12, 'testAddNodeError1', pkg, node, TEST_MACHINE, 0],
            #name
            ['', '', 123,                pkg, node, TEST_MACHINE, 0],
            # - invalid package implementation type
            ['', '', 'testAddNodeError2', 123,         node, TEST_MACHINE, 0],
            # - node impl name
            ['', '', 'testAddNodeError3', pkg, '',       TEST_MACHINE, 0],
            ['', '', 'testAddNodeError4', pkg, 123,      TEST_MACHINE, 0],
            # - machine parameter
            ['', '', 'testAddNodeError6', pkg, node, 'unknown', 0],
            ['', '', 'testAddNodeError7', pkg, 'noNode', 123, 0],
            # - port
            ['', '', 'testAddNodeError8', pkg, node, TEST_MACHINE, -80],
            ['', '', 'testAddNodeError9', pkg, node, TEST_MACHINE, "80"],
            ]
        for args in errors:
            try:
                apiError(master.addNode(*args))
            except Exception, e:
                self.fail("addNodeError case failed with args[%s] and exception [%s]"%(args, e))
        # - non-existent node implementation (this takes awhile)
        apiFail(master.addNode('', '', 'testAddNodeFail1', pkg, 'notANode', TEST_MACHINE, 0))

        testGraphState(master, graphNodes, graphFlows)
        tests = [[['','testAddNode1'], [TEST_MACHINE, 7980]],
                 [['','testAddNode2'], [TEST_MACHINE, 0]],
                 [['','testAddNode3'], ['', 0]],
                 [['','testAddNode4'], [TEST_MACHINE, 0]],
                 [['','testAddNode5'], [TEST_MACHINE, 0]],
                 [['','testAddNode6'], [TEST_MACHINE, 0]],
                 [['','testAddNode7'], [TEST_MACHINE, 0]],
                 # subcontext
                 [['push',                'testAddNode8'], [TEST_MACHINE, 0]],
                 [['push.one.two',        'testAddNode9'], [TEST_MACHINE, 0]],
                 [['stanford.addNodeTest','testAddNodeA'], [TEST_MACHINE, 0]],
                 [['wg.addNodeTest',      'testAddNodeA'], [TEST_MACHINE, 0]],
                 ]
        for fullname, args in tests:
            print "testAddNode: testing", fullname
            subcontext, name = fullname
            if subcontext:
                fullname = '%s.%s'%(subcontext, name)
            else:
                fullname = name
            machine, port = args
            apiSuccess(master.addNode('', subcontext, name, pkg, node, machine, port))
            verifyNodeAddress(master, '', fullname, machine, 'localhost', port)
            graphNodes.append(fullname)
            testGraphState(master, graphNodes, graphFlows)
            # duplicate call should succeed
            apiSuccess(master.addNode('', subcontext, name, pkg, node, machine, port))

        #TODO: more stress testing of name resolution with non-root callerIds
        
        # duplicate call with different params should fail
        port = apiSuccess(master.addNode('node', 'duplicate.test', 'nodeA', pkg, node, TEST_MACHINE, 0))
        apiError(master.addNode('node', 'duplicate.test', 'nodeA', pkg, node, TEST_MACHINE, port + 1))
        apiError(master.addNode('node', 'duplicate.test', 'nodeA', pkg+'foo', node, TEST_MACHINE, port))
        apiError(master.addNode('node', 'duplicate.test', 'nodeA', pkg, node+'foo', TEST_MACHINE, port))        
        #TODO: different machine
    
    def testGetGraph(self):
        #TODO: in ROS 2.0, graph will be scoped by callerId. For now its safe to assume
        #that we are implicitly testing getGraph()
        pass
    
    def testAddMachine(self):
        master = self.master
        host, port = masterAddr
        #test invalid calls on addMachine
        # - name
        apiError(master.addMachine('node', '', self.rosRoot, host, 22, '', ''))
        apiError(master.addMachine('node', 123, self.rosRoot, host, 22, '', ''))        
        # - ros root
        apiError(master.addMachine('node', 'name', '', host, 22, '', ''))
        apiError(master.addMachine('node', 'name', 123, host, 22, '', ''))        
        # - address
        apiError(master.addMachine('node', 'name', '', '', 22, '', ''))
        apiError(master.addMachine('node', 'name', '', 123, 22, '', ''))        
        # - ssh port 
        apiError(master.addMachine('node', 'name', '', host, -22, '', ''))
        apiError(master.addMachine('node', 'name', '', host, "22", '', ''))
        
        tests = [
            ['node', 'testAddMachine-1'],
            ['g1.node', 'testAddMachine-2'],
            ['g1.g2.g3.node', 'testAddMachine-3'],
            ['g1.g2.node', 'testAddMachine-4'],            
            ]
        rosRoot = self.rosRoot
        for callerId, m in tests:
            apiSuccess(master.addMachine(callerId, m, rosRoot, host, 22, '', ''))
            # - duplicate safe
            apiSuccess(master.addMachine(callerId, m, rosRoot, host, 22, '', ''))
            # - test error for each parameter being changed
            apiError(master.addMachine(callerId, m, rosRoot+'/foo/', host, 22, '', ''))
            apiError(master.addMachine(callerId, m, rosRoot, 'www.google.com', 22, '', ''))            
            apiError(master.addMachine(callerId, m, rosRoot, host, 21, '', ''))
            apiError(master.addMachine(callerId, m, rosRoot, host, 22, 'fake-user', ''))
            apiError(master.addMachine(callerId, m, rosRoot, host, 22, '', 'fake-password'))
            
        #TODO: rewrite once master has a method for interrogating machines

    def testRegisterNode_Flows(self):
        #TODO: test flows param
        pass

    def testRegisterNode(self):
        master = self.master
        flows = []
        # - invalid name
        apiError(master.registerNode('', '', 'localhost', 80, flows))
        # - invalid address
        apiError(master.registerNode('', 'registerNodeFail2', '', 80, flows))
        # - invalid ports
        apiError(master.registerNode('', 'registerNodeFail3', 'localhost', -80, flows))
        apiError(master.registerNode('', 'registerNodeFail4', 'localhost', 0, flows))        
        
        #implicitly test registerNode via local exec of test node
        # - this actually tests the slave as much as the master, but I don't want
        #   to call registerNode with correct parameters as it is ambiguous whether
        #   or not the master should verify that the slave node actually exists
        testCases = [
            ['', 'registerNodeExternal-1'],
            ['', 'rne.registerNodeExternal-2'],
            ['rne', 'registerNodeExternal-3'],
            ['', 'rne.rne2.rne3.registerNodeExternal-4'],
            ['rne', 'rne2.rne3.registerNodeExternal-5'],
            ['rne.rne2.rne3', 'registerNodeExternal-6'],
            ]
        for context, name in testCases:
            try:
                if context:
                    fullName = "%s.%s"%(context, name)
                    callerId = "%s.node"%context 
                else:
                    fullName = name
                    callerId = "node"
                startTestNode(fullName)
                # Block until node is responsive before continuing
                # - give test node 4 seconds to start
                timeoutT = time.time() + 4.0 
                node = getTestNode()
                val = None
                while time.time() < timeoutT and not val:
                    try:
                        _, _, val = node.getPid('')
                    except:
                        pass
                assert val, "unable to start test node for registerNode test case"
                # - we don't know the machine in this case
                verifyNodeAddress(master, callerId, name, None, testNodeAddr[0], testNodeAddr[1])
            finally:
                stopTestNode()
            
    def testConnectFlow(self):
        master = self.master
        pkg, node = testNode
        graphNodes = ['master']
        graphFlows = []
        machine = TEST_MACHINE
        for i in range(1, 5):
            apiSuccess(master.addNode('m', '', 'baseTcfNode-%s'%i, pkg, node, machine, 0))
            graphNodes.append('baseTcfNode-%s'%i) #for testing up leveling
            apiSuccess(master.addNode('m', '', 'tcfNode-%s'%i, pkg, node, machine, 0)) 
            graphNodes.append('tcfNode-%s'%i)
            apiSuccess(master.addNode('m', 'tcf1', 'tcfNode-%s'%i, pkg, node, machine, 0))
            graphNodes.append('tcf1.tcfNode-%s'%i)
            apiSuccess(master.addNode('m', 'tcf2', 'tcfNode-%s'%i, pkg, node, machine, 0))
            graphNodes.append('tcf2.tcfNode-%s'%i)
            apiSuccess(master.addNode('m', 'tcf1.sub1', 'tcfNode-%s'%i, pkg, node, machine, 0))
            graphNodes.append('tcf1.sub1.tcfNode-%s'%i)
            apiSuccess(master.addNode('m', 'tcf2.sub1', 'tcfNode-%s'%i, pkg, node, machine, 0))
            graphNodes.append('tcf2.sub1.tcfNode-%s'%i)
            apiSuccess(master.addNode('m', 'tcf3', 'tcfNode3-%s'%i, pkg, node, machine, 0))
            graphNodes.append('tcf3.tcfNode3-%s'%i)
            testGraphState(master, graphNodes, graphFlows)
            
        reliable = 1

        # illegal cases
        illegal = [
            # - name resolution scope
            ['node.tcf1',   'baseTcfNode-1:out', 'tcfNode-1:in'],
            ['tcf1.node',   'tcfNode-1:out', 'baseTcfNode-1:in'],
            ['tcf1.tcfNode-1', 'tcfNode-1:out', 'tcf1.tcfNode-1:in'],
            ['tcf1.tcfNode-1', ':out', 'tcf1.tcfNode-1:in'],
            ['tcf1.tcfNode-1', 'tcf1.tcfNode-2:out', ':in'],                    
            ['tcf1.node',   'tcf1.tcfNode-1:out', 'tcfNode-1:in'],
            ['tcf1.node',   'tcf2.tcfNode-1:out', 'tcfNode-1:in'],
            ['tcf1.node',   'tcfNode-1:out', 'tcf2.tcfNode-1:in'],
            ['tcf1.node',   'sub1.tcfNode-1:out', 'tcf2.sub1.tcfNode-1:in'],
            ['tcf1.sub1.node','baseTcfNode-1:out', 'tcfNode-1:in'],
            ['tcf1.sub1.node','sub1.tcfNode-1:out', 'tcfNode-1:in'],
            ['tcf1.sub1.node','tcf2.tcfNode-1:out', 'tcfNode-1:in'],            
            ]
        for callerId, source, sink in illegal:
            apiError(master.connectFlow(callerId, source, sink, reliable))

        # single source to sink sink
        singleCase = [
            #straight forward cases
            ['node', 'tcfNode-1:out', 'tcfNode-2:in', 'tcfNode-1:out', 'tcfNode-2:in',],
            ['tcfNode-1','tcf1.tcfNode-1:out', 'tcf1.tcfNode-2:in', 'tcf1.tcfNode-1:out', 'tcf1.tcfNode-2:in'],
            ['tcf2.tcfNode-1', 'tcfNode-1:out', 'tcfNode-2:in', 'tcf2.tcfNode-1:out', 'tcf2.tcfNode-2:in'],
            ['tcf1.tcfNode-2', 'sub1.tcfNode-1:out', 'sub1.tcfNode-2:in', 'tcf1.sub1.tcfNode-1:out', 'tcf1.sub1.tcfNode-2:in'],
            ['tcf2.tcfNode-1', 'tcfNode-1:out', 'sub1.tcfNode-1:in', 'tcf2.tcfNode-1:out', 'tcf2.sub1.tcfNode-1:in'],
            
            # '.locator' naming test
            ['tcf2.sub1.tcfNode-1', '.:out', 'tcfNode-2:in', 'tcf2.sub1.tcfNode-1:out', 'tcf2.sub1.tcfNode-2:in'],
            ['tcf3.tcfNode3-2', 'tcfNode3-1:out', '.:in', 'tcf3.tcfNode3-1:out', 'tcf3.tcfNode3-2:in'], 
            ]
        for callerId, source, sink, sourceFull, sinkFull in singleCase:        
            apiSuccess(master.connectFlow(callerId, source, sink, reliable)) 
            graphFlows.append((sourceFull, sinkFull))
            testGraphState(master, graphNodes, graphFlows)            

        # - already connected
        # Spec is fuzzy here. It's possible that this should be a succeed.
        apiError(master.connectFlow('tcf2.node',   'tcfNode-1:out', 'tcfNode-2:in', reliable))        
        apiError(master.connectFlow('node',        'tcf1.tcfNode-1:out', 'tcf1.tcfNode-2:in', reliable))        
        
        #TODO: test single source to multiple sinks
        #TODO: test multiple sources to single sink


    def testKillFlow(self):
        master = self.master
        if 0:
            master.killFlow(callerId, source, sink)             

    def testMisc(self):
        master = self.master
        assert master is not None, "master is None"
        masterUri = apiSuccess(master.getMasterUri(''))
        assert masterUri == apiSuccess(master.getMasterUri('a.different.id')), master.getMasterUri('a.different.id')
        assert (getMasterUri() == masterUri) or \
               (getMasterUriAlt() == masterUri), masterUri
        #getPid
        pid = apiSuccess(master.getPid(''))
        assert pid == apiSuccess(master.getPid('a.different.id')), master.getPid('a.different.id')
        #callerId must be string
        apiError(master.getPid(0))
        apiError(master.getMasterUri(0))
        #shutdown
        try:
            master.shutdown('some.id')
        except:
            pass
        time.sleep(0.1)
        try:
            code, status, val = master.getPid('')
            assert code < 1, "Master is still running after shutdown"
        except:
            pass
        
def testMasterMain(argv, stdout, env):
    return msMain(argv, stdout, env, [ParamServerTestCase, MasterTestCase], singletest) 
        
if __name__ == '__main__':
    testMasterMain(sys.argv, sys.stdout, os.environ)

        
