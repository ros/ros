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
ROS Test Framework
==================

ROS supports a Python unittest-based test framework for easily setting
up and tearing down Computation Graphs.

Current features:

 * Generates a new master for each test. Master can be accessed via L{getMaster}
 * Assertions for testing master/slave API return values (L{apiSuccess}/Fail/Error)
 * Starting a generic test node
 
This framework is fairly rudimentary for ROS 0.1 but will support many
new features in ROS 0.2 and beyond. Planned features include:

 * Scaffolders for tests (nodes and graphs)
 * Better log file management
 * Pretty reports
"""

import os, time, sys, getopt
import unittest
import xmlrpclib, socket, signal

import rospy

#TODO: #71 can't have hardcoded node port if we allow concurrent tests
masterAddr = ('localhost', 7979)

TIMEOUT = 15

# master/slave test config #####################################
# set the ros implementation to use in tests (master/slave)
# the slave implementation must have a .in inflow and .out outflow

# test config
masterPkg = 'rospy'
testNode = ['test_rospy', 'string_node']
#TODO: #71 can't have hardcoded node port if we allow concurrent tests
testNodeAddr = ('localhost', 8989)

def setMaster(pkg):
    global masterPkg
    masterPkg = pkg
    print "Using %s/nodes/master as master"%masterPkg

def setTestNode(pkg, node):
    global testNode
    testNode = (pkg, node)
    print "Using %s/%s as slave test node"%testNode

# ROS graph test case ##########################################

class ROSGraphTestCase(unittest.TestCase):
    """ROSGraphTestCase does common graph setup/teardown operations for
    graph-based test cases, i.e. test cases that require a master to
    be activated"""
        
    def setUp(self):
        # test case class + test id
        id = self.id()
        idx1 = id.rfind('.')
        masterUniqueId = id[id.rfind('.', 0, idx1)+1:]
        startMaster(masterUniqueId)
        waitForNewMaster(TIMEOUT)
        self.master = getMaster()
        self.rosRoot = getRosRoot()
        apiSuccess(
            self.master.addMachine('', TEST_MACHINE, self.rosRoot, masterAddr[0], 22, '', ''))
    
    def tearDown(self):
        stopMaster()

# base main routine            #################################

def usage(progname):
    print USAGE%vars()

def rostestMain(argv, stdout, env, testClasses, singleTest=None):
    """
    @param testClasses: list of TestCase classes to test
    @type testClasses: [Class unittest.TestCase]
    @param singleTest: if specified, only run this test case 
    """
    if singleTest:
        print "SINGLE TEST MODE: ",singleTest
    assert testClasses, "No TestCase classes specified"
    suite = unittest.TestSuite()
    loader = unittest.TestLoader()
    for test in testClasses:
        if singleTest:
            print loader.getTestCaseNames(test)
            if singleTest in loader.getTestCaseNames(test):
                testName = "%s.%s"%(test.__name__, singleTest)
                print "TEST NAME", testName
                suite.addTest(loader.loadTestsFromName(testName, __import__(test.__module__)))
            else:
                print "IGNORING", test
        else:
            suite.addTests(loader.loadTestsFromTestCase(test))
        
    unittest.TextTestRunner(verbosity=2).run(suite)            


# ROS api unit test assertions #################################

lastMsg = None
lastCode = None
lastVal = None
def getLastMsg():
    """stores the last message seen by apiSuccess/Fail/Error"""
    return lastMsg
def getLastCode():
    """stores the last status code seen by apiSuccess/Fail/Error"""
    return lastCode
def getLastVal():
    """stores the last return value seen by apiSuccess/Fail/Error"""
    return lastVal

def apiSuccess(args):
    """unit test assertion that fails if status code is not 1 and
    otherwise returns the value parameter
    @return val: value parameter from args (arg[2] for master/slave API)"""
    global lastCode, lastMsg, lastVal
    lastCode, lastMsg, lastVal = args
    assert lastCode == 1, "status code is not 1: %s"%lastMsg
    return lastVal

apiSucceed = apiSuccess #alias as I keep making this typo
    
def apiFail(args):
    """unit test assertions that fails if status code is not 0 and
    otherwise returns true"""
    global lastCode, lastMsg, lastVal
    lastCode, lastMsg, lastVal = args
    assert lastCode == 0, "Call should have failed with status code 0: %s"%lastMsg

def apiError(args):
    """unit test assertion that fails if status code is not -1 and
    otherwise returns true"""
    global lastCode, lastMsg, lastVal
    lastCode, lastMsg, lastVal = args
    assert lastCode == -1, "Call should have returned error -1 code: %s"%lastMsg

# utilities ####################################################

def getMasterUri():
    #TODO: #71 can't have hardcoded node port if we allow concurrent tests    
    return "http://%s:%s/"%masterAddr
def getMasterUriAlt():
    "@return: return master uri with localhost references resolved to an address"
    if masterAddr[0] == 'localhost':
        return "http://%s:%s/"%(socket.gethostbyname(socket.gethostname()), masterAddr[1])
    return "http://%s:%s/"%masterAddr

def getRosRoot():
    return os.environ[rospy.ROS_ROOT]


# test node ####################################################

nproxy = None
def getTestNode():
    """get an xmlrpc handle to test node. NOTE: unlike getMaster(), a test
    node is not automatically started during L{ROSGraphTestCase}s"""
    global nproxy
    if nproxy is None:
        uri = "http://%s:%s/"%(testNodeAddr)
        nproxy = xmlrpclib.ServerProxy(uri)
    return nproxy

#TODO: start on other machines
testNodePid = None
def startTestNode(fullName):
    """Starts a test node with the specified name.

    IMPORTANT: there is only one 'testnode' running at a time. This is
    simply a convenience routine for starting unit tests against a
    single node and isn't useful for multinode tests."""
    global testNodePid
    if testNodePid:
        stopTestNode()
    fullEnv = os.environ.copy()
    #TODO: #71 can't have hardcoded node port if we allow concurrent tests
    fullEnv[rospy.ROS_PORT] = str(testNodeAddr[1])
    fullEnv[rospy.ROS_MASTER_URI] = getMasterUri()
    fullEnv[rospy.ROS_NODE] = str(fullName)
    
    rosRoot = getRosRoot()
    pkg, node = testNode
    command = os.path.join(rosRoot, 'pkg', pkg, 'nodes', node)
    testNodePid = os.spawnve(os.P_NOWAIT, command, [command], fullEnv)

def stopTestNode():
    global testNodePid
    print "stopTestNode"
    if not testNodePid:
        return
    try:
        getTestNode().shutdown('rostest')
    except:
        pass
    #warning: slaves do not have SIGTERM handlers like masters as slaves are
    #meant to be embedded in another application
    import time
    time.sleep(0.1)
    os.kill(testNodePid, signal.SIGTERM)
    print "Waiting for",  testNodePid, "to die"
    os.waitpid(testNodePid, 0)
    testNodePid = None

# master    ####################################################

mproxy = None
def getMaster():
    global mproxy
    if mproxy is None:
        uri = "http://%s:%s/"%masterAddr
        mproxy = xmlrpclib.ServerProxy(uri)
    return mproxy

masterPid = None
def startMaster(id='master'):
    """spawn a ros master"""
    #TODO: parameterize so that other masters can be instantiated
    global masterPid
    if masterPid:
        stopMaster()
    fullEnv = os.environ.copy()
    #TODO: #71 can't have hardcoded node port if we allow concurrent tests    
    fullEnv[rospy.ROS_PORT] = str(masterAddr[1])
    #TODO: #74 use id to change the master log file name
    #fullEnv[rospy.ROS_LOG] = rostest.id.master.log
    rosRoot = getRosRoot()
    command = os.path.join(rosRoot, 'pkg', masterPkg, 'nodes', 'master')
    masterPid = os.spawnve(os.P_NOWAIT, command, [command], fullEnv)

def stopMaster():
    global masterPid
    print "stopMaster"
    if not masterPid:
        return
    try:
        getMaster().shutdown('rostest')
    except:
        pass
    #warning: an os.kill leaves stray processes as python atexit does not invoke on signals
    #we don't override signal handlers as this may interfere with higher level libs
    import time
    time.sleep(0.1)
    os.kill(masterPid, signal.SIGTERM)
    print "Waiting for",  masterPid, "to die"
    os.waitpid(masterPid, 0)
    print masterPid, "has died"
    masterPid = None

def waitForNewNode(name, node, timeout):
    timeoutT = time.time() + timeout
    val = None
    while time.time() < timeoutT and not val:
        try:
            code,status,val = node.getPid('')
        except socket.error, e:
            pass
        time.sleep(0.1)
    assert val, "Could not start new %s instance (timeout)"%name
    return val

def waitForNewTestNode(name, timeout):
    val = waitForNewNode('test node', getTestNode(), timeout)
    
def waitForNewMaster(timeout):
    master = getMaster()
    val = waitForNewNode('master', master, timeout)
    code, status, val = master.getGraph('') #[nodes, flows]
    assert len(val[0]) == 1, "Master graph is not empty: non-master nodes are present"
    assert not val[1], "Master graph is not empty: flows are present"

# process management   ##########################################
TEST_MACHINE = 'test-machine'

def rostestSigterm(*args):
    "try to prevent rogue python processes"
    print "rostest: handling SIGTERM"
    try:
        stopTestNode()
        stopMaster()
    finally:
        #can't use sys.exit because unittest traps the SystemExit exception
        os._exit(0)

#signal.signal(signal.SIGINT, rostestSigterm)
signal.signal(signal.SIGTERM, rostestSigterm)

import atexit
atexit.register(stopMaster)
atexit.register(stopTestNode)
    
