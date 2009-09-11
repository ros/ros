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
# Revision $Id: testSlave.py 1100 2008-05-29 20:23:54Z sfkwc $

import os
import sys
import string
import time
import xmlrpclib

import rospy
import roslib.rosenv 
import roslib.names 
from test_ros.rosclient import *

NODE_INTEGRATION_NAME = "node_integration_test"


_required_subscriptions = 'test_string_in', 'test_primitives_in', 'test_arrays_in', 'test_header_in', 'probe_topic'

# only publishers determine topic type, so we test against their declared spec
_required_publications_map = {
    'test_string_out': 'test_ros/TestString',
    'test_primitives_out': 'test_ros/TestPrimitives',
    'test_arrays_out': 'test_ros/TestArrays',
    'test_header_out': 'test_ros/TestHeader',
    }
_required_publications  = _required_publications_map.keys()

_TCPROS = 'TCPROS'

# NOTE: probe_topic is a unpublished topic that merely exists to test
# APIs that talk about subscriptions (e.g. publisherUpdate)

_name = None
## set_node_name() must be called prior to the unit test so that the test harness knows its
## ROS name.
def set_node_name(name):
    global _name
    _name = name

# Have to try as hard as possible to not use rospy code in testing rospy code, so this is
# a reimplementation of the caller ID spec so that NodeApiTestCase knows its name
## reimplementation of caller ID spec separately from rospy
def get_caller_id():
    if _name is None:
        raise Exception("set_node_name has not been called yet")
    ros_ns = os.environ.get(roslib.rosenv.ROS_NAMESPACE, roslib.names.GLOBALNS)
    return roslib.names.ns_join(ros_ns, _name)    
    
## Parent of node API and integration test cases. Performs common state setup
class _NodeTestCase(TestRosClient):

    def __init__(self, *args):
        super(_NodeTestCase, self).__init__(*args)
        
        self.ns = os.environ.get(roslib.rosenv.ROS_NAMESPACE, roslib.names.GLOBALNS)
        self.caller_id = get_caller_id()

        # load in name of test node
        self.test_node = 'test_node' #default
        for arg in sys.argv:
            if arg.startswith("--node="):
                self.test_node = arg[len("--node="):]
        # resolve
        self.test_node = roslib.names.ns_join(self.ns, self.test_node)
                
        
    def setUp(self):
        super(_NodeTestCase, self).setUp()
        # retrieve handle on node
        # give ourselves five seconds for node to appear
        import time
        timeout_t = 5.0 + time.time()
        self.node_api = None
        while time.time() < timeout_t and not self.node_api:
            code, msg, node_api = self.master.lookupNode(self.caller_id, self.test_node)
            if code == 1:
                self.node_api = node_api
        if not self.node_api:
            self.fail("master did not return XML-RPC API for [%s, %s]"%(self.caller_id, self.test_node))
        print "[%s] API  = %s"%(self.test_node, self.node_api)
        self.assert_(self.node_api.startswith('http'))
        self.node = xmlrpclib.ServerProxy(self.node_api)

    ## validates a URI as being http(s)
    def _checkUri(self, uri):
        import urlparse
        parsed = urlparse.urlparse(uri)
        self.assert_(parsed[0] in ['http', 'https'], 'protocol [%s] in [%s] invalid'%(parsed[0], uri))
        self.assert_(parsed[1], 'host missing [%s]'%uri)
        if not sys.version.startswith('2.4'): #check not available on py24
            self.assert_(parsed.port, 'port missing/invalid [%s]'%uri)        

    ## dynamically create the expected topic->type map based on the current name resolution context
    def _createTopicTypeMap(self):
        new_map = {}
        for t in _required_publications_map.iterkeys():
            new_map[rospy.resolve_name(t)] = _required_publications_map[t]
        return new_map
    
## Expects a single test node to be running with name 'test_node' and subscribed to 'test_string'
class NodeApiTestCase(_NodeTestCase):

    ## validate node.getPid(caller_id)        
    def testGetPid(self):
        # test with bad arity
        self.apiError(self.node.getPid())
        # test success        
        pid = self.apiSuccess(self.node.getPid(self.caller_id))
        self.assert_(pid > 0)

    ## subroutine for testGetSubscriptions/testGetPublications
    def _checkTopics(self, required, actual):
        actual = [t for t, _ in actual]
        missing = set(required) - set(actual)
        self.failIf(len(missing), 'missing required topics: %s'%(','.join(missing)))

    ## validate node.getPublications(caller_id)
    def testGetPublications(self):
        # test with bad arity
        self.apiError(self.node.getPublications())
        self.apiError(self.node.getPublications(self.caller_id, 'something extra'))
        
        # test success
        self._checkTopics([rospy.resolve_name(t) for t in _required_publications],
                          self.apiSuccess(self.node.getPublications(self.caller_id)))
    ## validate node.getSubscriptions(caller_id)
    def testGetSubscriptions(self):
        # test with bad arity
        self.apiError(self.node.getSubscriptions())
        self.apiError(self.node.getSubscriptions(self.caller_id, 'something extra'))        
        # test success
        self._checkTopics([rospy.resolve_name(t) for t in _required_subscriptions],
                          self.apiSuccess(self.node.getSubscriptions(self.caller_id)))

    ## validate node.paramUpdate(caller_id, key, value)
    def testParamUpdate(self):
        node = self.node
        good_key = roslib.names.ns_join(self.ns, 'good_key')
        bad_key = roslib.names.ns_join(self.ns, 'bad_key')
        
        # test bad key
        self.apiError(node.paramUpdate(self.caller_id, '', 'bad'))
        self.apiError(node.paramUpdate(self.caller_id, 'no_namespace', 'bad'))
        # test with bad arity
        self.apiError(node.paramUpdate(self.caller_id, bad_key))     
        self.apiError(node.paramUpdate(self.caller_id)) 

        # node is not subscribed to good_key (yet)
        self.apiError(node.paramUpdate(self.caller_id, good_key, 'good_value'))

        # we can't actually test success cases without forcing node to subscribe
        #self.apiSuccess(node.paramUpdate(self.caller_id, good_key, 1))
        #self.apiSuccess(node.paramUpdate(self.caller_id, good_key, True))
        #self.apiSuccess(node.paramUpdate(self.caller_id, good_key, 10.0))

    ## validate node.getUri(caller_id)        
    def testGetUri(self):
        # test bad arity
        self.apiError(self.node.getUri(self.caller_id, 'bad'))
        self.apiError(self.node.getUri())
        # test success
        self._checkUri(self.apiSuccess(self.node.getUri(self.caller_id)))

    ## validate node.getName(caller_id)        
    def testGetName(self):
        # test bad arity
        self.apiError(self.node.getName(self.caller_id, 'bad'))
        self.apiError(self.node.getName())
        # test success
        val = self.apiSuccess(self.node.getName(self.caller_id))
        self.assert_(len(val), "empty name")

    ## validate node.getMasterUri(caller_id)                
    def testGetMasterUri(self):
        # test bad arity
        self.apiError(self.node.getMasterUri(self.caller_id, 'bad'))
        self.apiError(self.node.getMasterUri())
        # test success
        uri = self.apiSuccess(self.node.getMasterUri(self.caller_id))
        self._checkUri(uri)
        self.assertEquals(roslib.rosenv.get_master_uri(), uri)

    ## validate node.publisherUpdate(caller_id, topic, uris) 
    def testPublisherUpdate(self):
        node = self.node
        probe_topic = roslib.names.ns_join(self.ns, 'probe_topic')
        fake_topic = roslib.names.ns_join(self.ns, 'fake_topic')        
        # test bad arity
        self.apiError(node.getBusStats(self.caller_id, 'bad'))
        self.apiError(node.getBusStats())
        # test bad args
        self.apiError(node.publisherUpdate(self.caller_id, '/bad_topic', 'bad'))
        self.apiError(node.publisherUpdate(self.caller_id, '/bad_topic', 2))
        self.apiError(node.publisherUpdate(self.caller_id, '/bad_topic', False))                
        self.apiError(node.publisherUpdate(self.caller_id, '/bad_topic', ['bad']))
        self.apiError(node.publisherUpdate())
        
        # test success
        # still success even if not actually interested in topic
        self.apiSuccess(node.publisherUpdate(self.caller_id, fake_topic,
                                             ['http://localhost:1234', 'http://localhost:5678']))
        self.apiSuccess(node.publisherUpdate(self.caller_id, fake_topic,
                                             []))
        # try it with it the /probe_topic, which will exercise some error branches in the client
        self.apiSuccess(node.publisherUpdate(self.caller_id, probe_topic,
                                             ['http://unroutablefakeservice:1234']))
        # give it some time to make sure it's attempted contact
        time.sleep(1.0)
        # check that it's still there
        self.apiSuccess(node.publisherUpdate(self.caller_id, probe_topic,
                                                  []))

    def _checkTCPROS(self, protocol_params):
        self.assert_(protocol_params, "no protocol params returned")
        self.assert_(type(protocol_params) == list, "protocol params must be a list: %s"%protocol_params)
        self.assertEquals(3, len(protocol_params), "TCPROS params should have length 3: %s"%protocol_params)
        self.assertEquals(protocol_params[0], _TCPROS)
        # expect ['TCPROS', 1.2.3.4, 1234]
        self.assertEquals(protocol_params[0], _TCPROS)            
        
    def testRequestTopic(self):
        node = self.node
        protocols = [[_TCPROS]]
        probe_topic = roslib.names.ns_join(self.ns, 'probe_topic')
        fake_topic = roslib.names.ns_join(self.ns, 'fake_topic')
        
        # test bad arity
        self.apiError(node.requestTopic(self.caller_id, probe_topic, protocols, 'extra stuff'))
        self.apiError(node.requestTopic(self.caller_id, probe_topic))
        self.apiError(node.requestTopic(self.caller_id))
        self.apiError(node.requestTopic())
        # test bad args
        self.apiError(node.requestTopic(self.caller_id, 1, protocols))
        self.apiError(node.requestTopic(self.caller_id, '', protocols))
        self.apiError(node.requestTopic(self.caller_id, fake_topic, protocols))
        self.apiError(node.requestTopic(self.caller_id, probe_topic, 'fake-protocols')) 
        
        topics = [roslib.names.ns_join(self.ns, t) for t in _required_publications]
        # currently only support TCPROS as we require all clients to support this
        protocols = [[_TCPROS]]
        for topic in topics:
            self._checkTCPROS(self.apiSuccess(node.requestTopic(self.caller_id, topic, protocols)))
        protocols = [['FakeTransport', 1234, 5678], [_TCPROS], ['AnotherFakeTransport']]
        # try each one more time, this time with more protocol choices
        for topic in topics:
            self._checkTCPROS(self.apiSuccess(node.requestTopic(self.caller_id, topic, protocols)))
            
    def testGetBusInfo(self):
        # test bad arity
        self.apiError(self.node.getBusInfo(self.caller_id, 'bad'))
        self.apiError(self.node.getBusInfo())
        #TODO: finish
        
    def testGetBusStats(self):
        # test bad arity
        self.apiError(self.node.getBusStats(self.caller_id, 'bad'))
        self.apiError(self.node.getBusStats())
        #TODO: finish

    ## test the state of the master based on expected node registration
    def testRegistrations(self):
        # setUp() ensures the node has registered with the master
        topics = self.apiSuccess(self.master.getPublishedTopics(self.caller_id, ''))
        topic_names = [t for t, type in topics]
        required_topic_pubs = [rospy.resolve_name(t) for t in _required_publications]
        required_topic_subs = [rospy.resolve_name(t) for t in _required_subscriptions]        
        self._checkTopics(required_topic_pubs, topics)
        
        # now check types
        topicTypeMap = self._createTopicTypeMap()
        for topic, type in topics:
            if topic in topicTypeMap:
                self.assertEquals(type, topicTypeMap[topic], "topic [%s]: type [%s] does not match expected [%s]"%(type, topic, topicTypeMap[topic]))

        # now check actual URIs
        node_name = self.test_node
        systemState = self.apiSuccess(self.master.getSystemState(self.caller_id))
        pubs, subs, srvs = systemState
        for topic, list in pubs:
            if topic in required_topic_pubs:
                self.assert_(node_name in list, "%s not in %s"%(self.node_api, list))
        for topic, list in subs:
            if topic in required_topic_subs:
                self.assert_(node_name in list, "%s not in %s"%(self.node_api, list))
        for service, list in srvs:
            #TODO: no service tests yet
            pass


## Performs end-to-end integration tests of a test_node. NodeIntegrationTestCase
## itself is a rospy node and thus implicitly depends on rospy functionality.        
class NodeIntegrationTestCase(_NodeTestCase):

    def __init__(self, *args):
        super(NodeIntegrationTestCase, self).__init__(*args)
        rospy.init_node(NODE_INTEGRATION_NAME)
        
    def testString(self):
        pub = rospy.Publisher('test_string_in', std_msgs.msg.String)
        sub = rospy.Subscriber('test_string_in', std_msgs.msg.String)
        #TODO: publish a bunch and check sequencing + caller_id
        pub.unregister()
        sub.unregister()

    def testPrimitives(self):
        pub = rospy.Publisher('test_primitives_in', std_msgs.msg.String)
        sub = rospy.Subscriber('test_primitives_out', std_msgs.msg.String) 
        #TODO: publish a bunch and check sequencing + caller_id
        pub.unregister()
        sub.unregister()

    def testArrays(self):
        pub = rospy.Publisher('test_header_in', std_msgs.msg.String)
        sub = rospy.Subscriber('test_header_out', std_msgs.msg.String)
        #TODO: publish a bunch and check sequencing + caller_id        
        pub.unregister()
        sub.unregister()

    def testHeader(self):
        #msg.auto_header = True
        pub = rospy.Publisher('test_header_in', std_msgs.msg.String)
        sub = rospy.Subscriber('test_header_out', std_msgs.msg.String)
        #TODO: publish a bunch and check sequencing + caller_id        
        pub.unregister()
        sub.unregister()


