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
    
class _MasterTestCase(TestRosClient):

    def __init__(self, *args):
        super(_MasterTestCase, self).__init__(*args)
        self.ns = os.environ.get(roslib.rosenv.ROS_NAMESPACE, roslib.names.GLOBALNS)
        self.caller_id = get_caller_id()
        
    def setUp(self):
        super(_MasterTestCase, self).setUp()
        self.master_uri = os.environ.get(roslib.rosenv.ROS_MASTER_URI, None)
        self._checkUri(self.master_uri)
        self.master = xmlrpclib.ServerProxy(self.master_uri)

    ## validates a URI as being http(s)
    def _checkUri(self, uri):
        import urlparse
        parsed = urlparse.urlparse(uri)
        self.assert_(parsed[0] in ['http', 'https'], 'protocol [%s] in [%s] invalid'%(parsed[0], uri))
        self.assert_(parsed[1], 'host missing [%s]'%uri)
        if not sys.version.startswith('2.4'): #check not available on py24
            self.assert_(parsed.port, 'port missing/invalid [%s]'%uri)        
    
## Expects a single test node to be running with name 'test_node' and subscribed to 'test_string'
class MasterApiTestCase(_MasterTestCase):

    ## validate master.getMasterUri(caller_id)
    def _testGetMasterUri(self):
        # test with bad arity
        self.apiError(self.master.getMasterUri())
        # test success        
        uri = self.apiSuccess(self.master.getMasterUri(self.caller_id))
        self._checkUri(uri)

        # make sure we agree on ports
        import urlparse
        parsed = urlparse.urlparse(uri)
        parsed2 = urlparse.urlparse(self.master_uri)        
        
        self.assertEquals(parsed.port, parsed2.port, "expected ports do not match")

    ## validate master.getPid(caller_id)        
    def _testGetPid(self):
        # test with bad arity
        self.apiError(self.master.getPid())
        # test success        
        pid = self.apiSuccess(self.master.getPid(self.caller_id))
        self.assert_(pid > 0)

    ## validate master.getUri(caller_id)        
    def _testGetUri(self):
        # test with bad arity
        self.apiError(self.master.getUri())
        # test success        
        uri = self.apiSuccess(self.master.getUri(self.caller_id))
        self.assert_(type(uri) == str)
        
    ## common test subroutine of both register and unregister tests. registers the common test cases
    def _subTestRegisterServiceSuccess(self):
        master = self.master
        
        caller_id = '/service_node'
        caller_api = 'http://localhost:4567/'                
        service_base = '/service'
        
        # test success        
        for i in xrange(0, 10):
            service_name = "%s-%s"%(service_base, i)
            service_api = 'rosrpc://localhost:123%s/'%i
            # register the service
            self.apiSuccess(master.registerService(caller_id, service_name, service_api, caller_api))
            # test master state
            val = self.apiSuccess(master.lookupService(caller_id, service_name))
            self.assertEquals(service_api, val)
            val = self.apiSuccess(master.lookupNode(self.caller_id, caller_id)) 
            self.assertEquals(caller_api, val)

            _, _, srvs = self.apiSuccess(master.getSystemState(self.caller_id))
            for j in xrange(0, i+1):
                jservice_name = "%s-%s"%(service_base, j)
                jentry = [jservice_name, [caller_id]]
                self.assert_(jentry in srvs, "master service list %s is missing %s"%(srvs, jentry))

        # TODO: have to test subscriber callback
        # TODO: validate with getSystemState()

    ## validate master.registerService(caller_id, service, service_api, caller_api) 
    def _testRegisterServiceSuccess(self):
        self._subTestRegisterServiceSuccess()

    def _testUnregisterServiceSuccess(self):
        self._subTestRegisterServiceSuccess()
        master = self.master
        caller_id = '/service_node'
        caller_api = 'http://localhost:4567/'                
        service_base = '/service'

        for i in xrange(0, 10):
            service_name = "%s-%s"%(service_base, i)
            service_api = 'rosrpc://localhost:123%s/'%i

            # unregister the service
            code, msg, val = master.unregisterService(caller_id, service_name, service_api)
            self.assertEquals(code, 1, "code != 1, return message was [%s]"%msg)

            # test the master state
            self.apiError(master.lookupService(self.caller_id, service_name), "master has a reference to unregistered service. message from master for unregister was [%s]"%msg)

            if i < 9:
                val = self.apiSuccess(master.lookupNode(self.caller_id, caller_id))
                self.assertEquals(caller_api, val, "master prematurely invalidated node entry for [%s] (lookupNode)"%caller_id)
            
            _, _, srvs = self.apiSuccess(master.getSystemState(self.caller_id))
            for j in xrange(0, i+1):
                jservice_name = "%s-%s"%(service_base, j)
                jentry = [jservice_name, [caller_id]]
                self.assert_(jentry not in srvs, "master service list %s should not have %s"%(srvs, jentry))
            for j in xrange(i+1, 10):
                jservice_name = "%s-%s"%(service_base, j)
                jentry = [jservice_name, [caller_id]]
                self.assert_(jentry in srvs, "master service list %s is missing %s"%(srvs, jentry))

            # TODO: have to test subscriber callback
            
        # Master's state should be zero'd out now
        
        #  - #457 make sure that lookupNode isn't returning stale info
        self.apiError(master.lookupNode(self.caller_id, caller_id), "master has a stale reference to unregistered service node API")
        _, _, srvs = self.apiSuccess(master.getSystemState(self.caller_id))
        srvs = [s for s in srvs if not s[0].startswith('/rosout/')]
        self.assertEquals(0, len(srvs), "all services should have been unregistered: %s"%srvs)

    def _testRegisterServiceInvalid(self):
        master = self.master
        
        service = '/service'
        service_api = 'rosrpc://localhost:1234/'
        caller_api = 'http://localhost:4567/'                

        # test with bad arity
        self.apiError(master.registerService())
        self.apiError(master.registerService(self.caller_id, service))
        self.apiError(master.registerService(self.caller_id, service, service_api))

        # test with bad args
        self.apiError(master.registerService(self.caller_id, '', service_api, caller_api))
        self.apiError(master.registerService(self.caller_id, service, '', caller_api))        
        self.apiError(master.registerService(self.caller_id, service, service_api, ''))                
        
    def _testUnregisterServiceInvalid(self):
        master = self.master
        
        service = '/service'
        service_api = 'rosrpc://localhost:1234/'

        # test with bad arity
        self.apiError(master.unregisterService())
        self.apiError(master.unregisterService(self.caller_id, service))

        # test with bad args
        self.apiError(master.unregisterService(self.caller_id, '', service_api))
        self.apiError(master.unregisterService(self.caller_id, service, ''))        

    def _testRegisterPublisherInvalid(self):
        master = self.master
        
        topic = '/pub_topic'
        topic_type = 'std_msgs/String'        
        caller_api = 'http://localhost:4567/'                

        # test with bad arity
        self.apiError(master.registerPublisher())
        self.apiError(master.registerPublisher(self.caller_id, topic))
        self.apiError(master.registerPublisher(self.caller_id, topic, topic_type))

        # test with bad args
        self.apiError(master.registerPublisher(self.caller_id, '',    topic_type, caller_api))
        self.apiError(master.registerPublisher(self.caller_id, topic, '',         caller_api))        
        self.apiError(master.registerPublisher(self.caller_id, topic, topic_type, ''))

    def _testUnregisterPublisherInvalid(self):
        master = self.master
        
        topic = '/pub_topic'
        caller_api = 'http://localhost:4567/'                

        # test with bad arity
        self.apiError(master.unregisterPublisher())
        self.apiError(master.unregisterPublisher(self.caller_id, topic))

        # test with bad args
        self.apiError(master.unregisterPublisher(self.caller_id, '',    caller_api))
        self.apiError(master.unregisterPublisher(self.caller_id, topic, ''))

    def _testRegisterSubscriberInvalid(self):
        master = self.master
        
        topic = '/sub_topic'
        topic_type = 'std_msgs/String'        
        caller_api = 'http://localhost:4567/'                

        # test with bad arity
        self.apiError(master.registerSubscriber())
        self.apiError(master.registerSubscriber(self.caller_id, topic))
        self.apiError(master.registerSubscriber(self.caller_id, topic, topic_type))

        # test with bad args
        self.apiError(master.registerSubscriber(self.caller_id, '',    topic_type, caller_api))
        self.apiError(master.registerSubscriber(self.caller_id, topic, '',         caller_api))        
        self.apiError(master.registerSubscriber(self.caller_id, topic, topic_type, ''))

    def _testUnregisterSubscriberInvalid(self):
        master = self.master
        
        topic = '/sub_topic'
        caller_api = 'http://localhost:4567/'                

        # test with bad arity
        self.apiError(master.registerSubscriber())
        self.apiError(master.registerSubscriber(self.caller_id, topic))

        # test with bad args
        self.apiError(master.unregisterSubscriber(self.caller_id, '',    caller_api))
        self.apiError(master.unregisterSubscriber(self.caller_id, topic, ''))

    ## common test subroutine of both register and unregister tests. registers the common test cases
    def _subTestRegisterPublisherSuccess(self):
        master = self.master
        
        caller_id = '/pub_node'
        caller_api = 'http://localhost:4567/'                
        topic_base = '/pub_topic'
        topic_type = 'std_msgs/String'  
        
        # test success        
        for i in xrange(0, 10):
            topic_name = "%s-%s"%(topic_base, i)
            # register the topic
            self.apiSuccess(master.registerPublisher(caller_id, topic_name, topic_type, caller_api))
            # test master state
            # - master knows caller_id
            val = self.apiSuccess(master.lookupNode(self.caller_id, caller_id))
            self.assertEquals(caller_api, val)
            # - master knows topic type
            val = self.apiSuccess(master.getPublishedTopics(self.caller_id, '/'))
            self.assert_([topic_name, topic_type] in val, "master does not know topic type: %s"%val)

            pubs, _, _ = self.apiSuccess(master.getSystemState(self.caller_id))
            for j in xrange(0, i+1):
                jtopic_name = "%s-%s"%(topic_base, j)
                jentry = [jtopic_name, [caller_id]]
                self.assert_(jentry in pubs, "master pub/sub list %s is missing %s"%(pubs, jentry))
            
        # TODO: have to test subscriber callback

    ## #591: this test may change if we make registering '*' unsupported
    def _testRegisterPublisherTypes(self):
        master = self.master
        caller_id = '/pub_node'
        caller_api = 'http://localhost:4567/'                
        topic_name = '/type_test_pub_topic'

        # register anytype first
        val = self.apiSuccess(master.registerPublisher(caller_id, topic_name, '*', caller_api))
        self.assertEquals([], val) # should report no subscribers
        val = self.apiSuccess(master.getPublishedTopics(self.caller_id, '/'))
        self.assert_([topic_name, '*'] in val, "master is not reporting * as type: %s"%val)
        # register a grounded type and make sure that '*' can't overwrite it
        for t in ['std_msgs/String', '*']:
            val = self.apiSuccess(master.registerPublisher(caller_id, topic_name, t, caller_api))   
            self.assertEquals([], val) # should report no subscribers
            val = self.apiSuccess(master.getPublishedTopics(self.caller_id, '/'))
            self.assert_([topic_name, 'std_msgs/String'] in val, "master is not reporting * as type: %s"%val)
        
    ## validate master.registerPublisher(caller_id, topic, topic_api, caller_api) 
    def _testRegisterPublisherSuccess(self):
        self._subTestRegisterPublisherSuccess()

        # a couple more test cases to verify that registerPublisher's return value is correct
        master = self.master
        topic = '/pub_topic-0'
        type = 'std_msgs/String'
        pub_caller_api = 'http://localhost:4567/'
        
        subs = []
        for i in xrange(5678, 5685):
            api = 'http://localhost:%s'%i
            subs.append(api)
            self.apiSuccess(master.registerSubscriber('/sub_node-%i'%i, topic, type, api))
            val = self.apiSuccess(master.registerPublisher('/pub_node', topic, type, pub_caller_api))            
            self.assertEquals(subs, val)
        
    def _testUnregisterPublisherSuccess(self):
        self._subTestRegisterPublisherSuccess()
        master = self.master
        caller_id = '/pub_node'
        caller_api = 'http://localhost:4567/' 
        topic_base = '/pub_topic'

        for i in xrange(0, 10):
            topic_name = "%s-%s"%(topic_base, i)

            # unregister the topic
            code, msg, val = master.unregisterPublisher(caller_id, topic_name, caller_api)
            self.assertEquals(code, 1, "code != 1, return message was [%s]"%msg)

            # test the master state
            if i < 9:
                val = self.apiSuccess(master.lookupNode(self.caller_id, caller_id))
                self.assertEquals(caller_api, val, "master prematurely invalidated node entry for [%s] (lookupNode)"%caller_id)

            pubs, _, _ = self.apiSuccess(master.getSystemState(self.caller_id))
            for j in xrange(0, i+1):
                jtopic_name = "%s-%s"%(topic_base, j)
                jentry = [jtopic_name, [caller_id]]
                self.assert_(jentry not in pubs, "master pub/sub list %s should not have %s"%(pubs, jentry))
            for j in xrange(i+1, 10):
                jtopic_name = "%s-%s"%(topic_base, j)
                jentry = [jtopic_name, [caller_id]]
                self.assert_(jentry in pubs, "master pub/sub list %s is missing %s"%(pubs, jentry))

            # TODO: have to test subscriber callback
            
        #  - #457 make sure that lookupNode isn't returning stale info
        self.apiError(master.lookupNode(self.caller_id, caller_id), "master has a stale reference to unregistered topic node API. pubs are %s"%pubs)

    ## common test subroutine of both register and unregister tests. registers the common test cases
    ## 'simple' test cases do not setup any publisher state to validate against
    def _subTestRegisterSubscriberSimpleSuccess(self):
        master = self.master
        
        caller_id = '/sub_node'
        caller_api = 'http://localhost:4567/'                
        topic_base = '/sub_topic'
        topic_type = 'std_msgs/String'  
        
        # test success        
        for i in xrange(0, 10):
            topic_name = "%s-%s"%(topic_base, i)
            # register the topic
            self.apiSuccess(master.registerSubscriber(caller_id, topic_name, topic_type, caller_api))
            # test master state
            # - master knows caller_id
            val = self.apiSuccess(master.lookupNode(self.caller_id, caller_id))
            self.assertEquals(caller_api, val)

            _, subs, _ = self.apiSuccess(master.getSystemState(self.caller_id))
            for j in xrange(0, i+1):
                jtopic_name = "%s-%s"%(topic_base, j)
                jentry = [jtopic_name, [caller_id]]
                self.assert_(jentry in subs, "master pub/sub list %s is missing %s"%(subs, jentry))
            
    ## validate master.registerSubscriber(caller_id, topic, topic_api, caller_api) 
    def _testRegisterSubscriberSimpleSuccess(self):
        self._subTestRegisterSubscriberSimpleSuccess()

    def _testUnregisterSubscriberSuccess(self):
        self._subTestRegisterSubscriberSimpleSuccess()
        master = self.master
        caller_id = '/sub_node'
        caller_api = 'http://localhost:4567/' 
        topic_base = '/sub_topic'

        for i in xrange(0, 10):
            topic_name = "%s-%s"%(topic_base, i)

            # unregister the topic
            code, msg, val = master.unregisterSubscriber(caller_id, topic_name, caller_api)
            self.assertEquals(code, 1, "code != 1, return message was [%s]"%msg)

            # test the master state
            if i < 9:
                val = self.apiSuccess(master.lookupNode(self.caller_id, caller_id))
                self.assertEquals(caller_api, val, "master prematurely invalidated node entry for [%s] (lookupNode)"%caller_id)

            _, subs, _ = self.apiSuccess(master.getSystemState(self.caller_id))
            for j in xrange(0, i+1):
                jtopic_name = "%s-%s"%(topic_base, j)
                jentry = [jtopic_name, [caller_id]]
                self.assert_(jentry not in subs, "master pub/sub list %s should not have %s"%(subs, jentry))
            for j in xrange(i+1, 10):
                jtopic_name = "%s-%s"%(topic_base, j)
                jentry = [jtopic_name, [caller_id]]
                self.assert_(jentry in subs, "master pub/sub list %s is missing %s"%(subs, jentry))

        #  - #457 make sure that lookupNode isn't returning stale info
        self.apiError(master.lookupNode(self.caller_id, caller_id), "master has a stale reference to unregistered topic node API. subs are %s"%subs)


