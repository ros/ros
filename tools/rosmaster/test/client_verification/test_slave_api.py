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

# This is a rewrite of the old node API tests, which focus too much on
# a completed node API and don't facilitate easy bring up of a new
# client library.

import os
import sys
import string
import time
import unittest
import xmlrpclib

import rosunit
import rosgraph

TCPROS = 'TCPROS'

CALLER_ID = '/test_harness'
TEST_NODE_NAME = '/test_node' #default

class TopicDescription(object):
    def __init__(self, topic_name, topic_type):
        self.topic_name = topic_name
        self.topic_type = topic_type

        #validate topic
        if not rosgraph.names.is_legal_name(topic_name):
            raise ValueError('topic name: %s'%(topic_name))
        
        # validate type
        p, t = topic_type.split('/')

class TopicDescriptionList(object):
    
    def __init__(self, xmlrpcvalue):
        # [ [topic1, topicType1]...[topicN, topicTypeN]]]
        if not type(xmlrpcvalue) == list:
            raise ValueError("publications must be a list")
        self.topics = []
        for n, t in xmlrpcvalue:
            self.topics.append(TopicDescription(n, t))

    def as_dict(self):
        d = {}
        for t in self.topics:
            d[t.topic_name] = t.topic_type
        return d

class TestSlaveApi(unittest.TestCase):

    def __init__(self, *args, **kwds):
        super(TestSlaveApi, self).__init__(*args)
        
        self.ns = os.environ.get(rosgraph.ROS_NAMESPACE, rosgraph.names.GLOBALNS)

        # load in name of test node
        self.test_node = 'test_node' #default
        self.required_pubs = TopicDescriptionList([])
        self.required_subs = TopicDescriptionList([])

        for arg in sys.argv:
            if arg.startswith("--node="):
                self.test_node = arg[len("--node="):]
            if arg.startswith("--profile="):
                self.test_node_profile = arg[len("--profile="):]
                self.load_profile(self.test_node_profile)
                
        # resolve
        self.test_node = rosgraph.names.ns_join(self.ns, self.test_node)
                
    def load_profile(self, filename):
        import yaml
        with open(filename) as f:
            d = yaml.load(f)
        self.required_pubs = d.get('pubs', {})
        self.required_subs = d.get('subs', {})
        
    def setUp(self):
        self.caller_id = CALLER_ID
        # retrieve handle on node
        # give ourselves 10 seconds for node to appear
        timeout_t = 10.0 + time.time()
        self.node_api = None
        self.master = rosgraph.Master(self.caller_id)
        while time.time() < timeout_t and not self.node_api:
            try:
                self.node_api = self.master.lookupNode(self.test_node)
            except:
                time.sleep(0.1)
        if not self.node_api:
            self.fail("master did not return XML-RPC API for [%s, %s]"%(self.caller_id, self.test_node))
        print "[%s] API  = %s"%(self.test_node, self.node_api)
        self.assert_(self.node_api.startswith('http'))
        self.node = xmlrpclib.ServerProxy(self.node_api)

        # hack: sleep for a couple seconds just in case the node is
        # still registering with the master.
        time.sleep(2.)

    def apiSuccess(self, args):
        """
        unit test assertion that fails if status code is not 1 and otherwise returns the value parameter.
        @param args: returnv value from ROS API call
        @type  args: [int, str, val]
        @return: value parameter from args (arg[2] for master/slave API)
        """
        self.assert_(len(args) == 3, "invalid API return value triplet: %s"%str(args))
        self.last_code, self.last_msg, self.last_val = args
        assert self.last_code == 1, "status code is not 1: %s"%self.last_msg
        return self.last_val

    def apiFail(self, args):
        """
        unit test assertions that fails if status code is not 0 and otherwise returns true.
        @param args: returnv value from ROS API call
        @type  args: [int, str, val]
        @return: True if status code is 0
        """
        self.assert_(len(args) == 3, "invalid API return value triplet: %s"%str(args))
        self.last_code, self.last_msg, self.last_val = args
        assert self.last_code == 0, "Call should have failed with status code 0: %s"%self.last_msg

    def apiError(self, args, msg=None):
        """
        unit test assertion that fails if status code is not -1 and otherwise returns true.
        @param args: returnv value from ROS API call
        @type  args: [int, str, val]
        @return: True if status code is -1
        """
        self.assert_(len(args) == 3, "invalid API return value triplet: %s"%str(args))
        self.last_code, self.last_msg, self.last_val = args
        if msg:
            assert self.last_code == -1, "%s (return msg was %s)"%(msg, self.last_msg)
        else:
            assert self.last_code == -1, "Call should have returned error -1 code: %s"%self.last_msg

    def check_uri(self, uri):
        """
        validates a URI as being http(s)
        """
        import urlparse
        parsed = urlparse.urlparse(uri)
        self.assert_(parsed[0] in ['http', 'https'], 'protocol [%s] is [%s] invalid'%(parsed[0], uri))
        self.assert_(parsed[1], 'host missing [%s]'%uri)
        self.assert_(parsed.port, 'port missing/invalid [%s]'%uri)        

    def test_getPid(self):
        """
        validate node.getPid(caller_id)        
        """
        # test success        
        pid = self.apiSuccess(self.node.getPid(self.caller_id))
        self.assert_(pid > 0)

        # test with bad arity: accept error or fault
        try:
            self.apiError(self.node.getPid())
        except xmlrpclib.Fault:
            pass

    def test_rosout(self):
        """
        make sure rosout is in publication and connection list
        """
        val = self.apiSuccess(self.node.getPublications(self.caller_id))
        pubs_d = TopicDescriptionList(val).as_dict()
        self.assertTrue('/rosout' in pubs_d, "node is not publishing to rosout")
        self.assertEquals('rosgraph_msgs/Log', pubs_d['/rosout'], "/rosout is not correct type")
        
    def test_simtime(self):
        """
        test that node obeys simtime (/Clock) contract

        http://www.ros.org/wiki/Clock
        """
        try:
            use_sim_time = self.master.getParam('/use_sim_time')
        except:
            use_sim_time = False
            
        val = self.apiSuccess(self.node.getSubscriptions(self.caller_id))
        subs_d = TopicDescriptionList(val).as_dict()
        if use_sim_time:
            self.assertTrue('/clock' in subs_d, "node is not subscribing to clock")
            self.assertEquals('rosgraph_msgs/Clock', subs_d['/clock'], "/clock is not correct type")
        else:
            self.assertFalse('/clock' in subs_d, "node is subscribed to /clock even though /use_sim_time is false")

    def test_getPublications(self):
        """
        validate node.getPublications(caller_id)
        """
        # test success
        pubs_value = self.apiSuccess(self.node.getPublications(self.caller_id))
        pubs = TopicDescriptionList(pubs_value)

        pubs_dict = pubs.as_dict()
        # this is separately tested by test_rosout
        if '/rosout' in pubs_dict:
            del pubs_dict['/rosout']
        self.assertEquals(self.required_pubs, pubs_dict)
        
        # test with bad arity: accept error or fault
        try:
            self.apiError(self.node.getPublications())
        except xmlrpclib.Fault:
            pass
        try:
            self.apiError(self.node.getPublications(self.caller_id, 'something extra'))
        except xmlrpclib.Fault:
            pass
        
    def test_getSubscriptions(self):
        """
        validate node.getSubscriptions(caller_id)
        """
        
        # test success
        value = self.apiSuccess(self.node.getSubscriptions(self.caller_id))
        subs = TopicDescriptionList(value)

        subs_dict = subs.as_dict()
        self.assertEquals(self.required_subs, subs_dict)

        # test with bad arity: accept error or fault
        try:
            self.apiError(self.node.getSubscriptions())
        except xmlrpclib.Fault:
            pass
        try:
            self.apiError(self.node.getSubscriptions(self.caller_id, 'something extra'))        
        except xmlrpclib.Fault:
            pass

    ## validate node.paramUpdate(caller_id, key, value)
    def test_paramUpdate(self):
        node = self.node
        good_key = rosgraph.names.ns_join(self.ns, 'good_key')
        bad_key = rosgraph.names.ns_join(self.ns, 'bad_key')
        
        # node is not subscribed to good_key (yet)
        self.apiError(node.paramUpdate(self.caller_id, good_key, 'good_value'))

        # test bad key
        self.apiError(node.paramUpdate(self.caller_id, '', 'bad'))
        self.apiError(node.paramUpdate(self.caller_id, 'no_namespace', 'bad'))

        # test with bad arity: accept error or fault
        try:
            self.apiError(node.paramUpdate(self.caller_id, bad_key))     
        except xmlrpclib.Fault:
            pass

        try:
            self.apiError(node.paramUpdate(self.caller_id)) 
        except xmlrpclib.Fault:
            pass

        # we can't actually test success cases without forcing node to subscribe
        #self.apiSuccess(node.paramUpdate(self.caller_id, good_key, 1))
        #self.apiSuccess(node.paramUpdate(self.caller_id, good_key, True))
        #self.apiSuccess(node.paramUpdate(self.caller_id, good_key, 10.0))

    def xtest_getUri(self):
        """
        Future: validate node.getUri(caller_id).  It would be nice to
        make this official API as it provides some debugging info.
        """
        # test success
        self.check_uri(self.apiSuccess(self.node.getUri(self.caller_id)))
        
        # test bad arity
        try:
            self.apiError(self.node.getUri(self.caller_id, 'bad'))
        except xmlrpclib.Fault:
            pass
        try:
            self.apiError(self.node.getUri())
        except xmlrpclib.Fault:
            pass
            
    def test_getMasterUri(self):
        """
        validate node.getMasterUri(caller_id)                
        """
        # test success
        uri = self.apiSuccess(self.node.getMasterUri(self.caller_id))
        self.check_uri(uri)

        # check against env, canonicalize for comparison
        import urlparse
        master_env = rosgraph.get_master_uri()
        if not master_env.endswith('/'):
            master_env = master_env + '/'
        self.assertEquals(urlparse.urlparse(master_env), urlparse.urlparse(uri))

        # test bad arity
        try:
            self.apiError(self.node.getMasterUri(self.caller_id, 'bad'))
        except xmlrpclib.Fault:
            pass
        try:
            self.apiError(self.node.getMasterUri())
        except xmlrpclib.Fault:
            pass

    def test_publisherUpdate(self):
        """
        validate node.publisherUpdate(caller_id, topic, uris) 
        """
        node = self.node
        probe_topic = rosgraph.names.ns_join(self.ns, 'probe_topic')
        fake_topic = rosgraph.names.ns_join(self.ns, 'fake_topic')        
        
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
        
        # test bad args
        try:
            self.apiError(node.publisherUpdate(self.caller_id, '/bad_topic', 'bad'))
        except xmlrpclib.Fault:
            pass
        try:
            self.apiError(node.publisherUpdate(self.caller_id, '/bad_topic', 2))
        except xmlrpclib.Fault:
            pass
        try:
            self.apiError(node.publisherUpdate(self.caller_id, '/bad_topic', False))                
        except xmlrpclib.Fault:
            pass
        try:
            self.apiError(node.publisherUpdate(self.caller_id, '/bad_topic', ['bad']))
        except xmlrpclib.Fault:
            pass

        # test bad arity
        try:
            self.apiError(node.publisherUpdate())
        except xmlrpclib.Fault:
            pass
        try:
            self.apiError(node.getBusStats(self.caller_id, 'bad'))
        except xmlrpclib.Fault:
            pass
        try:
            self.apiError(node.getBusStats())
        except xmlrpclib.Fault:
            pass

    def check_TCPROS(self, protocol_params):
        self.assert_(protocol_params, "no protocol params returned")
        self.assert_(type(protocol_params) == list, "protocol params must be a list: %s"%protocol_params)
        self.assertEquals(3, len(protocol_params), "TCPROS params should have length 3: %s"%protocol_params)
        self.assertEquals(protocol_params[0], TCPROS)
        # expect ['TCPROS', 1.2.3.4, 1234]
        self.assertEquals(protocol_params[0], TCPROS)            
        
    def testRequestTopic(self):
        node = self.node
        protocols = [[TCPROS]]

        publications = node.getPublications(self.caller_id)
        
        topics = self.required_pubs.keys()
        probe_topic = topics[0] if topics else None
        fake_topic = rosgraph.names.ns_join(self.ns, 'fake_topic')
        
        # currently only support TCPROS as we require all clients to support this
        protocols = [[TCPROS]]
        for topic in topics:
            self.check_TCPROS(self.apiSuccess(node.requestTopic(self.caller_id, topic, protocols)))
        protocols = [['FakeTransport', 1234, 5678], [TCPROS], ['AnotherFakeTransport']]
        # try each one more time, this time with more protocol choices
        for topic in topics:
            self.check_TCPROS(self.apiSuccess(node.requestTopic(self.caller_id, topic, protocols)))
            
        # test bad arity
        if probe_topic:
            try:
                self.apiError(node.requestTopic(self.caller_id, probe_topic, protocols, 'extra stuff'))
            except xmlrpclib.Fault:
                pass
            try:
                self.apiError(node.requestTopic(self.caller_id, probe_topic))
            except xmlrpclib.Fault:
                pass
            try:
                self.apiError(node.requestTopic(self.caller_id))
            except xmlrpclib.Fault:
                pass
            try:
                self.apiError(node.requestTopic())
            except xmlrpclib.Fault:
                pass

        # test bad args
        try:
            self.apiError(node.requestTopic(self.caller_id, 1, protocols))
        except xmlrpclib.Fault:
            pass
        try:
            self.apiError(node.requestTopic(self.caller_id, '', protocols))
        except xmlrpclib.Fault:
            pass
        try:
            self.apiError(node.requestTopic(self.caller_id, fake_topic, protocols))
        except xmlrpclib.Fault:
            pass
        try:
            self.apiError(node.requestTopic(self.caller_id, probe_topic, 'fake-protocols')) 
        except xmlrpclib.Fault:
            pass

        
    def test_getBusInfo(self):
        #TODO: finish
        # there should be a connection to rosout
        
        # test bad arity
        try:
            self.apiError(self.node.getBusInfo(self.caller_id, 'bad'))
        except xmlrpclib.Fault:
            pass
        try:
            self.apiError(self.node.getBusInfo())
        except xmlrpclib.Fault:
            pass

        
    ## test the state of the master based on expected node registration
    def test_registrations(self):
        # setUp() ensures the node has registered with the master

        # check actual URIs
        node_name = self.test_node
        pubs, subs, srvs = self.master.getSystemState()
        pub_topics = [t for t, _ in pubs]
        sub_topics = [t for t, _ in subs]
        
        # make sure all required topics are registered
        for t in self.required_pubs:
            self.assert_(t in pub_topics, "node did not register publication %s on master"%(t))
        for t in self.required_subs:
            self.assert_(t in sub_topics, "node did not register subscription %s on master"%(t))
        
        # check for node URI on master
        for topic, node_list in pubs:
            if topic in self.required_pubs:
                self.assert_(node_name in node_list, "%s not in %s"%(self.node_api, node_list))
        for topic, node_list in subs:
            if topic in self.required_subs:
                self.assert_(node_name in node_list, "%s not in %s"%(self.node_api, node_list))
        for service, srv_list in srvs:
            #TODO: no service tests yet
            pass

if __name__ == '__main__':
    rosunit.unitrun('rosmaster', sys.argv[0], TestSlaveApi)
