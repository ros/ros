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
import time

# mock of subscription tests
class ThreadPoolMock(object):
    def queue_task(*args): pass

class TestRosmasterRegistrations(unittest.TestCase):

    def test_NodeRef_services(self):
        from rosmaster.registrations import NodeRef, Registrations
        n = NodeRef('n1', 'http://localhost:1234')
        # test services
        n.add(Registrations.SERVICE, 'add_two_ints')
        self.failIf(n.is_empty())
        self.assert_('add_two_ints' in n.services)
        self.assertEquals(['add_two_ints'], n.services)
        
        n.add(Registrations.SERVICE, 'add_three_ints')
        self.failIf(n.is_empty())
        self.assert_('add_three_ints' in n.services)
        self.assert_('add_two_ints' in n.services)

        n.remove(Registrations.SERVICE, 'add_two_ints')
        self.assert_('add_three_ints' in n.services)
        self.assertEquals(['add_three_ints'], n.services)
        self.failIf('add_two_ints' in n.services)
        self.failIf(n.is_empty())
        
        n.remove(Registrations.SERVICE, 'add_three_ints')        
        self.failIf('add_three_ints' in n.services)
        self.failIf('add_two_ints' in n.services)
        self.assertEquals([], n.services)
        self.assert_(n.is_empty())

    def test_NodeRef_subs(self):
        from rosmaster.registrations import NodeRef, Registrations
        n = NodeRef('n1', 'http://localhost:1234')
        # test topic suscriptions
        n.add(Registrations.TOPIC_SUBSCRIPTIONS, 'topic1')
        self.failIf(n.is_empty())
        self.assert_('topic1' in n.topic_subscriptions)
        self.assertEquals(['topic1'], n.topic_subscriptions)
        
        n.add(Registrations.TOPIC_SUBSCRIPTIONS, 'topic2')
        self.failIf(n.is_empty())
        self.assert_('topic2' in n.topic_subscriptions)
        self.assert_('topic1' in n.topic_subscriptions)

        n.remove(Registrations.TOPIC_SUBSCRIPTIONS, 'topic1')
        self.assert_('topic2' in n.topic_subscriptions)
        self.assertEquals(['topic2'], n.topic_subscriptions)
        self.failIf('topic1' in n.topic_subscriptions)
        self.failIf(n.is_empty())
        
        n.remove(Registrations.TOPIC_SUBSCRIPTIONS, 'topic2')        
        self.failIf('topic2' in n.topic_subscriptions)
        self.failIf('topic1' in n.topic_subscriptions)
        self.assertEquals([], n.topic_subscriptions)
        self.assert_(n.is_empty())

    def test_NodeRef_pubs(self):
        from rosmaster.registrations import NodeRef, Registrations
        n = NodeRef('n1', 'http://localhost:1234')
        # test topic publications
        n.add(Registrations.TOPIC_PUBLICATIONS, 'topic1')
        self.failIf(n.is_empty())
        self.assert_('topic1' in n.topic_publications)
        self.assertEquals(['topic1'], n.topic_publications)
        
        n.add(Registrations.TOPIC_PUBLICATIONS, 'topic2')
        self.failIf(n.is_empty())
        self.assert_('topic2' in n.topic_publications)
        self.assert_('topic1' in n.topic_publications)

        n.remove(Registrations.TOPIC_PUBLICATIONS, 'topic1')
        self.assert_('topic2' in n.topic_publications)
        self.assertEquals(['topic2'], n.topic_publications)
        self.failIf('topic1' in n.topic_publications)
        self.failIf(n.is_empty())
        
        n.remove(Registrations.TOPIC_PUBLICATIONS, 'topic2')        
        self.failIf('topic2' in n.topic_publications)
        self.failIf('topic1' in n.topic_publications)
        self.assertEquals([], n.topic_publications)
        self.assert_(n.is_empty())

    def test_NodeRef_base(self):
        import rosmaster.exceptions
        from rosmaster.registrations import NodeRef, Registrations
        n = NodeRef('n1', 'http://localhost:1234')
        self.assertEquals('http://localhost:1234', n.api)
        self.assertEquals([], n.param_subscriptions)
        self.assertEquals([], n.topic_subscriptions)
        self.assertEquals([], n.topic_publications)
        self.assertEquals([], n.services)
        self.assert_(n.is_empty())

        try:
            n.add(12345, 'topic')
            self.fail("should have failed with invalid type")
        except rosmaster.exceptions.InternalException: pass
        try:
            n.remove(12345, 'topic')
            self.fail("should have failed with invalid type")
        except rosmaster.exceptions.InternalException: pass

        n.add(Registrations.TOPIC_PUBLICATIONS, 'topic1')
        n.add(Registrations.TOPIC_PUBLICATIONS, 'topic2')
        n.add(Registrations.TOPIC_SUBSCRIPTIONS, 'topic2')        
        n.add(Registrations.TOPIC_SUBSCRIPTIONS, 'topic3')        
        n.add(Registrations.PARAM_SUBSCRIPTIONS, 'topic4')        
        n.add(Registrations.SERVICE, 'serv')        
        self.failIf(n.is_empty())

        n.clear()
        self.assert_(n.is_empty())        

    def test_NodeRef_param_subs(self):
        from rosmaster.registrations import NodeRef, Registrations
        n = NodeRef('n1', 'http://localhost:1234')
        # test param suscriptions
        n.add(Registrations.PARAM_SUBSCRIPTIONS, 'param1')
        self.failIf(n.is_empty())
        self.assert_('param1' in n.param_subscriptions)
        self.assertEquals(['param1'], n.param_subscriptions)
        
        n.add(Registrations.PARAM_SUBSCRIPTIONS, 'param2')
        self.failIf(n.is_empty())
        self.assert_('param2' in n.param_subscriptions)
        self.assert_('param1' in n.param_subscriptions)

        n.remove(Registrations.PARAM_SUBSCRIPTIONS, 'param1')
        self.assert_('param2' in n.param_subscriptions)
        self.assertEquals(['param2'], n.param_subscriptions)
        self.failIf('param1' in n.param_subscriptions)
        self.failIf(n.is_empty())
        
        n.remove(Registrations.PARAM_SUBSCRIPTIONS, 'param2')        
        self.failIf('param2' in n.param_subscriptions)
        self.failIf('param1' in n.param_subscriptions)
        self.assertEquals([], n.param_subscriptions)
        self.assert_(n.is_empty())

    ## subroutine of registration tests that test topic/param type Reg objects
    ## @param r Registrations: initialized registrations object to test
    def _subtest_Registrations_basic(self, r):
        #NOTE: no real difference between topic and param names, so tests are reusable

        # - note that we've updated node1's API
        r.register('topic1', 'node1', 'http://node1:5678')
        self.assert_('topic1' in r) # test contains
        self.assert_(r.has_key('topic1')) # test contains
        self.assertEquals(['topic1'], [k for k in r.iterkeys()])
        self.assertEquals(['http://node1:5678'], r.get_apis('topic1'))
        self.assertEquals([('node1', 'http://node1:5678')], r['topic1'])
        self.failIf(not r) #test nonzero
        self.assertEquals(None, r.get_service_api('topic1')) #make sure no contamination
        self.assertEquals([['topic1', ['node1']]], r.get_state())

        r.register('topic1', 'node2', 'http://node2:5678')
        self.assertEquals(['topic1'], [k for k in r.iterkeys()])        
        self.assertEquals(['topic1'], [k for k in r.iterkeys()])
        self.assertEquals(2, len(r.get_apis('topic1')))
        self.assert_('http://node1:5678' in r.get_apis('topic1'))
        self.assert_('http://node2:5678' in r.get_apis('topic1'))
        self.assertEquals(2, len(r['topic1']))
        self.assert_(('node1', 'http://node1:5678') in r['topic1'], r['topic1'])
        self.assert_(('node2', 'http://node2:5678') in r['topic1'])                
        self.assertEquals([['topic1', ['node1', 'node2']]], r.get_state())

        # TODO: register second topic
        r.register('topic2', 'node3', 'http://node3:5678')
        self.assert_('topic2' in r) # test contains
        self.assert_(r.has_key('topic2')) # test contains
        self.assert_('topic1' in [k for k in r.iterkeys()])
        self.assert_('topic2' in [k for k in r.iterkeys()])
        self.assertEquals(['http://node3:5678'], r.get_apis('topic2'))
        self.assertEquals([('node3', 'http://node3:5678')], r['topic2'])
        self.failIf(not r) #test nonzero
        self.assert_(['topic1', ['node1', 'node2']] in r.get_state(), r.get_state())
        self.assert_(['topic2', ['node3']] in r.get_state(), r.get_state())
        
        # Unregister

        # - fail if node is not registered
        code, _, val = r.unregister('topic1', 'node3', 'http://node3:5678')
        self.assertEquals(0, val)
        # - fail if topic is not registered by that node
        code, _, val = r.unregister('topic2', 'node2', 'http://node2:5678')
        self.assertEquals(0, val)
        # - fail if URI does not match
        code, _, val = r.unregister('topic2', 'node2', 'http://fakenode2:5678')
        self.assertEquals(0, val)

        # - unregister node2
        code, _, val = r.unregister('topic1', 'node1', 'http://node1:5678')
        self.assertEquals(1, code)
        self.assertEquals(1, val)
        self.assert_('topic1' in r) # test contains
        self.assert_(r.has_key('topic1')) 
        self.assert_('topic1' in [k for k in r.iterkeys()])
        self.assert_('topic2' in [k for k in r.iterkeys()])
        self.assertEquals(['http://node2:5678'], r.get_apis('topic1'))
        self.assertEquals([('node2', 'http://node2:5678')], r['topic1'])
        self.failIf(not r) #test nonzero
        self.assert_(['topic1', ['node2']] in r.get_state())
        self.assert_(['topic2', ['node3']] in r.get_state())        

        code, _, val = r.unregister('topic1', 'node2', 'http://node2:5678')
        self.assertEquals(1, code)
        self.assertEquals(1, val)
        self.failIf('topic1' in r) # test contains
        self.failIf(r.has_key('topic1')) 
        self.assertEquals(['topic2'], [k for k in r.iterkeys()])
        self.assertEquals([], r.get_apis('topic1'))
        self.assertEquals([], r['topic1'])
        self.failIf(not r) #test nonzero
        self.assertEquals([['topic2', ['node3']]], r.get_state())

        # clear out last reg
        code, _, val = r.unregister('topic2', 'node3', 'http://node3:5678')
        self.assertEquals(1, code)
        self.assertEquals(1, val)
        self.failIf('topic2' in r) # test contains
        self.assert_(not r)
        self.assertEquals([], r.get_state())        
        
    def test_Registrations(self):
        import rosmaster.exceptions
        from rosmaster.registrations import Registrations
        types = [Registrations.TOPIC_SUBSCRIPTIONS,
                 Registrations.TOPIC_PUBLICATIONS,
                 Registrations.SERVICE,
                 Registrations.PARAM_SUBSCRIPTIONS]
        # test enums
        self.assertEquals(4, len(set(types)))
        try:
            r = Registrations(-1)
            self.fail("Registrations accepted invalid type")
        except rosmaster.exceptions.InternalException, e: pass
        
        for t in types:
            r = Registrations(t)
            self.assertEquals(t, r.type)
            self.assert_(not r) #test nonzero
            self.failIf('topic1' in r) #test contains            
            self.failIf(r.has_key('topic1')) #test has_key
            self.failIf([k for k in r.iterkeys()]) #no keys
            self.assertEquals(None, r.get_service_api('non-existent'))

        # Test topic subs
        r = Registrations(Registrations.TOPIC_SUBSCRIPTIONS)
        self._subtest_Registrations_basic(r)
        r = Registrations(Registrations.TOPIC_PUBLICATIONS)        
        self._subtest_Registrations_basic(r)
        r = Registrations(Registrations.PARAM_SUBSCRIPTIONS)        
        self._subtest_Registrations_basic(r)

        r = Registrations(Registrations.SERVICE)        
        self._subtest_Registrations_services(r)

    def test_RegistrationManager_services(self):
        from rosmaster.registrations import Registrations, RegistrationManager
        rm = RegistrationManager(ThreadPoolMock())
        
        self.assertEquals(None, rm.get_node('caller1'))

        # do an unregister first, before service_api is initialized
        code, msg, val = rm.unregister_service('s1', 'caller1', 'rosrpc://one:1234')
        self.assertEquals(1, code)
        self.assertEquals(0, val)        
        
        rm.register_service('s1', 'caller1', 'http://one:1234', 'rosrpc://one:1234')
        self.assert_(rm.services.has_key('s1'))
        self.assertEquals('rosrpc://one:1234', rm.services.get_service_api('s1')) 
        self.assertEquals('http://one:1234', rm.get_node('caller1').api)
        self.assertEquals([['s1', ['caller1']]], rm.services.get_state())
        
        # - verify that changed caller_api updates ref
        rm.register_service('s1', 'caller1', 'http://oneB:1234', 'rosrpc://one:1234')
        self.assert_(rm.services.has_key('s1'))
        self.assertEquals('rosrpc://one:1234', rm.services.get_service_api('s1'))        
        self.assertEquals('http://oneB:1234', rm.get_node('caller1').api)
        self.assertEquals([['s1', ['caller1']]], rm.services.get_state())
        
        # - verify that changed service_api updates ref
        rm.register_service('s1', 'caller1', 'http://oneB:1234', 'rosrpc://oneB:1234')
        self.assert_(rm.services.has_key('s1'))
        self.assertEquals('rosrpc://oneB:1234', rm.services.get_service_api('s1'))        
        self.assertEquals('http://oneB:1234', rm.get_node('caller1').api)
        self.assertEquals([['s1', ['caller1']]], rm.services.get_state())
        
        rm.register_service('s2', 'caller2', 'http://two:1234', 'rosrpc://two:1234')
        self.assertEquals('http://two:1234', rm.get_node('caller2').api)

        # - unregister should be noop if service api does not match
        code, msg, val = rm.unregister_service('s2', 'caller2', 'rosrpc://b:1234')
        self.assertEquals(1, code)
        self.assertEquals(0, val)        
        self.assert_(rm.services.has_key('s2'))
        self.assertEquals('http://two:1234', rm.get_node('caller2').api)        
        self.assertEquals('rosrpc://two:1234', rm.services.get_service_api('s2'))
        
        # - unregister should be noop if service is unknown
        code, msg, val = rm.unregister_service('unknown', 'caller2', 'rosrpc://two:1234')
        self.assertEquals(1, code)
        self.assertEquals(0, val)        
        self.assert_(rm.services.has_key('s2'))
        self.assertEquals('http://two:1234', rm.get_node('caller2').api)        
        self.assertEquals('rosrpc://two:1234', rm.services.get_service_api('s2'))

        # - unregister should clear all knowledge of caller2
        code,msg, val = rm.unregister_service('s2', 'caller2', 'rosrpc://two:1234')
        self.assertEquals(1, code)
        self.assertEquals(1, val)        
        self.assert_(rm.services.has_key('s1')) 
        self.failIf(rm.services.has_key('s2'))        
        self.assertEquals(None, rm.get_node('caller2'))

        code, msg, val = rm.unregister_service('s1', 'caller1', 'rosrpc://oneB:1234')
        self.assertEquals(1, code)        
        self.assertEquals(1, val)        
        self.assert_(not rm.services.__nonzero__())
        self.failIf(rm.services.has_key('s1'))        
        self.assertEquals(None, rm.get_node('caller1'))        

    def test_RegistrationManager_topic_pub(self):
        from rosmaster.registrations import Registrations, RegistrationManager
        rm = RegistrationManager(ThreadPoolMock())
        self.subtest_RegistrationManager(rm, rm.publishers, rm.register_publisher, rm.unregister_publisher)
        
    def test_RegistrationManager_topic_sub(self):
        from rosmaster.registrations import Registrations, RegistrationManager
        rm = RegistrationManager(ThreadPoolMock())
        self.subtest_RegistrationManager(rm, rm.subscribers, rm.register_subscriber, rm.unregister_subscriber)
    def test_RegistrationManager_param_sub(self):
        from rosmaster.registrations import Registrations, RegistrationManager
        rm = RegistrationManager(ThreadPoolMock())
        self.subtest_RegistrationManager(rm, rm.param_subscribers, rm.register_param_subscriber, rm.unregister_param_subscriber)
        
    def subtest_RegistrationManager(self, rm, r, register, unregister):
        self.assertEquals(None, rm.get_node('caller1'))

        register('key1', 'caller1', 'http://one:1234')
        self.assert_(r.has_key('key1'))
        self.assertEquals('http://one:1234', rm.get_node('caller1').api)
        self.assertEquals([['key1', ['caller1']]], r.get_state())
        
        # - verify that changed caller_api updates ref
        register('key1', 'caller1', 'http://oneB:1234')
        self.assert_(r.has_key('key1'))
        self.assertEquals('http://oneB:1234', rm.get_node('caller1').api)
        self.assertEquals([['key1', ['caller1']]], r.get_state())
        
        register('key2', 'caller2', 'http://two:1234')
        self.assertEquals('http://two:1234', rm.get_node('caller2').api)

        # - unregister should be noop if caller api does not match
        code, msg, val = unregister('key2', 'caller2', 'http://b:1234')
        self.assertEquals(1, code)
        self.assertEquals(0, val)        
        self.assertEquals('http://two:1234', rm.get_node('caller2').api)        
        
        # - unregister should be noop if key is unknown
        code, msg, val = unregister('unknown', 'caller2', 'http://two:1234')
        self.assertEquals(1, code)
        self.assertEquals(0, val)        
        self.assert_(r.has_key('key2'))
        self.assertEquals('http://two:1234', rm.get_node('caller2').api)        

        # - unregister should be noop if unknown node
        code, msg, val = rm.unregister_publisher('key2', 'unknown', 'http://unknown:1')
        self.assertEquals(1, code)
        self.assertEquals(0, val)        
        self.assert_(r.has_key('key2'))

        # - unregister should clear all knowledge of caller2
        code,msg, val = unregister('key2', 'caller2', 'http://two:1234')
        self.assertEquals(1, code)
        self.assertEquals(1, val)        
        self.assert_(r.has_key('key1')) 
        self.failIf(r.has_key('key2'))        
        self.assertEquals(None, rm.get_node('caller2'))

        code, msg, val = unregister('key1', 'caller1', 'http://oneB:1234')
        self.assertEquals(1, code)        
        self.assertEquals(1, val)        
        self.assert_(not r.__nonzero__())
        self.failIf(r.has_key('key1'))        
        self.assertEquals(None, rm.get_node('caller1'))        

    def test_RegistrationManager_base(self):
        import rosmaster.exceptions
        from rosmaster.registrations import Registrations, RegistrationManager
        threadpool = ThreadPoolMock()

        rm = RegistrationManager(threadpool)
        self.assert_(isinstance(rm.services, Registrations))
        self.assertEquals(Registrations.SERVICE, rm.services.type)
        self.assert_(isinstance(rm.param_subscribers, Registrations))
        self.assertEquals(Registrations.PARAM_SUBSCRIPTIONS, rm.param_subscribers.type)
        self.assert_(isinstance(rm.subscribers, Registrations))
        self.assertEquals(Registrations.TOPIC_SUBSCRIPTIONS, rm.subscribers.type)
        self.assert_(isinstance(rm.subscribers, Registrations))
        self.assertEquals(Registrations.TOPIC_PUBLICATIONS, rm.publishers.type)
        self.assert_(isinstance(rm.publishers, Registrations))

        #test auto-clearing of registrations if node API changes
        rm.register_publisher('pub1', 'caller1', 'http://one:1')
        rm.register_publisher('pub1', 'caller2', 'http://two:1')
        rm.register_publisher('pub1', 'caller3', 'http://three:1')
        rm.register_subscriber('sub1', 'caller1', 'http://one:1')
        rm.register_subscriber('sub1', 'caller2', 'http://two:1')
        rm.register_subscriber('sub1', 'caller3', 'http://three:1')
        rm.register_param_subscriber('p1', 'caller1', 'http://one:1')
        rm.register_param_subscriber('p1', 'caller2', 'http://two:1')
        rm.register_param_subscriber('p1', 'caller3', 'http://three:1')
        rm.register_service('s1', 'caller1', 'http://one:1', 'rosrpc://one:1')
        self.assertEquals('http://one:1', rm.get_node('caller1').api)
        self.assertEquals('http://two:1', rm.get_node('caller2').api)
        self.assertEquals('http://three:1', rm.get_node('caller3').api)        

        # - first, make sure that changing rosrpc URI does not erase state
        rm.register_service('s1', 'caller1', 'http://one:1', 'rosrpc://oneB:1')
        n = rm.get_node('caller1')
        self.assertEquals(['pub1'], n.topic_publications)
        self.assertEquals(['sub1'], n.topic_subscriptions)
        self.assertEquals(['p1'], n.param_subscriptions)                
        self.assertEquals(['s1'], n.services)
        self.assert_('http://one:1' in rm.publishers.get_apis('pub1'))
        self.assert_('http://one:1' in rm.subscribers.get_apis('sub1'))
        self.assert_('http://one:1' in rm.param_subscribers.get_apis('p1'))
        self.assert_('http://one:1' in rm.services.get_apis('s1'))

        # - also, make sure unregister does not erase state if API changed
        rm.unregister_publisher('pub1', 'caller1', 'http://not:1')
        self.assert_('http://one:1' in rm.publishers.get_apis('pub1'))
        rm.unregister_subscriber('sub1', 'caller1', 'http://not:1')
        self.assert_('http://one:1' in rm.subscribers.get_apis('sub1'))
        rm.unregister_param_subscriber('p1', 'caller1', 'http://not:1')
        self.assert_('http://one:1' in rm.param_subscribers.get_apis('p1'))
        rm.unregister_service('sub1', 'caller1', 'rosrpc://not:1')
        self.assert_('http://one:1' in rm.services.get_apis('s1'))
        
        
        # erase caller1 sub/srvs/params via register_publisher
        rm.register_publisher('pub1', 'caller1', 'http://newone:1')
        self.assertEquals('http://newone:1', rm.get_node('caller1').api)        
        # - check node ref
        n = rm.get_node('caller1')
        self.assertEquals(['pub1'], n.topic_publications)
        self.assertEquals([], n.services)
        self.assertEquals([], n.topic_subscriptions)
        self.assertEquals([], n.param_subscriptions)
        # - checks publishers
        self.assert_('http://newone:1' in rm.publishers.get_apis('pub1'))
        # - checks subscribers
        self.assert_(rm.subscribers.has_key('sub1'))
        self.failIf('http://one:1' in rm.subscribers.get_apis('sub1'))
        # - checks param subscribers
        self.assert_(rm.param_subscribers.has_key('p1'))
        self.failIf('http://one:1' in rm.param_subscribers.get_apis('p1'))

        # erase caller2 pub/sub/params via register_service
        # - initial state
        self.assert_('http://two:1' in rm.publishers.get_apis('pub1'))
        self.assert_('http://two:1' in rm.subscribers.get_apis('sub1'))
        self.assert_('http://two:1' in rm.param_subscribers.get_apis('p1'))
        # - change ownership of s1 to caller2
        rm.register_service('s1', 'caller2', 'http://two:1', 'rosrpc://two:1')
        self.assert_('http://two:1' in rm.services.get_apis('s1'))
        self.assert_('http://two:1' in rm.publishers.get_apis('pub1'))
        self.assert_('http://two:1' in rm.subscribers.get_apis('sub1'))
        self.assert_('http://two:1' in rm.param_subscribers.get_apis('p1'))
        
        rm.register_service('s1', 'caller2', 'http://newtwo:1', 'rosrpc://newtwo:1')
        self.assertEquals('http://newone:1', rm.get_node('caller1').api)        
        # - check node ref
        n = rm.get_node('caller2')
        self.assertEquals([], n.topic_publications)
        self.assertEquals(['s1'], n.services)
        self.assertEquals([], n.topic_subscriptions)
        self.assertEquals([], n.param_subscriptions)
        # - checks publishers
        self.assert_(rm.publishers.has_key('pub1'))
        self.failIf('http://two:1' in rm.publishers.get_apis('pub1'))
        # - checks subscribers
        self.assert_(rm.subscribers.has_key('sub1'))
        self.failIf('http://two:1' in rm.subscribers.get_apis('sub1'))
        self.assertEquals([['sub1', ['caller3']]], rm.subscribers.get_state())
        # - checks param subscribers
        self.assert_(rm.param_subscribers.has_key('p1'))
        self.failIf('http://two:1' in rm.param_subscribers.get_apis('p1'))
        self.assertEquals([['p1', ['caller3']]], rm.param_subscribers.get_state())

        
    def test_Registrations_unregister_all(self):
        import rosmaster.exceptions
        from rosmaster.registrations import Registrations

        r = Registrations(Registrations.TOPIC_SUBSCRIPTIONS)        
        for k in ['topic1', 'topic1b', 'topic1c', 'topic1d']:        
            r.register(k, 'node1', 'http://node1:5678')
        r.register('topic2', 'node2', 'http://node2:5678')
        r.unregister_all('node1')
        self.failIf(not r)
        for k in ['topic1', 'topic1b', 'topic1c', 'topic1d']:        
            self.failIf(r.has_key(k))
        self.assertEquals(['topic2'], [k for k in r.iterkeys()])
        
        r = Registrations(Registrations.TOPIC_PUBLICATIONS)        
        for k in ['topic1', 'topic1b', 'topic1c', 'topic1d']:        
            r.register(k, 'node1', 'http://node1:5678')
        r.register('topic2', 'node2', 'http://node2:5678')
        r.unregister_all('node1')
        self.failIf(not r)
        for k in ['topic1', 'topic1b', 'topic1c', 'topic1d']:        
            self.failIf(r.has_key(k))
        self.assertEquals(['topic2'], [k for k in r.iterkeys()])

        r = Registrations(Registrations.PARAM_SUBSCRIPTIONS)        
        r.register('param2', 'node2', 'http://node2:5678')
        for k in ['param1', 'param1b', 'param1c', 'param1d']:
            r.register(k, 'node1', 'http://node1:5678')
        r.unregister_all('node1')
        self.failIf(not r)
        for k in ['param1', 'param1b', 'param1c', 'param1d']:
            self.failIf(r.has_key(k))
        self.assertEquals(['param2'], [k for k in r.iterkeys()])
        
        r = Registrations(Registrations.SERVICE)        
        for k in ['service1', 'service1b', 'service1c', 'service1d']:
            r.register(k, 'node1', 'http://node1:5678', 'rosrpc://node1:1234')
        r.register('service2', 'node2', 'http://node2:5678', 'rosrpc://node2:1234')
        r.unregister_all('node1')
        self.failIf(not r)
        for k in ['service1', 'service1b', 'service1c', 'service1d']:
            self.failIf(r.has_key(k))
            self.assertEquals(None, r.get_service_api(k))
        self.assertEquals(['service2'], [k for k in r.iterkeys()])
        self.assertEquals('rosrpc://node2:1234', r.get_service_api('service2'))

    def _subtest_Registrations_services(self, r):
        import rosmaster.exceptions

        # call methods that use service_api_map, make sure they are guarded against lazy-init
        self.assertEquals(None, r.get_service_api('s1'))
        r.unregister_all('node1')

        # do an unregister first, before service_api is initialized
        code, msg, val = r.unregister('s1', 'caller1', None, 'rosrpc://one:1234')
        self.assertEquals(1, code)
        self.assertEquals(0, val)        

        try:
            r.register('service1', 'node1', 'http://node1:5678')
            self.fail("should require service_api")
        except rosmaster.exceptions.InternalException: pass
        
        r.register('service1', 'node1', 'http://node1:5678', 'rosrpc://node1:1234')
        
        self.assert_('service1' in r) # test contains
        self.assert_(r.has_key('service1')) # test contains
        self.assertEquals(['service1'], [k for k in r.iterkeys()])
        self.assertEquals(['http://node1:5678'], r.get_apis('service1'))
        self.assertEquals('rosrpc://node1:1234', r.get_service_api('service1'))
        self.assertEquals([('node1', 'http://node1:5678')], r['service1'])
        self.failIf(not r) #test nonzero
        self.assertEquals([['service1', ['node1']]], r.get_state())

        r.register('service1', 'node2', 'http://node2:5678', 'rosrpc://node2:1234')
        self.assertEquals(['service1'], [k for k in r.iterkeys()])
        self.assertEquals('rosrpc://node2:1234', r.get_service_api('service1'))
        self.assertEquals(['http://node2:5678'], r.get_apis('service1'))
        self.assertEquals([('node2', 'http://node2:5678')], r['service1'])
        self.assertEquals([['service1', ['node2']]], r.get_state())

        # register a second service
        r.register('service2', 'node3', 'http://node3:5678', 'rosrpc://node3:1234')
        self.assertEquals('rosrpc://node3:1234', r.get_service_api('service2'))
        self.assertEquals(2, len(r.get_state()))
        self.assert_(['service2', ['node3']] in r.get_state(), r.get_state())
        self.assert_(['service1', ['node2']] in r.get_state())
        
        # register a third service, second service for node2
        r.register('service1b', 'node2', 'http://node2:5678', 'rosrpc://node2:1234')
        self.assertEquals(3, len(r.get_state()))
        self.assert_(['service2', ['node3']] in r.get_state())
        self.assert_(['service1b', ['node2']] in r.get_state())
        self.assert_(['service1', ['node2']] in r.get_state())
        
        # Unregister
        try:
            r.unregister('service1', 'node2', 'http://node2:1234')
            self.fail("service_api param must be specified")
        except rosmaster.exceptions.InternalException: pass
        
        # - fail if service is not known
        code, _, val = r.unregister('unknown', 'node2', 'http://node2:5678', 'rosprc://node2:1234')
        self.assertEquals(0, val)
        # - fail if node is not registered
        code, _, val = r.unregister('service1', 'node3', 'http://node3:5678', 'rosrpc://node3:1234')
        self.assertEquals(0, val)
        # - fail if service API is different
        code, _, val = r.unregister('service1', 'node2', 'http://node2b:5678', 'rosrpc://node3:1234')
        self.assertEquals(0, val)

        # - unregister service2
        code, _, val = r.unregister('service2', 'node3', 'http://node3:5678', 'rosrpc://node3:1234')
        self.assertEquals(1, code)
        self.assertEquals(1, val)
        self.failIf('service2' in r) # test contains
        self.failIf(r.has_key('service2')) 
        self.assert_('service1' in [k for k in r.iterkeys()])
        self.assert_('service1b' in [k for k in r.iterkeys()])
        self.assertEquals([], r.get_apis('service2'))
        self.assertEquals([], r['service2'])
        self.failIf(not r) #test nonzero
        self.assertEquals(2, len(r.get_state()))
        self.failIf(['service2', ['node3']] in r.get_state())
        
        # - unregister node2
        code, _, val = r.unregister('service1', 'node2', 'http://node2:5678', 'rosrpc://node2:1234')
        self.assertEquals(1, code)
        self.assertEquals(1, val)
        self.failIf('service1' in r) # test contains
        self.failIf(r.has_key('service1')) 
        self.assertEquals(['service1b'], [k for k in r.iterkeys()])
        self.assertEquals([], r.get_apis('service1'))
        self.assertEquals([], r['service1'])
        self.failIf(not r) #test nonzero
        self.assertEquals([['service1b', ['node2']]], r.get_state())

        code, _, val = r.unregister('service1b', 'node2', 'http://node2:5678', 'rosrpc://node2:1234')
        self.assertEquals(1, code)
        self.assertEquals(1, val)
        self.failIf('service1' in r) # test contains
        self.failIf(r.has_key('service1')) 
        self.assertEquals([], [k for k in r.iterkeys()])
        self.assertEquals([], r.get_apis('service1'))
        self.assertEquals([], r['service1'])
        self.assert_(not r) #test nonzero
        self.assertEquals([], r.get_state())
        
