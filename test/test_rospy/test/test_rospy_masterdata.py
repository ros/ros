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
#
# Revision $Id$

import roslib; roslib.load_manifest('test_rospy')

import os
import sys
import struct
import unittest
import time

# mock of subscription tests
class ThreadPoolMock(object):
    def queue_task(*args): pass

class TestRospyMasterData(unittest.TestCase):

    def test_NodeRef(self):
        from rospy.masterdata import NodeRef
        n = NodeRef('http://localhost:1234')
        self.assertEquals('http://localhost:1234', n.api)
        self.assertEquals([], n.param_subscriptions)
        self.assertEquals([], n.topic_subscriptions)
        self.assertEquals([], n.topic_publications)
        self.assertEquals([], n.services)
        self.assert_(n.is_empty())

        # test services
        n.add_service('add_two_ints')
        self.failIf(n.is_empty())
        self.assert_('add_two_ints' in n.services)
        self.assertEquals(['add_two_ints'], n.services)
        
        n.add_service('add_three_ints')
        self.failIf(n.is_empty())
        self.assert_('add_three_ints' in n.services)
        self.assert_('add_two_ints' in n.services)

        n.remove_service('add_two_ints')
        self.assert_('add_three_ints' in n.services)
        self.assertEquals(['add_three_ints'], n.services)
        self.failIf('add_two_ints' in n.services)
        self.failIf(n.is_empty())
        
        n.remove_service('add_three_ints')        
        self.failIf('add_three_ints' in n.services)
        self.failIf('add_two_ints' in n.services)
        self.assertEquals([], n.services)
        self.assert_(n.is_empty())

        # test topic suscriptions
        n.add_topic_subscription('topic1')
        self.failIf(n.is_empty())
        self.assert_('topic1' in n.topic_subscriptions)
        self.assertEquals(['topic1'], n.topic_subscriptions)
        
        n.add_topic_subscription('topic2')
        self.failIf(n.is_empty())
        self.assert_('topic2' in n.topic_subscriptions)
        self.assert_('topic1' in n.topic_subscriptions)

        n.remove_topic_subscription('topic1')
        self.assert_('topic2' in n.topic_subscriptions)
        self.assertEquals(['topic2'], n.topic_subscriptions)
        self.failIf('topic1' in n.topic_subscriptions)
        self.failIf(n.is_empty())
        
        n.remove_topic_subscription('topic2')        
        self.failIf('topic2' in n.topic_subscriptions)
        self.failIf('topic1' in n.topic_subscriptions)
        self.assertEquals([], n.topic_subscriptions)
        self.assert_(n.is_empty())

        # test param suscriptions
        n.add_param_subscription('param1')
        self.failIf(n.is_empty())
        self.assert_('param1' in n.param_subscriptions)
        self.assertEquals(['param1'], n.param_subscriptions)
        
        n.add_param_subscription('param2')
        self.failIf(n.is_empty())
        self.assert_('param2' in n.param_subscriptions)
        self.assert_('param1' in n.param_subscriptions)

        n.remove_param_subscription('param1')
        self.assert_('param2' in n.param_subscriptions)
        self.assertEquals(['param2'], n.param_subscriptions)
        self.failIf('param1' in n.param_subscriptions)
        self.failIf(n.is_empty())
        
        n.remove_param_subscription('param2')        
        self.failIf('param2' in n.param_subscriptions)
        self.failIf('param1' in n.param_subscriptions)
        self.assertEquals([], n.param_subscriptions)
        self.assert_(n.is_empty())

        # test topic suscriptions
        n.add_topic_publication('topic1')
        self.failIf(n.is_empty())
        self.assert_('topic1' in n.topic_publications)
        self.assertEquals(['topic1'], n.topic_publications)
        
        n.add_topic_publication('topic2')
        self.failIf(n.is_empty())
        self.assert_('topic2' in n.topic_publications)
        self.assert_('topic1' in n.topic_publications)

        n.remove_topic_publication('topic1')
        self.assert_('topic2' in n.topic_publications)
        self.assertEquals(['topic2'], n.topic_publications)
        self.failIf('topic1' in n.topic_publications)
        self.failIf(n.is_empty())
        
        n.remove_topic_publication('topic2')        
        self.failIf('topic2' in n.topic_publications)
        self.failIf('topic1' in n.topic_publications)
        self.assertEquals([], n.topic_publications)
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
        self.assertEquals(['topic1'], [k for k in r.iterkeys()])
        self.assertEquals(['http://node2:5678'], r.get_apis('topic1'))
        self.assertEquals([('node2', 'http://node2:5678')], r['topic1'])
        self.failIf(not r) #test nonzero
        self.assertEquals([['topic1', ['node2']]], r.get_state())

        code, _, val = r.unregister('topic1', 'node2', 'http://node2:5678')
        self.assertEquals(1, code)
        self.assertEquals(1, val)
        self.failIf('topic1' in r) # test contains
        self.failIf(r.has_key('topic1')) 
        self.assertEquals([], [k for k in r.iterkeys()])
        self.assertEquals([], r.get_apis('topic1'))
        self.assertEquals([], r['topic1'])
        self.assert_(not r) #test nonzero
        self.assertEquals([], r.get_state())
        
    def test_Registrations(self):
        import rospy.exceptions
        from rospy.masterdata import Registrations, NodeRef
        types = [Registrations.TOPIC_SUBSCRIPTIONS,
                 Registrations.TOPIC_PUBLICATIONS,
                 Registrations.SERVICE,
                 Registrations.PARAM_SUBSCRIPTIONS]
        # test enums
        self.assertEquals(4, len(set(types)))
        threadpool = ThreadPoolMock()
        node_registrations = { 'node1': NodeRef('http://local:1234') }
        try:
            r = Registrations(-1, node_registrations, threadpool)
            self.fail("Registrations accepted invalid type")
        except rospy.exceptions.ROSInternalException, e: pass
        
        for t in types:
            r = Registrations(t, node_registrations, threadpool)
            self.assertEquals(t, r.type)
            self.assertEquals(node_registrations, r.node_registrations)
            self.assert_(not r) #test nonzero
            self.failIf('topic1' in r) #test contains            
            self.failIf(r.has_key('topic1')) #test has_key
            self.failIf([k for k in r.iterkeys()]) #no keys
            self.assertEquals(None, r.get_service_api('non-existent'))

        # Test topic subs
        r = Registrations(Registrations.TOPIC_SUBSCRIPTIONS, node_registrations.copy(), threadpool)
        self._subtest_Registrations_basic(r)
        r = Registrations(Registrations.TOPIC_PUBLICATIONS, node_registrations.copy(), threadpool)        
        self._subtest_Registrations_basic(r)
        r = Registrations(Registrations.PARAM_SUBSCRIPTIONS, node_registrations.copy(), threadpool)        
        self._subtest_Registrations_basic(r)

        #TODO: service API tests
        #TODO: auto-clearing of registrations if node API changes
        
if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_rospy', sys.argv[0], TestRospyMasterData, coverage_packages=['rospy.masterdata'])
