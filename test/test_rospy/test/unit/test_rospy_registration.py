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

import os
import sys
import struct
import unittest
import time
import cStringIO
        
class TestRospyRegistration(unittest.TestCase):

    def test_get_set_topic_manager(self):
        from rospy.impl.registration import get_topic_manager, set_topic_manager
        # rospy initialization sets this, but it is out of scope of
        # rospy.impl.registrations to test its value
        orig = get_topic_manager()
        try:
            self.assert_(orig is not None)
            class TopicManager(object): pass
            x = TopicManager()
        # currently untyped
            set_topic_manager(x)
            self.assertEquals(x, get_topic_manager())
            set_topic_manager(None)
            self.assert_(get_topic_manager() is None)
        finally:
            set_topic_manager(orig)            

    def test_get_set_service_manager(self):
        from rospy.impl.registration import get_service_manager, set_service_manager
        # rospy initialization sets this, but it is out of scope of
        # rospy.impl.registrations to test its value
        try:
            orig = get_service_manager()
            self.assert_(orig is not None)
            class ServiceManager(object): pass
            x = ServiceManager()
            # currently untyped
            set_service_manager(x)
            self.assertEquals(x, get_service_manager())
            set_service_manager(None)
            self.assert_(get_service_manager() is None)
        finally:
            set_service_manager(orig)            
        
    def test_Registration(self):
        # nothing to test here really
        from rospy.impl.registration import Registration
        self.assertEquals('pub', Registration.PUB)
        self.assertEquals('sub', Registration.SUB)
        self.assertEquals('srv', Registration.SRV)        
        r = Registration()
        self.assertEquals('pub', r.PUB)
        self.assertEquals('sub', r.SUB)
        self.assertEquals('srv', r.SRV)        
    def test_RegistrationListener(self):
        from rospy.impl.registration import RegistrationListener
        #RegistrationListener is just an API, nothing to test here
        l = RegistrationListener()
        l.reg_added('name', 'data_type', 'reg_type')
        l.reg_removed('name', 'data_type', 'reg_type')
        
    def test_RegistrationListeners(self):
        from rospy.impl.registration import RegistrationListeners, RegistrationListener

        class Mock(RegistrationListener):
            def __init__(self):
                self.args = []
            def reg_added(self, name, data_type_or_uri, reg_type): 
                self.args = ['added', name, data_type_or_uri, reg_type]
            def reg_removed(self, name, data_type_or_uri, reg_type):
                self.args = ['removed', name, data_type_or_uri, reg_type]
        class BadMock(RegistrationListener):
            def reg_added(self, name, data_type_or_uri, reg_type):
                raise Exception("haha!")
            def reg_removed(self, name, data_type_or_uri, reg_type):
                raise Exception("haha!")                

        r = RegistrationListeners()
        self.assertEquals([], r.listeners)

        try:
            r.lock.acquire()
        finally:
            r.lock.release()

        r.notify_added('n1', 'dtype1', 'rtype1')
        r.notify_removed('n1', 'dtype1', 'rtype1')
        l1 = Mock()
        l2 = Mock()
        
        r.add_listener(l1)
        self.assertEquals([l1], r.listeners)

        self.assertEquals([], l1.args)
        r.notify_added('n2', 'dtype2', 'rtype2')
        self.assertEquals(['added', 'n2', 'dtype2', 'rtype2'], l1.args)
        r.notify_removed('n2', 'dtype2', 'rtype2')
        self.assertEquals(['removed', 'n2', 'dtype2', 'rtype2'], l1.args)

        r.add_listener(l2)
        self.assert_(l2 in r.listeners)
        self.assert_(l1 in r.listeners)
        self.assertEquals(2, len(r.listeners))

        self.assertEquals([], l2.args)
        r.notify_added('n3', 'dtype3', 'rtype3')
        self.assertEquals(['added', 'n3', 'dtype3', 'rtype3'], l2.args)        
        self.assertEquals(['added', 'n3', 'dtype3', 'rtype3'], l1.args)
        r.notify_removed('n3', 'dtype3', 'rtype3')
        self.assertEquals(['removed', 'n3', 'dtype3', 'rtype3'], l2.args)
        self.assertEquals(['removed', 'n3', 'dtype3', 'rtype3'], l1.args)

        # l3 raises exceptions, make sure they don't break anything
        l3 = BadMock()
        r.add_listener(l3)
        self.assert_(l3 in r.listeners)
        self.assert_(l2 in r.listeners)
        self.assert_(l1 in r.listeners)
        self.assertEquals(3, len(r.listeners))

        r.notify_added('n4', 'dtype4', 'rtype4')
        self.assertEquals(['added', 'n4', 'dtype4', 'rtype4'], l2.args)        
        self.assertEquals(['added', 'n4', 'dtype4', 'rtype4'], l1.args)
        r.notify_removed('n4', 'dtype4', 'rtype4')
        self.assertEquals(['removed', 'n4', 'dtype4', 'rtype4'], l2.args)
        self.assertEquals(['removed', 'n4', 'dtype4', 'rtype4'], l1.args)
        
    def test_get_registration_listeners(self):
        from rospy.impl.registration import RegistrationListeners, get_registration_listeners
        r = get_registration_listeners()
        self.assert_(isinstance(r, RegistrationListeners))

    def test_RegManager(self):
        from rospy.impl.registration import RegManager
        class MockHandler(object):
            def __init__(self):
                self.done = False
                self.args = []
            def _connect_topic(self, topic, uri):
                self.args.append(['_connect_topic', topic, uri])
                # trip to done on connect topic so we can exit loop
                self.done = True
                return 1, 'msg', 1
        # bad return value for _connect_topic
        class BadHandler(object):
            def __init__(self):
                self.done = False
            def _connect_topic(self, topic, uri):
                self.done = True
                return None
        # bad code for _connect_topic
        class BadHandler2(object):
            def __init__(self):
                self.done = False
            def _connect_topic(self, topic, uri):
                self.done = True
                return -1, "failed", 1

        handler = MockHandler()
        m = RegManager(handler)
        self.assertEquals(handler, m.handler)
        self.assert_(m.logger is not None)
        self.assertEquals(m.master_uri, None)
        self.assertEquals(m.uri, None)
        self.assertEquals([], m.updates)
        try:
            m.cond.acquire()
        finally:
            m.cond.release()

        # call twice with topic 2 to test filtering logic

        m.publisher_update('topic1', ['http://uri:1', 'http://uri:1b'])
        m.publisher_update('topic2', ['http://old:2', 'http://old:2b'])
        m.publisher_update('topic1b', ['http://foo:1', 'http://foo:1b'])
        m.publisher_update('topic2', ['http://uri:2', 'http://uri:2b'])
        self.assertEquals([('topic1', ['http://uri:1', 'http://uri:1b']),
                           ('topic2', ['http://old:2', 'http://old:2b']),
                           ('topic1b', ['http://foo:1', 'http://foo:1b']),
                           ('topic2', ['http://uri:2', 'http://uri:2b'])], m.updates)

        # disabling these tests as they are just too dangerous now that registrations multithreads updates
        if 0:
            # this should only go through once as MockHandler trips to done
            m.run()

            # have to give time for threads to spin up and call handler
            timeout_t = 10. + time.time()
            while time.time() < timeout_t and len(m.updates) > 2:
                time.sleep(0.1)

            m.handler = BadHandler()
            # this should only go through once as BadHandler trips to done. this tests the
            # exception branch
            m.run()

            # this will cause an error to be logged in _connect_topic_thread
            # - reset data in m.updates
            m.updates= [('topic1', ['http://uri:1', 'http://uri:1b']),
                        ('topic1b', ['http://foo:1', 'http://foo:1b'])]
            m.handler = BadHandler2()
            m.run()
        
        # m.start should return immediately as m.master_uri is not set
        self.assertEquals(m.master_uri, None)
        m.start(None, None)
        # test that it returns if URIs are equal
        m.start('http://localhost:1234', 'http://localhost:1234')

        # - test with is_shutdown overriden so we don't enter loop
        def foo():
            return True
        sys.modules['rospy.impl.registration'].__dict__['is_shutdown'] = foo
        m.start('http://localhost:1234', 'http://localhost:4567')
        
        handler.done = True
        # handler done is true, so this should not run
        m.run()
        # no master uri, so these should just return
        m.master_uri = None
        m.reg_added('n1', 'type1', 'rtype1')
        m.reg_removed('n1', 'type1', 'rtype1')
        m.cleanup('reason')

