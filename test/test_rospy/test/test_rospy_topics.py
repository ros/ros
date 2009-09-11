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
# Revision $Id: test_rospy_api.py 4898 2009-06-17 01:31:18Z sfkwc $

import roslib; roslib.load_manifest('test_rospy')

import os
import sys
import struct
import unittest
import time

import test_rospy.msg

def callback1(data): pass
def callback2(data): pass

# for use with publish() tests
import rospy.transport
class ConnectionOverride(rospy.transport.Transport):
    def __init__(self, endpoint_id):
        super(ConnectionOverride, self).__init__(rospy.transport.OUTBOUND, endpoint_id)
        self.endpoint_id = endpoint_id
        self.data = ''

    def set_cleanup_callback(self, cb): pass
    def write_data(self, data):
        self.data = self.data + data

# test rospy API verifies that the rospy module exports the required symbols
class TestRospyTopics(unittest.TestCase):

    def test_args_kwds_to_message(self):
        import rospy
        from rospy.topics import args_kwds_to_message
        from test_rospy.msg import Val
        
        v = Val('hello world-1')
        d = args_kwds_to_message(Val, (v,), None)
        self.assert_(d == v)
        d = args_kwds_to_message(Val, ('hello world-2',), None)
        self.assertEquals(d.val, 'hello world-2')
        d = args_kwds_to_message(Val, (), {'val':'hello world-3'})
        self.assertEquals(d.val, 'hello world-3')

        # error cases
        try:
            args_kwds_to_message(Val, 'hi', val='hello world-3')
            self.fail("should not allow args and kwds")
        except TypeError: pass
            
    def test_Publisher(self):
        import rospy
        from rospy.registration import get_topic_manager, Registration
        from rospy.topics import Publisher, DEFAULT_BUFF_SIZE
        # Publisher(self, name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None)

        name = 'foo'
        rname = rospy.resolve_name('foo')
        data_class = test_rospy.msg.Val

        # test invalid params
        for n in [None, '', 1]:
            try:
                Publisher(n, data_class)
                self.fail("should not allow invalid name")
            except ValueError: pass
        for d in [None, 1, TestRospyTopics]:
            try:
                Publisher(name, d)
                self.fail("should now allow invalid data_class")
            except ValueError: pass
        try:
            Publisher(name, None)
            self.fail("None should not be allowed for data_class")
        except ValueError: pass

        # round 1: test basic params
        pub = Publisher(name, data_class)
        self.assertEquals(rname, pub.name)
        self.assertEquals(data_class, pub.data_class)
        self.assertEquals('test_rospy/Val', pub.type)
        self.assertEquals(data_class._md5sum, pub.md5sum)
        self.assertEquals(Registration.PUB, pub.reg_type)
        
        # verify impl as well
        impl = get_topic_manager().get_impl(Registration.PUB, rname)
        self.assert_(impl == pub.impl)
        self.assertEquals(rname, impl.name)
        self.assertEquals(data_class, impl.data_class)                
        self.failIf(impl.is_latch)
        self.assertEquals(None, impl.latch)                
        self.assertEquals(0, impl.seq)
        self.assertEquals(1, impl.ref_count)
        self.assertEquals('', impl.buff.getvalue())
        self.failIf(impl.closed)
        self.failIf(impl.has_connections())
        # check publish() fall-through
        from test_rospy.msg import Val
        impl.publish(Val('hello world-1'))
        
        # check stats
        self.assertEquals(0, impl.message_data_sent)
        # check acquire/release don't bomb
        impl.acquire()
        impl.release()        

        # do a single publish with connection override. The connection
        # override is a major cheat as the object isn't even an actual
        # connection. I will need to add more integrated tests later
        co1 = ConnectionOverride('co1')
        self.failIf(impl.has_connection('co1'))
        impl.add_connection(co1)
        self.assert_(impl.has_connection('co1'))
        self.assert_(impl.has_connections())        
        impl.publish(Val('hello world-1'), connection_override=co1)

        import cStringIO
        buff = cStringIO.StringIO()
        Val('hello world-1').serialize(buff)
        # - check equals, but strip length field first
        self.assertEquals(co1.data[4:], buff.getvalue())
        self.assertEquals(None, impl.latch)
        
        # Now enable latch
        pub = Publisher(name, data_class, latch=True)
        impl = get_topic_manager().get_impl(Registration.PUB, rname)
        # have to verify latching in pub impl
        self.assert_(impl == pub.impl)
        self.assertEquals(True, impl.is_latch)
        self.assertEquals(None, impl.latch)
        self.assertEquals(2, impl.ref_count)        

        co2 = ConnectionOverride('co2')
        self.failIf(impl.has_connection('co2'))
        impl.add_connection(co2)
        for n in ['co1', 'co2']:
            self.assert_(impl.has_connection(n))
        self.assert_(impl.has_connections())        
        v = Val('hello world-2')
        impl.publish(v, connection_override=co2)
        self.assert_(v == impl.latch)

        buff = cStringIO.StringIO()
        Val('hello world-2').serialize(buff)
        # - strip length and check value
        self.assertEquals(co2.data[4:], buff.getvalue())

        # test that latched value is sent to connections on connect
        co3 = ConnectionOverride('co3')
        self.failIf(impl.has_connection('co3'))
        impl.add_connection(co3)
        for n in ['co1', 'co2', 'co3']:
            self.assert_(impl.has_connection(n))
        self.assert_(impl.has_connections())
        self.assertEquals(co3.data[4:], buff.getvalue())        
        
        # TODO: tcp_nodelay
        # TODO: suscribe listener
        self.assert_(impl.has_connection('co1'))
        impl.remove_connection(co1)
        self.failIf(impl.has_connection('co1'))
        self.assert_(impl.has_connections())
        # TODO: need to validate DeadTransport better
        self.assert_([d for d in impl.dead_connections if d.endpoint_id == 'co1'])
        
        self.assert_(impl.has_connection('co3'))
        impl.remove_connection(co3)        
        self.failIf(impl.has_connection('co3'))
        self.assert_(impl.has_connections())
        for id in ['co1', 'co3']:
            self.assert_([d for d in impl.dead_connections if d.endpoint_id == id])
        
        self.assert_(impl.has_connection('co2'))
        impl.remove_connection(co2)        
        self.failIf(impl.has_connection('co2'))
        self.failIf(impl.has_connections())
        for id in ['co1', 'co2', 'co3']:
            self.assert_([d for d in impl.dead_connections if d.endpoint_id == id])


        # test publish() latch on a new Publisher object (this was encountered in testing, so I want a test case for it)
        pub = Publisher('bar', data_class, latch=True)
        v = Val('no connection test')
        pub.impl.publish(v)
        self.assert_(v == pub.impl.latch)

        # test connection header
        h = {'foo': 'bar', 'fuga': 'hoge'}
        pub = Publisher('header_test', data_class, headers=h)
        self.assertEquals(h, pub.impl.headers)
        
    def test_Subscriber(self):
        #TODO: test callback args
        #TODO: negative buff_size
        #TODO: negative queue_size        
        import rospy
        from rospy.registration import get_topic_manager, Registration
        from rospy.topics import Subscriber, DEFAULT_BUFF_SIZE

        #Subscriber: name, data_class, callback=None, callback_args=None,
        #queue_size=None, buff_size=DEFAULT_BUFF_SIZE, tcp_nodelay=False):

        name = 'foo'
        rname = rospy.resolve_name('foo')
        data_class = test_rospy.msg.Val

        # test invalid params
        for n in [None, '', 1]:
            try:
                Subscriber(n, data_class)
                self.fail("should not allow invalid name")
            except ValueError: pass
        for d in [None, 1, TestRospyTopics]:
            try:
                Subscriber(name, d)
                self.fail("should now allow invalid data_class")
            except ValueError: pass
        try:
            Subscriber(name, None)
            self.fail("None should not be allowed for data_class")
        except ValueError: pass
        
        sub = Subscriber(name, data_class)
        self.assertEquals(rname, sub.name)
        self.assertEquals(data_class, sub.data_class)
        self.assertEquals('test_rospy/Val', sub.type)
        self.assertEquals(data_class._md5sum, sub.md5sum)
        self.assertEquals(Registration.SUB, sub.reg_type)
        
        # verify impl as well
        impl = get_topic_manager().get_impl(Registration.SUB, rname)
        self.assert_(impl == sub.impl)
        self.assertEquals([], impl.callbacks)
        self.assertEquals(rname, impl.name)
        self.assertEquals(data_class, impl.data_class)                
        self.assertEquals(None, impl.queue_size)
        self.assertEquals(DEFAULT_BUFF_SIZE, impl.buff_size)
        self.failIf(impl.tcp_nodelay)
        self.assertEquals(1, impl.ref_count)
        self.failIf(impl.closed)
        
        # round 2, now start setting options and make sure underlying impl is reconfigured
        name = 'foo'
        data_class = test_rospy.msg.Val
        queue_size = 1
        buff_size = 1
        sub = Subscriber(name, data_class, callback=callback1,
                         queue_size=queue_size, buff_size=buff_size, tcp_nodelay=True)
        self.assertEquals(rname, sub.name)
        self.assertEquals(data_class, sub.data_class)

        # verify impl 
        impl2 = get_topic_manager().get_impl(Registration.SUB, rname)
        self.assert_(impl == impl2) # should be same instance
        self.assertEquals([(callback1, None)], impl.callbacks)
        self.assertEquals(rname, impl.name)
        self.assertEquals(data_class, impl.data_class)                
        self.assertEquals(queue_size, impl.queue_size)
        self.assertEquals(buff_size, impl.buff_size)
        self.assert_(impl.tcp_nodelay)
        self.assertEquals(2, impl.ref_count)
        self.failIf(impl.closed)

        # round 3, make sure that options continue to reconfigure
        # underlying impl also test that tcp_nodelay is sticky. this
        # is technically undefined, but this is how rospy chose to
        # implement.
        name = 'foo'
        data_class = test_rospy.msg.Val
        queue_size = 2
        buff_size = 2
        sub = Subscriber(name, data_class, callback=callback2, 
                         queue_size=queue_size, buff_size=buff_size, tcp_nodelay=False)

        # verify impl 
        impl2 = get_topic_manager().get_impl(Registration.SUB, rname)
        self.assert_(impl == impl2) # should be same instance
        self.assertEquals(set([(callback1, None), (callback2, None)]), set(impl.callbacks))
        self.assertEquals(queue_size, impl.queue_size)
        self.assertEquals(buff_size, impl.buff_size)
        self.assert_(impl.tcp_nodelay)
        self.assertEquals(3, impl.ref_count)
        self.failIf(impl.closed)
        
if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_rospy', 'test_rospy_topic', TestRospyTopics, coverage_packages=['rospy.topics'])
