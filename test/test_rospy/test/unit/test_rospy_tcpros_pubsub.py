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

import os
import sys
import socket
import struct
import unittest
import time

class FakeSocket(object):
    def __init__(self):
        self.data = ''
        self.sockopt = None
    def fileno(self):
        # fool select logic by giving it stdout fileno
        return 1
    def setblocking(self, *args):
        pass
    def setsockopt(self, *args):
        self.sockopt = args
    def send(self, d):
        self.data = self.data+d
        return len(d)
    def sendall(self, d):
        self.data = self.data+d
    def close(self):
        pass

# test rospy API verifies that the rospy module exports the required symbols
class TestRospyTcprosPubsub(unittest.TestCase):

    def test_TCPROSSub(self):
        import rospy.impl.transport
        from rospy.impl.tcpros_pubsub import TCPROSSub        
        import test_rospy.msg

        callerid = 'test_TCPROSSub'
        import rospy.names
        rospy.names._set_caller_id(callerid)

        #name, recv_data_class, queue_size=None, buff_size=DEFAULT_BUFF_SIZE
        name = 'name-%s'%time.time()
        recv_data_class = test_rospy.msg.Val
        s = TCPROSSub(name, recv_data_class)
        self.assertEquals(name, s.resolved_name)
        self.assertEquals(rospy.impl.transport.INBOUND, s.direction)
        self.assertEquals(recv_data_class, s.recv_data_class)
        self.assert_(s.buff_size > -1)
        self.failIf(s.tcp_nodelay)
        self.assertEquals(None, s.queue_size)

        fields = s.get_header_fields()
        self.assertEquals(name, fields['topic'])
        self.assertEquals(recv_data_class._md5sum, fields['md5sum'])
        self.assertEquals(recv_data_class._full_text, fields['message_definition'])
        self.assertEquals('test_rospy/Val', fields['type'])
        self.assert_(callerid, fields['callerid'])
        if 'tcp_nodelay' in fields:
            self.assertEquals('0', fields['tcp_nodelay'])
        
        v = int(time.time())
        s = TCPROSSub(name, recv_data_class, queue_size=v)
        self.assertEquals(v, s.queue_size)        

        s = TCPROSSub(name, recv_data_class, buff_size=v)
        self.assertEquals(v, s.buff_size)

        s = TCPROSSub(name, recv_data_class, tcp_nodelay=True)
        self.assert_(s.tcp_nodelay)
        self.assertEquals('1', s.get_header_fields()['tcp_nodelay'])
        
    def test_TCPROSPub(self):
        import rospy.impl.transport
        from rospy.impl.tcpros_pubsub import TCPROSPub        
        import test_rospy.msg

        callerid = 'test_TCPROSPub'
        import rospy.names
        rospy.names._set_caller_id(callerid)

        #name, pub_data_class
        name = 'name-%s'%time.time()
        pub_data_class = test_rospy.msg.Val
        p = TCPROSPub(name, pub_data_class)
        self.assertEquals(name, p.resolved_name)
        self.assertEquals(rospy.impl.transport.OUTBOUND, p.direction)
        self.assertEquals(pub_data_class, p.pub_data_class)
        self.assert_(p.buff_size > -1)
        self.failIf(p.is_latch)

        fields = p.get_header_fields()
        self.assertEquals(name, fields['topic'])
        self.assertEquals(pub_data_class._md5sum, fields['md5sum'])
        self.assertEquals(pub_data_class._full_text, fields['message_definition'])
        self.assertEquals('test_rospy/Val', fields['type'])
        self.assert_(callerid, fields['callerid'])
        if 'latching' in fields:
            self.assertEquals('0', fields['latching'])

        p = TCPROSPub(name, pub_data_class, is_latch=True)
        self.assert_(p.is_latch)
        self.assertEquals('1', p.get_header_fields()['latching'])

        # test additional header fields
        p = TCPROSPub(name, pub_data_class, headers={'foo': 'bar', 'hoge': 'fuga'})
        fields = p.get_header_fields()        
        self.assertEquals(name, fields['topic'])
        self.assertEquals('fuga', fields['hoge'])
        self.assertEquals('bar', fields['foo'])        
        
    def test_configure_pub_socket(self):
        # #1241 regression test to make sure that imports don't get messed up again
        from rospy.impl.tcpros_pubsub import _configure_pub_socket
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        _configure_pub_socket(sock, True)
        sock.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)        
        _configure_pub_socket(sock, False)
        sock.close()

    def test_TCPROSHandler_topic_connection_handler(self):
        
        import rospy
        import rospy.core
        # very ugly hack to handle bad design choice in rospy and bad isolation inside of nose
        rospy.core._shutdown_flag  = False        
        rospy.core._in_shutdown  = False        
        from rospy.impl.registration import Registration
        from rospy.impl.tcpros_pubsub import TCPROSHandler
        import test_rospy.msg

        handler = TCPROSHandler()
        tch = handler.topic_connection_handler 
        sock = FakeSocket()
        client_addr = '127.0.0.1'
        data_class = test_rospy.msg.Val
        topic_name = '/foo-tch'
        
        headers = { 'topic': topic_name, 'md5sum': data_class._md5sum, 'callerid': '/node'}
        # test required logic
        for k in headers.iterkeys():
            header_copy = headers.copy()
            del header_copy[k]
            err = tch(sock, client_addr, header_copy)
            self.assertNotEquals('', err)

        # '/foo-tch' is not registered, so this should error
        err = tch(sock, client_addr, headers)
        self.assert_(err)

        # register '/foo-tch'
        tm = rospy.impl.registration.get_topic_manager()
        impl = tm.acquire_impl(Registration.PUB, topic_name, data_class)
        self.assert_(impl is not None)

        # test with mismatched md5sum
        header_copy = headers.copy()
        header_copy['md5sum'] = 'bad'
        md5_err = tch(sock, client_addr, header_copy)
        self.assert_("md5sum" in md5_err, md5_err) 

        # now test with correct params
        err = tch(sock, client_addr, headers)        
        self.failIf(err)
        self.assertEquals(None, sock.sockopt)

        # test with mismatched type
        # - if md5sums match, this should not error
        header_copy = headers.copy()
        header_copy['type'] = 'bad_type/Bad'
        err = tch(sock, client_addr, header_copy)        
        self.failIf(err)
        # - now give mismatched md5sum, this will test different error message
        header_copy['md5sum'] = 'bad'
        type_md5_err = tch(sock, client_addr, header_copy)
        self.assert_("types" in type_md5_err, type_md5_err)
        
        # - these error messages should be different
        self.assertNotEquals(md5_err, type_md5_err)
        
        # test tcp_nodelay
        # - should be equivalent to above
        headers['tcp_nodelay'] = '0'
        err = tch(sock, client_addr, headers)        
        self.failIf(err)
        self.assertEquals(None, sock.sockopt)        
        
        # - now test actual sock opt
        headers['tcp_nodelay'] = '1'        
        err = tch(sock, client_addr, headers)        
        self.failIf(err)
        self.assertEquals((socket.IPPROTO_TCP, socket.TCP_NODELAY, 1), sock.sockopt)
        # test connection headers
        impl.headers = {'foo': 'baz', 'hoge': 'fuga'}        
        headers['tcp_nodelay'] = '0'
        err = tch(sock, client_addr, headers)        
        self.failIf(err)
        connection = impl.connections[-1]
        fields = connection.protocol.get_header_fields()
        self.assertEquals(impl.resolved_name, fields['topic'])
        self.assertEquals('fuga', fields['hoge'])
        self.assertEquals('baz', fields['foo'])        
            
