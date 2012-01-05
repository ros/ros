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
import struct
import unittest
import time

class TestRospyTransport(unittest.TestCase):

    def test_Transport(self):
        from rospy.impl.transport import Transport, INBOUND, OUTBOUND, BIDIRECTIONAL
        ids = []
        for d in [INBOUND, OUTBOUND, BIDIRECTIONAL]:
            t = Transport(d)
            self.assertEquals(d, t.direction)
            self.assertEquals("UNKNOWN", t.transport_type)            
            self.failIf(t.done)
            self.assertEquals(None, t.cleanup_cb)
            self.assertEquals('', t.endpoint_id)            
            self.assertEquals('unnamed', t.name)
            self.assertEquals(0, t.stat_bytes)            
            self.assertEquals(0, t.stat_num_msg)            
            self.failIf(t.id in ids)
            ids.append(t.id)

            t = Transport(d, 'a name')
            self.assertEquals(d, t.direction)
            self.assertEquals('a name', t.name)

        # test cleanup with and without a callback
        t = Transport(INBOUND)
        t.close()
        self.assert_(t.done)
        t = Transport(INBOUND)

        self.cleanup_obj = None
        def cleanup(obj):
            self.cleanup_obj = obj

        t.set_cleanup_callback(cleanup)
        self.assertEquals(t.cleanup_cb, cleanup)
        t.close()
        self.assert_(t.done)
        self.assertEquals(self.cleanup_obj, t)

        t = Transport(OUTBOUND)
        import traceback
        try:
            t.send_message('msg', 1)
            self.fail("send_message() should be abstract")
        except:
            traceback.print_exc()
        try:
            t.write_data('data')
            self.fail("write_data() should be abstract")
        except:
            traceback.print_exc()            
        t = Transport(INBOUND)
        try:
            t.receive_once()
            self.fail("receive_once() should be abstract")
        except: pass
        t = Transport(INBOUND)
        try:
            t.receive_loop()
            self.fail("receive_loop() should be abstract")
        except: pass
        
    def test_DeadTransport(self):
        from rospy.impl.transport import Transport, DeadTransport, INBOUND, OUTBOUND, BIDIRECTIONAL
        t = Transport(INBOUND, 'foo')
        t.stat_bytes = 1234
        t.stat_num_msg = 5678
        dead = DeadTransport(t)
        self.assertEquals(INBOUND, dead.direction)
        self.assertEquals('foo', dead.name)
        self.assertEquals(1234, dead.stat_bytes)
        self.assertEquals(5678, dead.stat_num_msg)        
        self.assertEquals(True, dead.done)
        self.assertEquals('', dead.endpoint_id)

        t = Transport(OUTBOUND, 'bar')
        t.endpoint_id = 'blah blah'
        t.close()
        dead = DeadTransport(t)
        self.assertEquals(OUTBOUND, dead.direction)
        self.assertEquals('bar', dead.name)
        self.assertEquals(True, dead.done)
        self.assertEquals(t.endpoint_id, dead.endpoint_id)
        
    def test_ProtocolHandler(self):
        # tripwire tests
        from rospy.impl.transport import ProtocolHandler
        h = ProtocolHandler()
        self.failIf(h.supports('TCPROS'))
        self.assertEquals([], h.get_supported())
        try:
            h.create_connection("/topic", 'http://localhost:1234', ['TCPROS'])
            self.fail("create_connection should raise an exception")
        except: pass
        try:
            h.init_publisher("/topic", 'TCPROS')
            self.fail("init_publisher raise an exception")
        except: pass
        h.shutdown()
