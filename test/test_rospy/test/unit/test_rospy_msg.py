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
from cStringIO import StringIO
import time
import random

import genpy

class TestRospyMsg(unittest.TestCase):

    def test_args_kwds_to_message(self):
        import rospy
        from rospy.msg import args_kwds_to_message
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

    def test_serialize_message(self):
        import rospy.msg
        import rospy.rostime
        # have to fake-init rostime so that Header can be stamped
        rospy.rostime.set_rostime_initialized(True)
        
        buff = StringIO()
        seq = random.randint(1, 1000)
        
        #serialize_message(seq, msg)
        from test_rospy.msg import Val

        #serialize a simple 'Val' with a string in it
        teststr = 'foostr-%s'%time.time()
        val = Val(teststr)

        fmt = "<II%ss"%len(teststr)
        size = struct.calcsize(fmt) - 4
        valid = struct.pack(fmt, size, len(teststr), teststr)
        
        rospy.msg.serialize_message(buff, seq, val)

        self.assertEquals(valid, buff.getvalue())
        
        #test repeated serialization
        rospy.msg.serialize_message(buff, seq, val)
        rospy.msg.serialize_message(buff, seq, val)        
        self.assertEquals(valid*3, buff.getvalue())

        # - once more just to make sure that the buffer position is
        # being preserved properly
        buff.seek(0)
        rospy.msg.serialize_message(buff, seq, val)
        self.assertEquals(valid*3, buff.getvalue())        
        rospy.msg.serialize_message(buff, seq, val)
        self.assertEquals(valid*3, buff.getvalue()) 
        rospy.msg.serialize_message(buff, seq, val)        
        self.assertEquals(valid*3, buff.getvalue())        

        #test sequence parameter
        buff.truncate(0)

        from test_rospy.msg import HeaderVal
        t = rospy.Time.now()
        t.secs = t.secs - 1 # move it back in time
        h = rospy.Header(None, rospy.Time.now(), teststr)
        h.stamp = t
        val = HeaderVal(h, teststr)
        seq += 1
        
        rospy.msg.serialize_message(buff, seq, val)
        self.assertEquals(val.header, h)
        self.assertEquals(seq, h.seq)
        #should not have been changed
        self.assertEquals(t, h.stamp) 
        self.assertEquals(teststr, h.frame_id) 
        
        #test frame_id setting
        h.frame_id = None
        rospy.msg.serialize_message(buff, seq, val)
        self.assertEquals(val.header, h)
        self.assertEquals('0', h.frame_id) 
        

    def test_deserialize_messages(self):
        import rospy.msg
        from test_rospy.msg import Val
        num_tests = 10
        teststrs = ['foostr-%s'%random.randint(0, 10000) for i in xrange(0, num_tests)]
        valids = []
        for t in teststrs:
            fmt = "<II%ss"%len(t)
            size = struct.calcsize(fmt) - 4
            valids.append(struct.pack(fmt, size, len(t), t))
        data_class = Val

        def validate_vals(vals, teststrs=teststrs):
            for i, v in zip(range(0, len(vals)), vals):
                self.assert_(isinstance(v, Val))
                self.assertEquals(teststrs[i], v.val)        
        
        b = StringIO()
        msg_queue = []
        
        #test with null buff
        try:
            rospy.msg.deserialize_messages(None, msg_queue, data_class)
        except genpy.DeserializationError: pass
        #test will null msg_queue
        try:
            rospy.msg.deserialize_messages(b, None, data_class)
        except genpy.DeserializationError: pass
        #test with empty buff
        rospy.msg.deserialize_messages(b, msg_queue, data_class)
        self.assertEquals(0, len(msg_queue))
        self.assertEquals(0, b.tell())

        #deserialize a simple value
        b.truncate(0)
        b.write(valids[0])
        rospy.msg.deserialize_messages(b, msg_queue, data_class)
        self.assertEquals(1, len(msg_queue))
        validate_vals(msg_queue)
        # - buffer should be reset
        self.assertEquals(0, b.tell())         
        del msg_queue[:]

        #verify deserialize does not read past b.tell()
        b.truncate(0)
        b.write(valids[0])
        b.write(valids[1])        
        b.seek(len(valids[0]))
        rospy.msg.deserialize_messages(b, msg_queue, data_class)
        self.assertEquals(1, len(msg_queue))
        validate_vals(msg_queue)
        # - buffer should be reset
        self.assertEquals(0, b.tell())        

        del msg_queue[:]

        #deserialize an incomplete message
        b.truncate(0)
        b.write(valids[0][:-1])        
        rospy.msg.deserialize_messages(b, msg_queue, data_class)        
        self.failIf(msg_queue, "deserialize of an incomplete buffer returned %s"%msg_queue)
        
        del msg_queue[:]
        
        #deserialize with extra data leftover
        b.truncate(0)
        b.write(valids[0]+'leftovers')
        rospy.msg.deserialize_messages(b, msg_queue, data_class)
        self.assertEquals(1, len(msg_queue))
        validate_vals(msg_queue)
        # - leftovers should be pushed to the front of the buffer
        self.assertEquals('leftovers', b.getvalue())
        
        del msg_queue[:]

        #deserialize multiple values
        b.truncate(0)
        for v in valids:
            b.write(v)
        rospy.msg.deserialize_messages(b, msg_queue, data_class)
        self.assertEquals(len(valids), len(msg_queue))
        validate_vals(msg_queue)
        # - buffer should be reset
        self.assertEquals(0, b.tell())        
        
        del msg_queue[:]

        #deserialize multiple values with max_msgs
        max_msgs = 5
        b.truncate(0)
        for v in valids:
            b.write(v)
        rospy.msg.deserialize_messages(b, msg_queue, data_class, max_msgs=max_msgs)
        self.assertEquals(max_msgs, len(msg_queue))
        validate_vals(msg_queue)
        # - buffer should be have remaining msgs
        b2 = StringIO()
        for v in valids[max_msgs:]:
            b2.write(v)
        self.assertEquals(b.getvalue(), b2.getvalue())

        #deserialize rest and verify that msg_queue is appended to
        rospy.msg.deserialize_messages(b, msg_queue, data_class)
        self.assertEquals(len(valids), len(msg_queue))
        validate_vals(msg_queue)
        
        del msg_queue[:]
        
        #deserialize multiple values with queue_size
        queue_size = 5
        b.truncate(0)
        for v in valids:
            b.write(v)
        # fill queue with junk
        msg_queue = [1, 2, 3, 4, 5, 6, 7, 9, 10, 11]
        rospy.msg.deserialize_messages(b, msg_queue, data_class, queue_size=queue_size)
        self.assertEquals(queue_size, len(msg_queue))
        # - msg_queue should have the most recent values only
        validate_vals(msg_queue, teststrs[-queue_size:])
        # - buffer should be reset
        self.assertEquals(0, b.tell())        
        
        #deserialize multiple values with max_msgs and queue_size
        queue_size = 5
        max_msgs = 5
        b.truncate(0)
        for v in valids:
            b.write(v)
        # fill queue with junk
        msg_queue = [1, 2, 3, 4, 5, 6, 7, 9, 10, 11]
        rospy.msg.deserialize_messages(b, msg_queue, data_class, max_msgs=max_msgs, queue_size=queue_size)
        self.assertEquals(queue_size, len(msg_queue))
        # - msg_queue should have the oldest messages 
        validate_vals(msg_queue)
        # - buffer should be have remaining msgs
        b2 = StringIO()
        for v in valids[max_msgs:]:
            b2.write(v)
        self.assertEquals(b.getvalue(), b2.getvalue())

