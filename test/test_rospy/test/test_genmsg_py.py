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
import math

from roslib.message import SerializationError
        
class TestGenmsgPy(unittest.TestCase):

    def test_PythonKeyword(self):
        from test_rospy.msg import PythonKeyword
        # the md5sum is pulled from the c++ message generator. The
        # test here is that the Python msg generator didn't
        # accidentally mutate a md5sum based on a message that has its
        # fieldname remapped.
        self.assertEquals(PythonKeyword._md5sum, "1330d6bbfad8e75334346fec949d5133")
                          
    ## Utility for testing roundtrip serialization
    ## @param orig Message to test roundtrip serialization of
    ## @param blank Uninitialized instance of message to deserialize into
    ## @param float bool: if True, use almostEquals instead of equals
    ## comparison. This variant assumes only data field is named
    ## 'data'
    def _test_ser_deser(self, orig, blank, float=False):
        b = StringIO()
        orig.serialize(b)
        blank.deserialize(b.getvalue())
        if not float:
            self.assertEquals(orig, blank, str(orig)+" != "+str(blank))
        else:
            self.assertAlmostEquals(orig.data, blank.data, 5)

    ## #2133/2152
    def test_test_rospy_TransitiveImport(self):
        from test_rospy.msg import TransitiveImport
        m = TransitiveImport()
        # invoking serialize should be enough to expose issue. The bug
        # was that genmsg_py was failing to include the imports of
        # embedded messages. Because messages are flattened, this
        # causes ImportErrors.
        self._test_ser_deser(m, TransitiveImport())        

    def test_test_rospy_TestFixedArray(self):
        from test_rospy.msg import TestFixedArray
        m = TestFixedArray()
        self.assertEquals([0.], m.f32_1)
        self.assertEquals([0., 0., 0.], m.f32_3)        
        self.assertEquals([0.], m.f64_1)
        self.assertEquals([0., 0., 0.], m.f64_3)        
        self.assertEquals([0], m.i8_1)
        self.assertEquals([0, 0, 0], m.i8_3)        
        self.assertEquals(chr(0), m.u8_1)
        self.assertEquals(chr(0)*3, m.u8_3)        
        self.assertEquals([0], m.i32_1)
        self.assertEquals([0, 0, 0], m.i32_3)        
        self.assertEquals([0], m.u32_1)
        self.assertEquals([0, 0, 0], m.u32_3)
        self.assertEquals([''], m.s_1)
        self.assertEquals(['', '', ''], m.s_3)

        self._test_ser_deser(m, TestFixedArray())

        m = TestFixedArray(i32_1 = [1])
        c = TestFixedArray()
        self._test_ser_deser(m, c)
        self.assertEquals((1,), c.i32_1)

        m = TestFixedArray(i32_3 = [-3, 2, 10])
        c = TestFixedArray()
        self._test_ser_deser(m, c)
        self.assertEquals((-3, 2, 10), c.i32_3)
        
        m = TestFixedArray(u32_1 = [1234])
        c = TestFixedArray()
        self._test_ser_deser(m, c)
        self.assertEquals((1234,), c.u32_1)
        
        m = TestFixedArray(u32_3 = [3, 2, 10])
        c = TestFixedArray()
        self._test_ser_deser(m, c)
        self.assertEquals((3, 2, 10), c.u32_3)
        
        # this could potentially fail due to floating point lossiness
        m,c = TestFixedArray(f32_1 = [2.]), TestFixedArray()
        self._test_ser_deser(m, c)
        self.assertEquals((2.,), c.f32_1)
        
        m,c = TestFixedArray(f32_3 = [1., 2., 3.]), TestFixedArray()
        self._test_ser_deser(m, c)
        self.assertEquals((1., 2., 3.), c.f32_3)
        
        m,c = TestFixedArray(u8_1 = 'x'), TestFixedArray()
        self._test_ser_deser(m, c)
        self.assertEquals('x', c.u8_1)

        m,c = TestFixedArray(u8_3 = 'xyz'), TestFixedArray()
        self._test_ser_deser(m, c)
        self.assertEquals('xyz', c.u8_3)

        m,c = TestFixedArray(s_1 = ['']), TestFixedArray()
        self._test_ser_deser(m, c)
        self.assertEquals([''], c.s_1)

        m,c = TestFixedArray(s_1 = ['blah blah blah']), TestFixedArray()
        self._test_ser_deser(m, c)
        self.assertEquals(['blah blah blah',], c.s_1)

        m = TestFixedArray(s_3 = ['', 'x', 'xyz'])
        c = TestFixedArray()
        self._test_ser_deser(m, c)
        self.assertEquals(['', 'x', 'xyz'], c.s_3)

        for v in [True, False]:
            m = TestFixedArray(b_1 = [v])
            c = TestFixedArray()
            self._test_ser_deser(m, c)
            self.assertEquals([v], c.b_1)

        m = TestFixedArray(b_3 = [True, False, True])
        c = TestFixedArray()
        self._test_ser_deser(m, c)
        self.assertEquals([True, False, True], c.b_3)
        
        #TODO: enable tests for auto-convert of uint8[] to string
        
    def test_test_rospy_TestConstants(self):
        from test_rospy.msg import TestConstants
        self.assertEquals(-123.0, TestConstants.A)
        self.assertEquals(124.0, TestConstants.B)
        self.assertEquals(125.0, TestConstants.C)
        self.assertEquals(123, TestConstants.X)
        self.assertEquals(-123, TestConstants.Y)
        self.assertEquals(124, TestConstants.Z)
        self.assertEquals("'hi", TestConstants.SINGLEQUOTE)
        self.assertEquals('"hello" there', TestConstants.DOUBLEQUOTE)
        self.assertEquals('"hello" \'goodbye\'', TestConstants.MULTIQUOTE)
        self.assertEquals('foo', TestConstants.FOO) 
        self.assertEquals('"#comments" are ignored, and leading and trailing whitespace removed',TestConstants.EXAMPLE)
        self.assertEquals('strip', TestConstants.WHITESPACE)
        self.assertEquals('', TestConstants.EMPTY)

        self.assertEquals(True, TestConstants.TRUE)
        self.assertEquals(False, TestConstants.FALSE)        
        
    def test_std_msgs_empty(self):
        from std_msgs.msg import Empty
        self.assertEquals(Empty(), Empty())
        self._test_ser_deser(Empty(), Empty())

    def test_std_msgs_Bool(self):
        from std_msgs.msg import Bool
        self.assertEquals(Bool(), Bool())
        self._test_ser_deser(Bool(), Bool())
        # default value should be False
        self.assertEquals(False, Bool().data)
        # test various constructor permutations
        for v in [True, False]:
            self.assertEquals(Bool(v), Bool(v))
            self.assertEquals(Bool(v), Bool(data=v))
            self.assertEquals(Bool(data=v), Bool(data=v))
        self.assertNotEquals(Bool(True), Bool(False))            

        self._test_ser_deser(Bool(True), Bool())
        self._test_ser_deser(Bool(False), Bool())

        # validate type cast to bool
        blank = Bool()
        b = StringIO()
        Bool(True).serialize(b)
        blank.deserialize(b.getvalue())
        self.assert_(blank.data)
        self.assert_(type(blank.data) == bool)        

        b = StringIO()
        Bool(True).serialize(b)
        blank.deserialize(b.getvalue())
        self.assert_(blank.data)
        self.assert_(type(blank.data) == bool)
        
        
    def test_std_msgs_String(self):
        from std_msgs.msg import String
        self.assertEquals(String(), String())
        self.assertEquals('', String().data)
        # default value should be empty string
        self.assertEquals(String(''), String())
        self.assertEquals(String(''), String(''))
        self.assertEquals(String('foo'), String('foo'))
        self.assertEquals(String('foo'), String(data='foo'))
        self.assertEquals(String(data='foo'), String(data='foo'))
        
        self.assertNotEquals(String('foo'), String('bar'))
        self.assertNotEquals(String('foo'), String(data='bar'))
        self.assertNotEquals(String(data='foo'), String(data='bar'))
        
        self._test_ser_deser(String(''), String())
        self._test_ser_deser(String('a man a plan a canal panama'), String())

    def test_std_msgs_SignedInt(self):
        from std_msgs.msg import Int8, Int16, Int32, Int64
        for cls in [Int8, Int16, Int32, Int64]:
            v = random.randint(1, 127)
            self.assertEquals(cls(), cls())
            self.assertEquals(0, cls().data)
            self.assertEquals(cls(), cls(0))
            self.assertEquals(cls(0), cls(0))        
            self.assertEquals(cls(v), cls(v))
            self.assertEquals(cls(-v), cls(-v))
            self.assertEquals(cls(v), cls(data=v))        
            self.assertEquals(cls(data=v), cls(data=v))
        
            self.assertNotEquals(cls(v), cls())
            self.assertNotEquals(cls(data=v), cls(data=-v))
            self.assertNotEquals(cls(data=v), cls(data=v-1))            
            self.assertNotEquals(cls(data=v), cls(v-1))
            self.assertNotEquals(cls(v), cls(v-1))
            
            self._test_ser_deser(cls(), cls())
            self._test_ser_deser(cls(0), cls())
            self._test_ser_deser(cls(-v), cls())
            self._test_ser_deser(cls(v), cls())

        # rospy currently does not spot negative overflow due to the fact that Python's struct doesn't either
        widths = [(8, Int8), (16, Int16), (32, Int32), (64, Int64)]
        for w, cls in widths:
            maxp = long(math.pow(2, w-1)) - 1
            maxn = -long(math.pow(2, w-1)) + 1
            self._test_ser_deser(cls(maxp), cls())
            self._test_ser_deser(cls(maxn), cls())
            try:
                cls(maxp+1)._check_types()
                self.fail("check_types should have noted width error[%s]: %s, %s"%(w, maxp+1, cls.__name__))
            except SerializationError: pass
            try:
                cls(maxn-1)._check_types()
                self.fail("check_types should have noted width error[%s]: %s, %s"%(w, maxn-1, cls.__name__))
            except SerializationError: pass
            
    def test_std_msgs_UnsignedInt(self):
        from std_msgs.msg import UInt8, UInt16, UInt32, UInt64
        for cls in [UInt8, UInt16, UInt32, UInt64]:
            v = random.randint(1, 127)
            self.assertEquals(cls(), cls())
            self.assertEquals(0, cls().data)
            self.assertEquals(cls(), cls(0))
            self.assertEquals(cls(0), cls(0))        
            self.assertEquals(cls(v), cls(v))
            self.assertEquals(cls(v), cls(data=v))        
            self.assertEquals(cls(data=v), cls(data=v))
        
            self.assertNotEquals(cls(v), cls())
            self.assertNotEquals(cls(data=v), cls(data=-v))
            self.assertNotEquals(cls(data=v), cls(data=v-1))            
            self.assertNotEquals(cls(data=v), cls(v-1))
            self.assertNotEquals(cls(v), cls(v-1))
            
            self._test_ser_deser(cls(), cls())
            self._test_ser_deser(cls(0), cls())
            self._test_ser_deser(cls(v), cls())

            try:
                cls(-1)._check_types()
                self.fail("check_types should have noted sign error[%s]: %s"%(w, cls.__name__))
            except SerializationError: pass

        # rospy currently does not spot negative overflow due to the fact that Python's struct doesn't either
        widths = [(8, UInt8), (16, UInt16), (32, UInt32), (64, UInt64)]
        for w, cls in widths:
            maxp = long(math.pow(2, w)) - 1
            self._test_ser_deser(cls(maxp), cls())
            try:
                cls(maxp+1)._check_types()
                self.fail("check_types should have noted width error[%s]: %s, %s"%(w, maxp+1, cls.__name__))
            except SerializationError: pass
            
    def test_std_msgs_Float(self):
        from std_msgs.msg import Float32, Float64
        for cls in [Float32, Float64]:
            self.assertEquals(cls(), cls())
            self.assertEquals(0., cls().data)
            self.assertEquals(cls(), cls(0.))
            self.assertEquals(cls(0.), cls(0.))        
            self.assertEquals(cls(1.), cls(1.))
            self.assertEquals(cls(1.), cls(data=1.))        
            self.assertEquals(cls(data=1.), cls(data=1.))
            self.assertEquals(cls(math.pi), cls(math.pi))
            self.assertEquals(cls(math.pi), cls(data=math.pi))        
            self.assertEquals(cls(data=math.pi), cls(data=math.pi))
        
            self.assertNotEquals(cls(1.), cls())
            self.assertNotEquals(cls(math.pi), cls())
            self.assertNotEquals(cls(data=math.pi), cls(data=-math.pi))
            self.assertNotEquals(cls(data=math.pi), cls(data=math.pi-1))            
            self.assertNotEquals(cls(data=math.pi), cls(math.pi-1))
            self.assertNotEquals(cls(math.pi), cls(math.pi-1))
            
            self._test_ser_deser(cls(), cls())
            self._test_ser_deser(cls(0.), cls())
            self._test_ser_deser(cls(1.), cls(), float=True)
            self._test_ser_deser(cls(math.pi), cls(), float=True)

    def test_std_msgs_MultiArray(self):
        # multiarray is good test of embed plus array type
        from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout, UInt8MultiArray
        
        dims = [MultiArrayDimension('foo', 1, 2), MultiArrayDimension('bar', 3, 4),\
                    MultiArrayDimension('foo2', 5, 6), MultiArrayDimension('bar2', 7, 8)]
        for d in dims:
            self.assertEquals(d, d)

        # there was a bug with UInt8 arrays, so this is a regression
        # test. the buff was with the uint8[] type consistency
        buff = StringIO()
        self.assertEquals(UInt8MultiArray(),UInt8MultiArray())
        self.assertEquals('',UInt8MultiArray().data)        
        UInt8MultiArray().serialize(buff)
        self.assertEquals(UInt8MultiArray(layout=MultiArrayLayout()),UInt8MultiArray())
        UInt8MultiArray(layout=MultiArrayLayout()).serialize(buff)
        data = ''.join([chr(i) for i in range(0, 100)])
        v = UInt8MultiArray(data=data)
        self._test_ser_deser(UInt8MultiArray(data=data),UInt8MultiArray())
        
        self.assertEquals(Int32MultiArray(),Int32MultiArray())
        self.assertEquals(Int32MultiArray(layout=MultiArrayLayout()),Int32MultiArray())
        self.assertEquals(Int32MultiArray(layout=MultiArrayLayout(), data=[1, 2, 3]),Int32MultiArray(data=[1, 2, 3]))
        self.assertEquals(Int32MultiArray(layout=MultiArrayLayout(), data=[1, 2, 3]),\
                              Int32MultiArray(layout=MultiArrayLayout(),data=[1, 2, 3]))
        self.assertEquals(Int32MultiArray(layout=MultiArrayLayout(dim=[]), data=[1, 2, 3]),\
                              Int32MultiArray(layout=MultiArrayLayout(),data=[1, 2, 3]))
        self.assertEquals(Int32MultiArray(layout=MultiArrayLayout([], 0), data=[1, 2, 3]),\
                              Int32MultiArray(layout=MultiArrayLayout(),data=[1, 2, 3]))
        self.assertEquals(Int32MultiArray(layout=MultiArrayLayout(dim=[], data_offset=0), data=[1, 2, 3]),\
                              Int32MultiArray(layout=MultiArrayLayout(),data=[1, 2, 3]))
        self.assertEquals(Int32MultiArray(layout=MultiArrayLayout(dim=dims, data_offset=0), data=[1, 2, 3]),\
                              Int32MultiArray(layout=MultiArrayLayout(dim=dims),data=[1, 2, 3]))
        self.assertEquals(Int32MultiArray(layout=MultiArrayLayout(dims, 10), data=[1, 2, 3]),\
                              Int32MultiArray(layout=MultiArrayLayout(dim=dims,data_offset=10),data=[1, 2, 3]))


        self.assertNotEquals(Int32MultiArray(data=[1, 2, 3]),Int32MultiArray(data=[4,5,6]))
        self.assertNotEquals(Int32MultiArray(layout=MultiArrayLayout([], 1), data=[1, 2, 3]),\
                                 Int32MultiArray(layout=MultiArrayLayout([], 0),data=[1, 2, 3]))
        self.assertNotEquals(Int32MultiArray(layout=MultiArrayLayout([], 1), data=[1, 2, 3]),\
                                 Int32MultiArray(layout=MultiArrayLayout(dim=[]),data=[1, 2, 3]))
        self.assertNotEquals(Int32MultiArray(layout=MultiArrayLayout(dims, 10), data=[1, 2, 3]),\
                                 Int32MultiArray(layout=MultiArrayLayout(dim=dims,data_offset=11),data=[1, 2, 3]))
        self.assertNotEquals(Int32MultiArray(layout=MultiArrayLayout(dim=dims, data_offset=10), data=[1, 2, 3]),\
                                 Int32MultiArray(layout=MultiArrayLayout(dim=dims[1:],data_offset=10),data=[1, 2, 3]))
        

        self._test_ser_deser(Int32MultiArray(),Int32MultiArray())
        self._test_ser_deser(Int32MultiArray(layout=MultiArrayLayout()),Int32MultiArray())
        self._test_ser_deser(Int32MultiArray(data=[1, 2, 3]),Int32MultiArray())
