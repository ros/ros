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
import unittest
import cStringIO
import time
import yaml
        
from contextlib import contextmanager

class TestRostopicUnit(unittest.TestCase):

    def test_sub_str_plot_fields(self):
        from rostopic import _str_plot_fields
        from std_msgs.msg import String, Int32, Header
        from test_ros.msg import Simple, TVals, Floats, Arrays, Embed

        from genpy import Time, Duration
        from rostopic import create_field_filter

        # str plotting requires rospy time, we fix time to a set time
        import rospy.rostime
        rospy.rostime.set_rostime_initialized(True)
        rospy.rostime._set_rostime(Time(0, 1234))
        r_time = Time(0, 5678)

        # prepare test values
        simple_v = Simple(1, -2, 3, -4, 'a', 7, 8, 9, 'bar')
        simple_d = 'time,field.b,field.int16,field.int32,field.int64,field.c,field.uint16,field.uint32,field.uint64,field.str'
        simple_nostr = 'time,field.b,field.int16,field.int32,field.int64,field.c,field.uint16,field.uint32,field.uint64'

        arrays_v = Arrays([-1], chr(2)+chr(3), [3, 4, 5], [6, 7, 8], ['a1', 'b2', 'b3'], [Time(123, 456), Time(78, 90)])
        arrays_d = 'time,field.int8_arr0,field.uint8_arr0,field.uint8_arr1,field.int32_arr0,field.int32_arr1,field.int32_arr2,field.uint32_arr0,field.uint32_arr1,field.uint32_arr2,field.string_arr0,field.string_arr1,field.string_arr2,field.time_arr0,field.time_arr1'
        arrays_nostr = 'time,field.int8_arr0,field.uint8_arr0,field.uint8_arr1,field.int32_arr0,field.int32_arr1,field.int32_arr2,field.uint32_arr0,field.uint32_arr1,field.uint32_arr2,field.time_arr0,field.time_arr1'

        embed_v = Embed(simple_v, arrays_v)
        embed_d = simple_d.replace('field.', 'field.simple.')+','+arrays_d.replace('field.', 'field.arrays.')[5:]
        embed_nostr = simple_nostr.replace('field.', 'field.simple.')+','+arrays_nostr.replace('field.', 'field.arrays.')[5:]
        embed_noarr = simple_d.replace('field.', 'field.simple.')
        embed_nostr_noarr = simple_nostr.replace('field.', 'field.simple.')
        
        # test over all combinations of field filters
        f = create_field_filter(echo_nostr=False, echo_noarr=False)
        m = String()
        self.assertEquals("time,field.data", _str_plot_fields(m, 'field',f))
        m = String('foo')
        self.assertEquals('time,field.data', _str_plot_fields(m, 'field',f))
        m = TVals(Time(123, 456), Duration(78, 90))
        v = _str_plot_fields(m, 'field', f)
        self.assertEquals('time,field.t,field.d', v)
        m = simple_v
        self.assertEquals(simple_d, _str_plot_fields(m, 'field', f))
        m = arrays_v
        self.assertEquals(arrays_d, _str_plot_fields(m, 'field', f))
        m = embed_v
        self.assertEquals(embed_d, _str_plot_fields(m, 'field', f))

        f = create_field_filter(echo_nostr=True, echo_noarr=False)
        m = String()
        self.assertEquals("time,", _str_plot_fields(m, 'field',f))
        m = String('foo')
        self.assertEquals('time,', _str_plot_fields(m, 'field',f))
        m = TVals(Time(123, 456), Duration(78, 90))
        v = _str_plot_fields(m, 'field', f)
        self.assertEquals('time,field.t,field.d', v)
        m = simple_v
        self.assertEquals(simple_nostr, _str_plot_fields(m, 'field', f))
        m = arrays_v
        self.assertEquals(arrays_nostr, _str_plot_fields(m, 'field', f))
        m = embed_v
        self.assertEquals(embed_nostr, _str_plot_fields(m, 'field', f))

        f = create_field_filter(echo_nostr=False, echo_noarr=True)
        m = String()
        self.assertEquals("time,field.data", _str_plot_fields(m, 'field',f))
        m = String('foo')
        self.assertEquals("time,field.data", _str_plot_fields(m, 'field',f))
        m = TVals(Time(123, 456), Duration(78, 90))
        v = _str_plot_fields(m, 'field', f)
        self.assertEquals('time,field.t,field.d', v)
        m = simple_v
        self.assertEquals(simple_d, _str_plot_fields(m, 'field', f))
        m = arrays_v
        self.assertEquals('time,', _str_plot_fields(m, 'field', f))
        m = embed_v
        self.assertEquals(embed_noarr, _str_plot_fields(m, 'field', f))

        f = create_field_filter(echo_nostr=True, echo_noarr=True)
        m = String()
        self.assertEquals("time,", _str_plot_fields(m, 'field', f))
        m = String('foo')
        self.assertEquals('time,', _str_plot_fields(m, 'field', f))
        m = TVals(Time(123, 456), Duration(78, 90))
        v = _str_plot_fields(m, 'field', f)
        self.assertEquals('time,field.t,field.d', v)
        m = simple_v
        self.assertEquals(simple_nostr, _str_plot_fields(m, 'field', f))
        m = arrays_v
        self.assertEquals('time,', _str_plot_fields(m, 'field', f))
        m = embed_v
        self.assertEquals(embed_nostr_noarr, _str_plot_fields(m, 'field', f))
        
    def test_str_plot(self):
        from rostopic import _str_plot
        from std_msgs.msg import String, Int32, Header
        from test_ros.msg import Simple, TVals, Floats, Arrays, Embed

        from genpy import Time, Duration
        from rostopic import create_field_filter

        # str plotting requires rospy time, we fix time to a set time
        import rospy.rostime
        rospy.rostime.set_rostime_initialized(True)
        rospy.rostime._set_rostime(Time(0, 1234))
        r_time = Time(0, 5678)

        # prepare test values
        simple_v = Simple(1, -2, 3, -4, 'a', 7, 8, 9, 'bar')
        simple_d = '1234,1,-2,3,-4,a,7,8,9,bar'
        simple_nostr = '1234,1,-2,3,-4,a,7,8,9'

        arrays_v = Arrays([-1], chr(2)+chr(3), [3, 4, 5], [6, 7, 8], ['a1', 'b2', 'b3'], [Time(123, 456), Time(78, 90)])
        arrays_d = '1234,-1,2,3,3,4,5,6,7,8,a1,b2,b3,123000000456,78000000090'
        arrays_nostr = '1234,-1,2,3,3,4,5,6,7,8,123000000456,78000000090'

        embed_v = Embed(simple_v, arrays_v)
        embed_d = simple_d+','+arrays_d[5:]

        # test current_time override
        m = String('foo')
        self.assertEquals('5678,foo', _str_plot(m, current_time=r_time, field_filter=None))

        # test over all combinations of field filters
        f = create_field_filter(echo_nostr=False, echo_noarr=False)
        m = String()
        self.assertEquals("1234,", _str_plot(m, field_filter=f))
        m = String('foo')
        self.assertEquals('1234,foo', _str_plot(m, field_filter=f))
        m = TVals(Time(123, 456), Duration(78, 90))
        v = _str_plot(m, field_filter=f)
        self.assertEquals('1234,123000000456,78000000090', v)
        m = simple_v
        self.assertEquals(simple_d, _str_plot(m, field_filter=f))
        m = arrays_v
        self.assertEquals(arrays_d, _str_plot(m, field_filter=f))
        m = embed_v
        self.assertEquals(embed_d, _str_plot(m, field_filter=f))

        f = create_field_filter(echo_nostr=True, echo_noarr=False)
        m = String()
        self.assertEquals("1234,", _str_plot(m, field_filter=f))
        m = String('foo')
        self.assertEquals('1234,', _str_plot(m, field_filter=f))
        m = TVals(Time(123, 456), Duration(78, 90))
        v = _str_plot(m, field_filter=f)
        self.assertEquals('1234,123000000456,78000000090', v)
        m = simple_v
        self.assertEquals(simple_nostr, _str_plot(m, field_filter=f))
        m = arrays_v
        self.assertEquals(arrays_nostr, _str_plot(m, field_filter=f))
        m = embed_v
        self.assertEquals(simple_nostr+arrays_nostr[4:], _str_plot(m, field_filter=f))

        f = create_field_filter(echo_nostr=False, echo_noarr=True)
        m = String()
        self.assertEquals("1234,", _str_plot(m, field_filter=f))
        m = String('foo')
        self.assertEquals('1234,foo', _str_plot(m, field_filter=f))
        m = TVals(Time(123, 456), Duration(78, 90))
        v = _str_plot(m, field_filter=f)
        self.assertEquals('1234,123000000456,78000000090', v)
        m = simple_v
        self.assertEquals(simple_d, _str_plot(m, field_filter=f))
        m = arrays_v
        self.assertEquals('1234,', _str_plot(m, field_filter=f))
        m = embed_v
        self.assertEquals(simple_d, _str_plot(m, field_filter=f))

        f = create_field_filter(echo_nostr=True, echo_noarr=True)
        m = String()
        self.assertEquals("1234,", _str_plot(m, field_filter=f))
        m = String('foo')
        self.assertEquals('1234,', _str_plot(m, field_filter=f))
        m = TVals(Time(123, 456), Duration(78, 90))
        v = _str_plot(m, field_filter=f)
        self.assertEquals('1234,123000000456,78000000090', v)
        m = simple_v
        self.assertEquals(simple_nostr, _str_plot(m, field_filter=f))
        m = arrays_v
        self.assertEquals('1234,', _str_plot(m, field_filter=f))
        m = embed_v
        self.assertEquals(simple_nostr, _str_plot(m, field_filter=f))

    def test_strify_message(self):
        # strify message is part of roslib, but we want to test with
        # rostopic's field filters.  It's also the case that
        # roslib.messages cannot be unit tested within the ROS stack
        # -- part of the reason it needs to be moved elsewhere.
        from std_msgs.msg import String, Int32, Header
        from test_ros.msg import Simple, TVals, Floats, Arrays, Embed

        from genpy import Time, Duration
        from roslib.message import strify_message
        from rostopic import create_field_filter

        simple_v = Simple(1, -2, 3, -4, 'a', 7, 8, 9, 'bar')
        simple_d = {'b': 1, 'int16': -2, 'int32': 3, 'int64': -4, 'c': 'a', 'uint16': 7, 'uint32': 8, 'uint64': 9, 'str': 'bar'}
        simple_nostr = simple_d.copy()
        del simple_nostr['str']

        arrays_v = Arrays([-1], chr(2)+chr(3), [3, 4, 5], [6, 7, 8], ['a1', 'b2', 'b3'], [Time(123, 456), Time(78, 90)])
        arrays_d = {'int8_arr': [-1], 'uint8_arr': [2, 3], 'int32_arr': [3, 4, 5], 'uint32_arr': [6, 7, 8], 'string_arr': ['a1', 'b2', 'b3'], 'time_arr': [{'secs': 123, 'nsecs': 456}, {'secs': 78, 'nsecs': 90}]}
        arrays_nostr = arrays_d.copy()
        del arrays_nostr['string_arr']

        embed_v = Embed(simple_v, arrays_v)
        embed_d = {'simple': simple_d, 'arrays': arrays_d}

        f = create_field_filter(echo_nostr=False, echo_noarr=False)
        m = String()
        self.assertEquals("data: ''", strify_message(m, field_filter=f))
        m = String('foo')
        self.assertEquals('data: foo', strify_message(m, field_filter=f))
        m = TVals(Time(123, 456), Duration(78, 90))
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals({'t': {'secs': 123, 'nsecs': 456}, 'd': {'secs': 78, 'nsecs': 90}}, v)
        m = simple_v
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals(simple_d, v)
        m = arrays_v
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals(arrays_d, v)
        m = embed_v
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals(embed_d, v)

        f = create_field_filter(echo_nostr=True, echo_noarr=False)
        m = String()
        self.assertEquals('', strify_message(m, field_filter=f))
        m = String('foo')
        self.assertEquals('', strify_message(m, field_filter=f))
        m = TVals(Time(123, 456), Duration(78, 90))
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals({'t': {'secs': 123, 'nsecs': 456}, 'd': {'secs': 78, 'nsecs': 90}}, v)
        m = simple_v
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals(simple_nostr, v)
        m = arrays_v
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals(arrays_nostr, v)
        m = embed_v
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals({'simple': simple_nostr, 'arrays': arrays_nostr}, v)

        f = create_field_filter(echo_nostr=False, echo_noarr=True)
        m = String()
        self.assertEquals("data: ''", strify_message(m, field_filter=f))
        m = String('foo')
        self.assertEquals('data: foo', strify_message(m, field_filter=f))
        m = TVals(Time(123, 456), Duration(78, 90))
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals({'t': {'secs': 123, 'nsecs': 456}, 'd': {'secs': 78, 'nsecs': 90}}, v)
        m = simple_v
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals(simple_d, v)
        m = arrays_v
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals(None, v)
        m = embed_v
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals({'simple': simple_d, 'arrays': None}, v)

        f = create_field_filter(echo_nostr=True, echo_noarr=True)
        m = String()
        self.assertEquals('', strify_message(m, field_filter=f))
        m = String('foo')
        self.assertEquals('', strify_message(m, field_filter=f))
        m = TVals(Time(123, 456), Duration(78, 90))
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals({'t': {'secs': 123, 'nsecs': 456}, 'd': {'secs': 78, 'nsecs': 90}}, v)
        m = simple_v
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals(simple_nostr, v)
        m = embed_v
        v = yaml.load(strify_message(m, field_filter=f))
        self.assertEquals({'simple': simple_nostr, 'arrays': None}, v)
        
    def test_create_field_filter(self):
        from std_msgs.msg import String, Int32, Header
        from test_ros.msg import Simple, TVals, Floats, Arrays, Embed

        from rostopic import create_field_filter
        f = create_field_filter(echo_nostr=False, echo_noarr=False)
        m = String()
        self.assertEquals(['data'], list(f(m)))
        m = Int32()
        self.assertEquals(['data'], list(f(m)))
        m = Arrays()
        self.assertEquals(['int8_arr', 'uint8_arr', 'int32_arr', 'uint32_arr', 'string_arr', 'time_arr'], list(f(m)))
        m = Embed()
        self.assertEquals(['simple', 'arrays'], list(f(m)))        
        m = Simple()
        self.assertEquals(['b', 'int16', 'int32', 'int64', 'c', 'uint16', 'uint32', 'uint64', 'str'], list(f(m)))
        m = Floats()
        self.assertEquals(['float32', 'float64'], list(f(m)))
        m = TVals()
        self.assertEquals(['t', 'd'], list(f(m)))
        m = Header()
        self.assertEquals(['seq', 'stamp', 'frame_id'], list(f(m)))

        f = create_field_filter(echo_nostr=True, echo_noarr=False)
        m = String()
        self.assertEquals([], list(f(m)))
        m = Int32()
        self.assertEquals(['data'], list(f(m)))
        m = Arrays()
        self.assertEquals(['int8_arr', 'uint8_arr', 'int32_arr', 'uint32_arr', 'time_arr'], list(f(m)))
        m = Embed()
        self.assertEquals(['simple', 'arrays'], list(f(m)))
        m = Simple()
        self.assertEquals(['b', 'int16', 'int32', 'int64', 'c', 'uint16', 'uint32', 'uint64'], list(f(m)))
        m = Floats()
        self.assertEquals(['float32', 'float64'], list(f(m)))
        m = TVals()
        self.assertEquals(['t', 'd'], list(f(m)))
        m = Header()
        self.assertEquals(['seq', 'stamp'], list(f(m)))

        f = create_field_filter(echo_nostr=False, echo_noarr=True)
        m = String()
        self.assertEquals(['data'], list(f(m)))
        m = Int32()
        self.assertEquals(['data'], list(f(m)))
        m = Arrays()
        self.assertEquals([], list(f(m)))
        m = Embed()
        self.assertEquals(['simple', 'arrays'], list(f(m)))        
        m = Simple()
        self.assertEquals(['b', 'int16', 'int32', 'int64', 'c', 'uint16', 'uint32', 'uint64', 'str'], list(f(m)))
        m = Floats()
        self.assertEquals(['float32', 'float64'], list(f(m)))
        m = TVals()
        self.assertEquals(['t', 'd'], list(f(m)))
        m = Header()
        self.assertEquals(['seq', 'stamp', 'frame_id'], list(f(m)))
        
        f = create_field_filter(echo_nostr=True, echo_noarr=True)
        m = String()
        self.assertEquals([], list(f(m)))
        m = Int32()
        self.assertEquals(['data'], list(f(m)))
        m = Arrays()
        self.assertEquals([], list(f(m)))
        m = Embed()
        self.assertEquals(['simple', 'arrays'], list(f(m)))        
        m = Simple()
        self.assertEquals(['b', 'int16', 'int32', 'int64', 'c', 'uint16', 'uint32', 'uint64'], list(f(m)))
        m = Floats()
        self.assertEquals(['float32', 'float64'], list(f(m)))
        m = TVals()
        self.assertEquals(['t', 'd'], list(f(m)))
        m = Header()
        self.assertEquals(['seq', 'stamp'], list(f(m)))
