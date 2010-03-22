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

NAME = 'test_genpy'
import roslib; roslib.load_manifest('test_roslib')

import sys 
import unittest
import cStringIO
import time
        
import rostest

# NOTE: roslib.genpy is in the roslib package to prevent circular
# dependencies on messages in the roslib package
# (Header/Time/Duration)

class TestGenpy(unittest.TestCase):

    def setUp(self):
        pass
        
    ## Test genpy.reduce_pattern
    def test_reduce_pattern(self):
        tests = [
            ('', ''),
            ('hhhh', '4h'),
            ('hhhhi', '4hi'),
            ('hhhhiiiibbb', '4h4i3b'),            
            ('1h2h3h', '1h2h3h'),            
            ('hIi', 'hIi'),
            ('66h', '66h'),
            ('%ss', '%ss'), #don't reduce strings with format chars in them
            ('<I', '<I'),
            ('<11s', '<11s'),            
            ]
        from roslib.genpy import reduce_pattern
        for input, result in tests:
            self.assertEquals(result, reduce_pattern(input))
    def test_is_simple(self):
        from roslib.genpy import is_simple
        for t in ['uint8', 'int8', 'uint16', 'int16', 'uint32', 'int32', 'uint64', 'int64', 'float32', 'float64', 'byte', 'char']:
            self.assert_(is_simple(t))
    def test_SIMPLE_TYPES(self):
        import roslib.msgs
        import roslib.genpy
        # tripwire to make sure we don't add builtin types without making sure that simple types has been updated
        self.assertEquals(set(['string', 'time', 'duration']),
                          set(roslib.msgs.BUILTIN_TYPES) - set(roslib.genpy.SIMPLE_TYPES))
    def test_is_special(self):
        from roslib.genpy import is_special
        for t in ['time', 'duration', 'Header']:
            self.assert_(is_special(t))
    def test_Simple(self):
        from roslib.genpy import get_special
        self.assertEquals('import roslib.rostime', get_special('time').import_str)
        self.assertEquals('import roslib.rostime', get_special('duration').import_str)
        self.assertEquals('import roslib.msg', get_special('Header').import_str)

        self.assertEquals('roslib.rostime.Time()', get_special('time').constructor)
        self.assertEquals('roslib.rostime.Duration()', get_special('duration').constructor)
        self.assertEquals('roslib.msg._Header.Header()', get_special('Header').constructor)

        self.assertEquals('self.foo.canon()', get_special('time').get_post_deserialize('self.foo'))
        self.assertEquals('bar.canon()', get_special('time').get_post_deserialize('bar'))
        self.assertEquals('self.foo.canon()', get_special('duration').get_post_deserialize('self.foo'))
        self.assertEquals(None, get_special('Header').get_post_deserialize('self.foo'))

    def test_compute_post_deserialize(self):
        from roslib.genpy import compute_post_deserialize
        self.assertEquals('self.bar.canon()', compute_post_deserialize('time', 'self.bar'))
        self.assertEquals('self.bar.canon()', compute_post_deserialize('duration', 'self.bar'))
        self.assertEquals(None, compute_post_deserialize('Header', 'self.bar'))

        self.assertEquals(None, compute_post_deserialize('int8', 'self.bar'))
        self.assertEquals(None, compute_post_deserialize('string', 'self.bar'))        

    def test_compute_struct_pattern(self):
        from roslib.genpy import compute_struct_pattern
        self.assertEquals(None, compute_struct_pattern(None))
        self.assertEquals(None, compute_struct_pattern([]))
        # string should immediately bork any simple types
        self.assertEquals(None, compute_struct_pattern(['string']))
        self.assertEquals(None, compute_struct_pattern(['uint32', 'string']))
        self.assertEquals(None, compute_struct_pattern(['string', 'int32']))
        # array types should not compute
        self.assertEquals(None, compute_struct_pattern(['uint32[]']))
        self.assertEquals(None, compute_struct_pattern(['uint32[1]']))

        self.assertEquals("B", compute_struct_pattern(['uint8']))
        self.assertEquals("BB", compute_struct_pattern(['uint8', 'uint8']))        
        self.assertEquals("B", compute_struct_pattern(['char']))
        self.assertEquals("BB", compute_struct_pattern(['char', 'char']))        
        self.assertEquals("b", compute_struct_pattern(['byte']))
        self.assertEquals("bb", compute_struct_pattern(['byte', 'byte']))        
        self.assertEquals("b", compute_struct_pattern(['int8']))
        self.assertEquals("bb", compute_struct_pattern(['int8', 'int8']))        
        self.assertEquals("H", compute_struct_pattern(['uint16']))
        self.assertEquals("HH", compute_struct_pattern(['uint16', 'uint16']))        
        self.assertEquals("h", compute_struct_pattern(['int16']))
        self.assertEquals("hh", compute_struct_pattern(['int16', 'int16']))        
        self.assertEquals("I", compute_struct_pattern(['uint32']))
        self.assertEquals("II", compute_struct_pattern(['uint32', 'uint32']))        
        self.assertEquals("i", compute_struct_pattern(['int32']))
        self.assertEquals("ii", compute_struct_pattern(['int32', 'int32']))        
        self.assertEquals("Q", compute_struct_pattern(['uint64']))
        self.assertEquals("QQ", compute_struct_pattern(['uint64', 'uint64']))        
        self.assertEquals("q", compute_struct_pattern(['int64']))
        self.assertEquals("qq", compute_struct_pattern(['int64', 'int64']))        
        self.assertEquals("f", compute_struct_pattern(['float32']))
        self.assertEquals("ff", compute_struct_pattern(['float32', 'float32']))        
        self.assertEquals("d", compute_struct_pattern(['float64']))
        self.assertEquals("dd", compute_struct_pattern(['float64', 'float64']))

        self.assertEquals("bBhHiIqQfd", compute_struct_pattern(['int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'float32', 'float64']))

    def test_flatten(self):
        from roslib.msgs import register, MsgSpec
        from roslib.genpy import flatten

        simple = MsgSpec(['string'], ['data'], [], 'string data\n')
        simple2 = MsgSpec(['string', 'int32'], ['data', 'data2'], [], 'string data\nint32 data2\n')
        self.assertEquals(simple, flatten(simple))
        self.assertEquals(simple2, flatten(simple2))
        
        b1 = MsgSpec(['int8'], ['data'], [], 'X')
        b2 = MsgSpec(['f_msgs/Base'], ['data'], [], 'X')
        b3 = MsgSpec(['f_msgs/Base2', 'f_msgs/Base2'], ['data3', 'data4'], [], 'X')
        b4 = MsgSpec(['f_msgs/Base3', 'f_msgs/Base3'], ['dataA', 'dataB'], [], 'X')
        register('f_msgs/Base', b1)
        register('f_msgs/Base2', b2)
        register('f_msgs/Base3', b3)
        register('f_msgs/Base4', b4)

        self.assertEquals(MsgSpec(['int8'], ['data.data'], [], 'X'), flatten(b2))
        self.assertEquals(MsgSpec(['int8', 'int8'], ['data3.data.data', 'data4.data.data'], [], 'X'), flatten(b3))        
        self.assertEquals(MsgSpec(['int8', 'int8', 'int8', 'int8'],
                                  ['dataA.data3.data.data', 'dataA.data4.data.data', 'dataB.data3.data.data', 'dataB.data4.data.data'],
                                  [], 'X'), flatten(b4))
        
    def test_numpy_dtype(self):
        for t in roslib.genpy.SIMPLE_TYPES:
            self.assert_(t in roslib.genpy._NUMPY_DTYPE)

    def test_default_value(self):
        from roslib.msgs import register, MsgSpec
        from roslib.genpy import default_value

        register('fake_msgs/String', MsgSpec(['string'], ['data'], [], 'string data\n'))
        register('fake_msgs/ThreeNums', MsgSpec(['int32', 'int32', 'int32'], ['x', 'y', 'z'], [], 'int32 x\nint32 y\nint32 z\n'))
        
        # trip-wire: make sure all builtins have a default value
        for t in roslib.msgs.BUILTIN_TYPES:
            self.assert_(type(default_value(t, 'roslib')) == str)
            
        # simple types first
        for t in ['uint8', 'int8', 'uint16', 'int16', 'uint32', 'int32', 'uint64', 'int64', 'byte', 'char']:
            self.assertEquals('0', default_value(t, 'std_msgs'))
            self.assertEquals('0', default_value(t, 'roslib'))
        for t in ['float32', 'float64']:
            self.assertEquals('0.', default_value(t, 'std_msgs'))
            self.assertEquals('0.', default_value(t, 'roslib'))
        self.assertEquals("''", default_value('string', 'roslib'))

        # builtin specials
        self.assertEquals('roslib.rostime.Time()', default_value('time', 'roslib'))
        self.assertEquals('roslib.rostime.Duration()', default_value('duration', 'roslib'))
        self.assertEquals('roslib.msg._Header.Header()', default_value('Header', 'roslib'))

        self.assertEquals('roslib.rostime.Time()', default_value('time', 'std_msgs'))
        self.assertEquals('roslib.rostime.Duration()', default_value('duration', 'std_msgs'))
        self.assertEquals('roslib.msg._Header.Header()', default_value('Header', 'std_msgs'))

        # generic instances
        # - unregistered type
        self.assertEquals(None, default_value("unknown_msgs/Foo", "unknown_msgs"))
        # - wrong context
        self.assertEquals(None, default_value('ThreeNums', 'std_msgs'))

        # - registered types
        self.assertEquals('fake_msgs.msg.String()', default_value('fake_msgs/String', 'std_msgs'))
        self.assertEquals('fake_msgs.msg.String()', default_value('fake_msgs/String', 'fake_msgs'))
        self.assertEquals('fake_msgs.msg.String()', default_value('String', 'fake_msgs'))
        self.assertEquals('fake_msgs.msg.ThreeNums()', default_value('fake_msgs/ThreeNums', 'roslib'))
        self.assertEquals('fake_msgs.msg.ThreeNums()', default_value('fake_msgs/ThreeNums', 'fake_msgs'))
        self.assertEquals('fake_msgs.msg.ThreeNums()', default_value('ThreeNums', 'fake_msgs'))

        # var-length arrays always default to empty arrays... except for byte and uint8 which are strings
        for t in ['int8', 'uint16', 'int16', 'uint32', 'int32', 'uint64', 'int64', 'float32', 'float64', 'char']:
            self.assertEquals('[]', default_value(t+'[]', 'std_msgs'))
            self.assertEquals('[]', default_value(t+'[]', 'roslib'))

        self.assertEquals("''", default_value('uint8[]', 'roslib'))
        self.assertEquals("''", default_value('byte[]', 'roslib'))
        
        # fixed-length arrays should be zero-filled... except for byte and uint8 which are strings
        for t in ['float32', 'float64']:
            self.assertEquals('[0.,0.,0.]', default_value(t+'[3]', 'std_msgs'))
            self.assertEquals('[0.]', default_value(t+'[1]', 'std_msgs'))
        for t in ['int8', 'uint16', 'int16', 'uint32', 'int32', 'uint64', 'int64', 'char']:
            self.assertEquals('[0,0,0,0]', default_value(t+'[4]', 'std_msgs'))
            self.assertEquals('[0]', default_value(t+'[1]', 'roslib'))

        self.assertEquals("chr(0)*1", default_value('uint8[1]', 'roslib'))
        self.assertEquals("chr(0)*4", default_value('uint8[4]', 'roslib'))
        self.assertEquals("chr(0)*1", default_value('byte[1]', 'roslib'))
        self.assertEquals("chr(0)*4", default_value('byte[4]', 'roslib'))
        
        self.assertEquals('[]', default_value('fake_msgs/String[]', 'std_msgs'))        
        self.assertEquals('[fake_msgs.msg.String(),fake_msgs.msg.String()]', default_value('fake_msgs/String[2]', 'std_msgs'))        

    def test_make_python_safe(self):
        from roslib.msgs import register, MsgSpec, Constant
        from roslib.genpy import make_python_safe
        s = MsgSpec(['int32', 'int32', 'int32', 'int32'], ['ok', 'if', 'self', 'fine'],
                    [Constant('int32', 'if', '1', '1'), Constant('int32', 'okgo', '1', '1')],
                    'x')
        s2 = make_python_safe(s)
        self.assertNotEquals(s, s2)
        self.assertEquals(['ok', 'if_', 'self_', 'fine'], s2.names)
        self.assertEquals(s2.types, s.types)
        self.assertEquals([Constant('int32', 'if_', '1', '1'), Constant('int32', 'okgo', '1', '1')], s2.constants)
        self.assertEquals(s2.text, s.text)
    
    def test_compute_pkg_type(self):
        from roslib.genpy import compute_pkg_type, MsgGenerationException
        try:
            compute_pkg_type('std_msgs', 'really/bad/std_msgs/String')
        except MsgGenerationException: pass
        self.assertEquals(('std_msgs', 'String'), compute_pkg_type('std_msgs', 'std_msgs/String'))
        self.assertEquals(('std_msgs', 'String'), compute_pkg_type('foo', 'std_msgs/String'))        
        self.assertEquals(('std_msgs', 'String'), compute_pkg_type('std_msgs', 'String'))
        
    def test_compute_import(self):
        from roslib.msgs import register, MsgSpec
        from roslib.genpy import compute_import

        self.assertEquals([], compute_import('foo', 'bar'))
        self.assertEquals([], compute_import('foo', 'int32'))        
        
        register('ci_msgs/Base', MsgSpec(['int8'], ['data'], [], 'int8 data\n'))
        register('ci2_msgs/Base2', MsgSpec(['ci_msgs/Base'], ['data2'], [], 'ci_msgs/Base data2\n'))
        register('ci3_msgs/Base3', MsgSpec(['ci2_msgs/Base2'], ['data3'], [], 'ci2_msgs/Base2 data3\n'))
        register('ci4_msgs/Base', MsgSpec(['int8'], ['data'], [], 'int8 data\n'))
        register('ci4_msgs/Base4', MsgSpec(['ci2_msgs/Base2', 'ci3_msgs/Base3', 'ci4_msgs/Base'],
                                           ['data4a', 'data4b', 'data4c'],
                                           [], 'ci2_msgs/Base2 data4a\nci3_msgs/Base3 data4b\nci4_msgs/Base data4c\n'))

        register('ci5_msgs/Base', MsgSpec(['time'], ['data'], [], 'time data\n'))
        
        self.assertEquals(['import ci_msgs.msg'], compute_import('foo', 'ci_msgs/Base'))
        self.assertEquals(['import ci_msgs.msg'], compute_import('ci_msgs', 'ci_msgs/Base'))
        self.assertEquals(['import ci2_msgs.msg', 'import ci_msgs.msg'], compute_import('ci2_msgs', 'ci2_msgs/Base2'))
        self.assertEquals(['import ci2_msgs.msg', 'import ci_msgs.msg'], compute_import('foo', 'ci2_msgs/Base2'))
        self.assertEquals(['import ci3_msgs.msg', 'import ci2_msgs.msg', 'import ci_msgs.msg'], compute_import('ci3_msgs', 'ci3_msgs/Base3'))
        
        self.assertEquals(set(['import ci4_msgs.msg', 'import ci3_msgs.msg', 'import ci2_msgs.msg', 'import ci_msgs.msg']),
                          set(compute_import('foo', 'ci4_msgs/Base4')))        
        self.assertEquals(set(['import ci4_msgs.msg', 'import ci3_msgs.msg', 'import ci2_msgs.msg', 'import ci_msgs.msg']),
                          set(compute_import('ci4_msgs', 'ci4_msgs/Base4')))
        
        self.assertEquals(['import ci4_msgs.msg'], compute_import('foo', 'ci4_msgs/Base'))        
        self.assertEquals(['import ci4_msgs.msg'], compute_import('ci4_msgs', 'ci4_msgs/Base'))
        self.assertEquals(['import ci4_msgs.msg'], compute_import('ci4_msgs', 'Base'))    
        
        self.assertEquals(['import ci5_msgs.msg', 'import roslib.rostime'], compute_import('foo', 'ci5_msgs/Base'))
        
    def test_get_registered_ex(self):
        from roslib.msgs import MsgSpec, register
        from roslib.genpy import MsgGenerationException, get_registered_ex
        s = MsgSpec(['string'], ['data'], [], 'string data\n')
        register('tgr_msgs/String', s)
        self.assertEquals(s, get_registered_ex('tgr_msgs/String'))
        try:
            get_registered_ex('bad_msgs/String')
        except MsgGenerationException: pass
            
    def test_compute_constructor(self):
        from roslib.msgs import register, MsgSpec
        from roslib.genpy import compute_constructor
        register('fake_msgs/String', MsgSpec(['string'], ['data'], [], 'string data\n'))
        register('fake_msgs/ThreeNums', MsgSpec(['int32', 'int32', 'int32'], ['x', 'y', 'z'], [], 'int32 x\nint32 y\nint32 z\n'))
        
        # builtin specials
        self.assertEquals('roslib.rostime.Time()', compute_constructor('roslib', 'time'))
        self.assertEquals('roslib.rostime.Duration()', compute_constructor('roslib', 'duration'))
        self.assertEquals('roslib.msg._Header.Header()', compute_constructor('roslib', 'Header'))

        self.assertEquals('roslib.rostime.Time()', compute_constructor('std_msgs', 'time'))
        self.assertEquals('roslib.rostime.Duration()', compute_constructor('std_msgs', 'duration'))
        self.assertEquals('roslib.msg._Header.Header()', compute_constructor('std_msgs', 'Header'))

        # generic instances
        # - unregistered type
        self.assertEquals(None, compute_constructor("unknown_msgs", "unknown_msgs/Foo"))
        self.assertEquals(None, compute_constructor("unknown_msgs", "Foo"))
        # - wrong context
        self.assertEquals(None, compute_constructor('std_msgs', 'ThreeNums'))

        # - registered types
        self.assertEquals('fake_msgs.msg.String()', compute_constructor('std_msgs', 'fake_msgs/String'))
        self.assertEquals('fake_msgs.msg.String()', compute_constructor('fake_msgs', 'fake_msgs/String'))
        self.assertEquals('fake_msgs.msg.String()', compute_constructor('fake_msgs', 'String'))
        self.assertEquals('fake_msgs.msg.ThreeNums()', compute_constructor('fake_msgs', 'fake_msgs/ThreeNums'))
        self.assertEquals('fake_msgs.msg.ThreeNums()', compute_constructor('fake_msgs', 'fake_msgs/ThreeNums'))
        self.assertEquals('fake_msgs.msg.ThreeNums()', compute_constructor('fake_msgs', 'ThreeNums'))

    def test_pack(self):
        from roslib.genpy import pack
        self.assertEquals("buff.write(struct.pack('<3lL3bB', foo, bar))", pack('lllLbbbB', 'foo, bar'))

    def test_pack2(self):
        from roslib.genpy import pack2
        self.assertEquals('buff.write(struct.pack(patt_name, foo, bar))', pack2('patt_name', 'foo, bar'))

    def test_unpack(self):
        from roslib.genpy import unpack
        self.assertEquals("var_x = struct.unpack('<I3if2I',bname)", unpack('var_x', 'IiiifII', 'bname'))

    def test_unpack2(self):
        from roslib.genpy import unpack2
        self.assertEquals('x = struct.unpack(patt, b)', unpack2('x', 'patt', 'b'))

    def test_generate_dynamic(self):
        from roslib.genpy import generate_dynamic
        msgs = generate_dynamic("gd_msgs/EasyString", "string data\n")
        self.assertEquals(['gd_msgs/EasyString'], msgs.keys())
        m_cls = msgs['gd_msgs/EasyString']
        m_instance = m_cls()
        m_instance.data = 'foo'
        buff = cStringIO.StringIO()
        m_instance.serialize(buff)
        m_cls().deserialize(buff.getvalue())

        # 'probot_msgs' is a test for #1183, failure if the package no longer exists
        msgs = generate_dynamic("gd_msgs/MoveArmState", """Header header
probot_msgs/ControllerStatus status

#Current arm configuration
probot_msgs/JointState[] configuration
#Goal arm configuration
probot_msgs/JointState[] goal

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: probot_msgs/ControllerStatus
# This message defines the expected format for Controller Statuss messages
# Embed this in the feedback state message of highlevel controllers
byte UNDEFINED=0
byte SUCCESS=1
byte ABORTED=2
byte PREEMPTED=3
byte ACTIVE=4

# Status of the controller = {UNDEFINED, SUCCESS, ABORTED, PREEMPTED, ACTIVE}
byte value

#Comment for debug
string comment
================================================================================
MSG: probot_msgs/JointState
string name
float64 position
float64 velocity
float64 applied_effort
float64 commanded_effort
byte is_calibrated

""")
        self.assertEquals(set(['gd_msgs/MoveArmState', 'probot_msgs/JointState', 'probot_msgs/ControllerStatus', 'roslib/Header']),
                          set(msgs.keys()))
        import roslib.rostime
        import time
        m_instance1 = msgs['roslib/Header']() # make sure default constructor works
        m_instance2 = msgs['roslib/Header'](stamp=roslib.rostime.Time.from_seconds(time.time()), frame_id='foo-%s'%time.time(), seq=12391)
        self._test_ser_deser(m_instance2, m_instance1)

        m_instance1 = msgs['probot_msgs/ControllerStatus']()
        m_instance2 = msgs['probot_msgs/ControllerStatus'](value=4, comment=str(time.time()))
        d = {'UNDEFINED':0,'SUCCESS':1,'ABORTED':2,'PREEMPTED':3,'ACTIVE':4}
        for k, v in d.iteritems():
            self.assertEquals(v, getattr(m_instance1, k))
        self._test_ser_deser(m_instance2, m_instance1)

        m_instance1 = msgs['probot_msgs/JointState']()
        m_instance2 = msgs['probot_msgs/JointState'](position=time.time(), velocity=time.time(), applied_effort=time.time(), commanded_effort=time.time(), is_calibrated=2)
        self._test_ser_deser(m_instance2, m_instance1)
        
        m_instance1 = msgs['gd_msgs/MoveArmState']()
        js = msgs['probot_msgs/JointState']
        config = []
        goal = []
        # generate some data for config/goal
        for i in range(0, 10):
            config.append(js(position=time.time(), velocity=time.time(), applied_effort=time.time(), commanded_effort=time.time(), is_calibrated=2))
            goal.append(js(position=time.time(), velocity=time.time(), applied_effort=time.time(), commanded_effort=time.time(), is_calibrated=2))
        m_instance2 = msgs['gd_msgs/MoveArmState'](header=msgs['roslib/Header'](),
                                                   status=msgs['probot_msgs/ControllerStatus'](),
                                                   configuration=config, goal=goal)
        self._test_ser_deser(m_instance2, m_instance1)

    def _test_ser_deser(self, m_instance1, m_instance2):
        buff = cStringIO.StringIO()
        m_instance1.serialize(buff)
        m_instance2.deserialize(buff.getvalue())
        self.assertEquals(m_instance1, m_instance2)
        
    def test_len_serializer_generator(self):
        # generator tests are mainly tripwires/coverage tests
        from roslib.genpy import len_serializer_generator
        # Test Serializers
        # string serializer simply initializes local var
        g = len_serializer_generator('foo', True, True)
        self.assertEquals('length = len(foo)', '\n'.join(g))
        # array len serializer writes var
        g = len_serializer_generator('foo', False, True)        
        self.assertEquals("length = len(foo)\nbuff.write(struct.pack('<I', length))", '\n'.join(g))

        # Test Deserializers
        val = """start = end
end += 4
(length,) = struct.unpack('<I',str[start:end])"""
        # string serializer and array serializer are identical
        g = len_serializer_generator('foo', True, False)
        self.assertEquals(val, '\n'.join(g))
        g = len_serializer_generator('foo', False, False)        
        self.assertEquals(val, '\n'.join(g))

    def test_string_serializer_generator(self):
        # generator tests are mainly tripwires/coverage tests
        from roslib.genpy import string_serializer_generator
        # Test Serializers
        g = string_serializer_generator('foo', 'string', 'var_name', True)
        self.assertEquals("""length = len(var_name)
#serialize var_name
buff.write(struct.pack('<I%ss'%length, length, var_name))""", '\n'.join(g))

        for t in ['uint8[]', 'byte[]', 'uint8[10]', 'byte[20]']:
            g = string_serializer_generator('foo', 'uint8[]', 'b_name', True)
            self.assertEquals("""length = len(b_name)
#serialize b_name
# - if encoded as a list instead, serialize as bytes instead of string
if type(b_name) in [list, tuple]:
  buff.write(struct.pack('<I%sB'%length, length, *b_name))
else:
  buff.write(struct.pack('<I%ss'%length, length, b_name))""", '\n'.join(g))

        # Test Deserializers
        val = """start = end
end += 4
(length,) = struct.unpack('<I',str[start:end])
#deserialize var_name
pattern = '<%ss'%length
start = end
end += struct.calcsize(pattern)
(var_name,) = struct.unpack(pattern, str[start:end])"""
        # string serializer and array serializer are identical
        g = string_serializer_generator('foo', 'string', 'var_name', False)
        self.assertEquals(val, '\n'.join(g))

if __name__ == '__main__':
    rostest.unitrun('test_roslib', NAME, TestGenpy, sys.argv, coverage_packages=['roslib.genpy'])
