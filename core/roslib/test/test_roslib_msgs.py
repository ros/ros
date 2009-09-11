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
import roslib; roslib.load_manifest('roslib')

import os
import sys
import unittest

import roslib.msgs

# avoid circular package depend with dyn import
roslib.load_manifest('rostest')
import rostest

class MsgSpecTest(unittest.TestCase):
  
  def test_verbose(self):
    self.failIf(roslib.msgs.is_verbose())
    roslib.msgs.set_verbose(True)
    self.assert_(roslib.msgs.is_verbose())
    roslib.msgs.set_verbose(False)    
    self.failIf(roslib.msgs.is_verbose())
    
  def test_base_msg_type(self):
    tests = [(None, None), ('String', 'String'), ('std_msgs/String', 'std_msgs/String'),
             ('String[10]', 'String'), ('string[10]', 'string'), ('std_msgs/String[10]', 'std_msgs/String'),
             ]
    for val, res in tests:
      self.assertEquals(res, roslib.msgs.base_msg_type(val))

  def test_Constant(self):
    import random
    vals = [random.randint(0, 1000) for i in xrange(0, 3)]
    type, name, val = [str(x) for x in vals]
    x = roslib.msgs.Constant(type, name, val)
    self.assertEquals(type, x.type)
    self.assertEquals(name, x.name)
    self.assertEquals(val, x.val)

    try:
      roslib.msgs.Constant(None, name, val)
    except: pass
    try:
      roslib.msgs.Constant(type, None, val)
    except: pass
    try:
      roslib.msgs.Constant(type, name, None)      
    except: pass
    
    try:
      x.foo = 'bar'
      self.fail('Constant should not allow arbitrary attr assignment')
    except: pass
    
  def test_MsgSpec(self):
    def sub_test_MsgSpec(types, names, constants, text, has_header):
      m = MsgSpec(types, names, constants, text)
      self.assertEquals(m.types, types)
      self.assertEquals(m.names, names)
      self.assertEquals(m.text, text)
      self.assertEquals(has_header, m.has_header())
      self.assertEquals(m.constants, constants)
      self.assertEquals(zip(types, names), m.fields())
      self.assertEquals(m, MsgSpec(types, names, constants, text))
      return m
    
    from roslib.msgs import MsgSpec
    # allow empty msg
    empty = sub_test_MsgSpec([], [], [], '', False)
    self.assertEquals([], empty.fields())    

    # one-field
    one_field = sub_test_MsgSpec(['int32'], ['x'], [], 'int32 x', False)
    # make sure that equals tests every declared field
    self.assertEquals(one_field, MsgSpec(['int32'], ['x'], [], 'int32 x'))
    self.assertNotEquals(one_field, MsgSpec(['uint32'], ['x'], [], 'int32 x'))
    self.assertNotEquals(one_field, MsgSpec(['int32'], ['y'], [], 'int32 x'))
    self.assertNotEquals(one_field, MsgSpec(['int32'], ['x'], [], 'uint32 x'))
    # test against __ne__ as well
    self.assert_(one_field != MsgSpec(['int32'], ['x'], [], 'uint32 x'))
    
    # test variations of multiple fields and headers
    two_fields = sub_test_MsgSpec(['int32', 'string'], ['x', 'str'], [], 'int32 x\nstring str', False)
    one_header = sub_test_MsgSpec(['Header'], ['header'], [], 'Header header', True)
    header_and_fields = sub_test_MsgSpec(['Header', 'int32', 'string'], ['header', 'x', 'str'], [], 'Header header\nint32 x\nstring str', True)
    embed_types = sub_test_MsgSpec(['Header', 'std_msgs/Int32', 'string'], ['header', 'x', 'str'], [], 'Header header\nstd_msgs/Int32 x\nstring str', True)
    
    # types and names mismatch
    try:
      MsgSpec(['int32', 'int32'], ['intval'], [], 'int32 intval\int32 y')
      self.fail("types and names must align")
    except: pass

    # test (not) equals against non msgspec
    self.failIf(one_field == 1)
    self.assert_(one_field != 1)    
    #TODO: test flatten

    # test that repr doesn't throw an error
    [repr(x) for x in [empty, one_field, one_header, two_fields, embed_types]]

  def test_init(self):
    roslib.msgs._initialized = False
    roslib.msgs._init()
    self.assert_(roslib.msgs._initialized)
    # test repeated initialization
    roslib.msgs._init()    

  def test_list_msg_types(self):
    # can only validly test with roslib, but we test with std_msgs anyways
    types1 = roslib.msgs.list_msg_types('roslib', False)
    types2 = roslib.msgs.list_msg_types('roslib', True)
    
    self.assert_('Header' in types1, types1)
    self.assert_('Time' in types1, types1)
    self.assert_('Log' in types1, types1)        
    
    # roslib has no deps, so this should be equivalent
    self.assertEquals(types1, types2)

    #TODO: need a real test with dependencies
    types1 = roslib.msgs.list_msg_types('std_msgs', False)
    types2 = roslib.msgs.list_msg_types('std_msgs', True)
    
    self.assert_('String' in types1, types1)
    self.assert_('String' in types2, types2)    
    self.assert_('Int32' in types1, types1)
    self.assert_('Int32' in types2, types2)    
    
    self.assert_('roslib/Header' in types2, types2)
    self.assert_('roslib/Header' not in types1, types1)
    self.assert_('roslib/Time' in types2, types2)
    self.assert_('roslib/Time' not in types1, types1)
    self.assert_('roslib/Log' in types2, types2)        
    self.assert_('roslib/Log' not in types1, types1)        
    
    # std_msgs depends on roslib, so these should be different
    self.assertNotEquals(types1, types2)

  def test_msg_file(self):
    f = roslib.msgs.msg_file('roslib', 'Log')
    self.assert_(os.path.isfile(f))
    self.assert_(f.endswith('roslib/msg/Log.msg'))

    # msg_file should return paths even for non-existent resources
    f = roslib.msgs.msg_file('roslib', 'Fake')
    self.failIf(os.path.isfile(f))
    self.assert_(f.endswith('roslib/msg/Fake.msg'))

  def test_is_valid_msg_type(self):

    vals = [
      #basic
      'F', 'f', 'Foo', 'Foo1',
      'std_msgs/String',
      # arrays
      'Foo[]', 'Foo[1]', 'Foo[10]',
      ]
    for v in vals:
      self.assert_(roslib.msgs.is_valid_msg_type(v), "roslib.msgs.is_valid_msg_type should have returned True for '%s'"%v)
 
    # bad cases
    vals = [None, '', '#', '%', 'Foo%', 'Woo Woo',
            '/', '/String', 
            'Foo[f]', 'Foo[1d]', 'Foo[-1]', 'Foo[1:10]', 'Foo[', 'Foo]', 'Foo[]Bar']
    for v in vals:
      self.failIf(roslib.msgs.is_valid_msg_type(v), "roslib.msgs.is_valid_msg_type should have returned False for '%s'"%v)
      
  def test_is_valid_constant_type(self):
    valid = ['int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', \
             'uint64', 'float32', 'float64', 'char', 'byte', 'string']
    invalid = [
      'std_msgs/String', '/', 'String',
      'time', 'duration','header',
    ]
    for v in valid:
      self.assert_(roslib.msgs.is_valid_constant_type(v), "roslib.msgs.is_valid_constant_type should have returned True for '%s'"%v)
    for v in invalid:
      self.failIf(roslib.msgs.is_valid_constant_type(v), "roslib.msgs.is_valid_constant_type should have returned False for '%s'"%v)
    

if __name__ == '__main__':
  rostest.unitrun('roslib', 'test_msgspec', MsgSpecTest, coverage_packages=['roslib.msgs'])

