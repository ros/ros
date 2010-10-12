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
#

## ROS message source code generation for Lisp
## 
## Converts ROS .msg files in a package into Lisp source code

## t0: needed for script to work
## t1: for reference; remove once running
## t2: can be changed once we remove strict diff-compatibility requirement with old version of genmsg_lisp

import roslib; roslib.load_manifest('roslisp')

import sys
import os
import traceback

import roslib.msgs 
import roslib.packages
import roslib.gentools

from cStringIO import StringIO

############################################################
# Built in types
############################################################

def is_fixnum(t):
    return t in ['int8', 'uint8', 'int16', 'uint16']

def is_integer(t):
    return is_fixnum(t) or t in ['byte', 'char', 'int32', 'uint32', 'int64', 'uint64'] #t2 byte, char can be fixnum

def is_signed_int(t):
    return t in ['int8', 'int16', 'int32', 'int64']

def is_unsigned_int(t):
    return t in ['uint8', 'uint16', 'uint32', 'uint64']

def is_bool(t):
    return t == 'bool'

def is_string(t):
    return t == 'string'

def is_float(t):
    return t in ['float16', 'float32', 'float64']

def is_time(t):
    return t in ['time', 'duration']

def field_type(f):
    if f.is_builtin:
        elt_type = lisp_type(f.base_type)
    else:
        elt_type = msg_type(f)
    if f.is_array:
        return '(vector %s)'%elt_type
    else:
        return elt_type

# t2 no need for is_array    
def msg_type(f):
    if f.base_type == 'Header':
        (pkg, msg) = ('roslib', 'Header')
    else:
        (pkg, msg) = f.base_type.split('/')
    return '%s-msg:<%s>'%(pkg, msg)

def lisp_type(t):
    if is_fixnum(t):
        return 'fixnum'
    elif is_integer(t):
        return 'integer'
    elif is_bool(t):
        return 'boolean'
    elif is_float(t):
        return 'float'
    elif is_time(t):
        return 'real'
    elif is_string(t):
        return 'string'
    else:
        raise ValueError('%s is not a recognized primitive type'%t)

def field_initform(f):
    if f.is_builtin:
        initform = lisp_initform(f.base_type)
        elt_type = lisp_type(f.base_type)
    else:
        initform = '(make-instance \'%s)'%msg_type(f)
        elt_type = msg_type(f)
    if f.is_array:
        len = f.array_len or 0
        return '(make-array %s :element-type \'%s :initial-element %s)'%(len, elt_type, initform)
    else:
        return initform

def lisp_initform(t):
    if is_integer(t):
        return '0'
    elif is_bool(t):
        return 'nil'
    elif is_float(t):
        return '0.0'
    elif is_time(t):
        return 0
    elif is_string(t):
        return '\"\"'
    else:
        raise ValueError('%s is not a recognized primitive type'%t)

NUM_BYTES = {'int8': 1, 'int16': 2, 'int32': 4, 'int64': 8,
             'uint8': 1, 'uint16': 2, 'uint32': 4, 'uint64': 8}
             
             

############################################################
# Indented writer
############################################################

class IndentedWriter():

    def __init__(self, s):
        self.str = s
        self.indentation = 0
        self.block_indent = False

    def write(self, s, indent=True, newline=True):
        if not indent:
            newline = False
        if self.block_indent:
            self.block_indent = False
        else:
            if newline:
                self.str.write('\n')
            if indent:
                for i in xrange(self.indentation):
                    self.str.write(' ')
        self.str.write(s)

    def newline(self):
        self.str.write('\n')

    def inc_indent(self, inc=2):
        self.indentation += inc

    def dec_indent(self, dec=2):
        self.indentation -= dec

    def reset_indent(self):
        self.indentation = 0

    def block_next_indent(self):
        self.block_indent = True

class Indent():

    def __init__(self, w, inc=2, indent_first=True):
        self.writer = w
        self.inc = inc
        self.indent_first = indent_first

    def __enter__(self):
        self.writer.inc_indent(self.inc)
        if not self.indent_first:
            self.writer.block_next_indent()

    def __exit__(self, type, val, traceback):
        self.writer.dec_indent(self.inc)

    
        
    

def write_begin(s, spec, path):
    """
    Writes the beginning of the file: a comment saying it's auto-generated and the in-package form
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The spec
    @type spec: roslib.msgs.MsgSpec
    @param path: The file this message is being generated for
    @type path: str
    """
    s.write('; Auto-generated. Do not edit!\n\n\n', newline=False)
    s.write('(in-package %s-msg)\n\n\n'%spec.package, newline=False)
    s.write(';//! \\htmlinclude %s.msg.html\n'%spec.short_name, newline=False) # t2

def write_slot_definition(s, field):
    """
    Write the definition of a slot corresponding to a single message field
    """

    s.write('(%s'%field.name)
    with Indent(s, 1):
        s.write(':reader %s-val'%field.name)
        s.write(':initarg :%s'%field.name)
        s.write(':type %s'%field_type(field))
    i = 0 if field.is_array else 1 # t2
    with Indent(s, i):
        s.write(':initform %s)'%field_initform(field))
            

def write_defclass(s, spec, pkg):
    """
    Writes the defclass that defines the message type
    """
    s.write('(defclass %s (ros-message)'%message_class(spec))
    with Indent(s):
        s.write('(')
        with Indent(s, inc=1, indent_first=False):
            for field in spec.parsed_fields():
                write_slot_definition(s, field)
        s.write(')', indent=False)
    s.write(')')
    
    

def message_class(spec):
    """
    Return the CLOS class name for this message type

    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    """
    return '<%s>'%spec.short_name

    
def write_end(s, spec):
    """
    Writes the end of the header file: the ending of the include guards
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The spec
    @type spec: roslib.msgs.MsgSpec
    """
    s.write('#endif // %s_MESSAGE_%s_H\n'%(spec.package.upper(), spec.short_name.upper()))
    
def write_generic_includes(s):
    """
    Writes the includes that all messages need
    
    @param s: The stream to write to
    @type s: stream
    """
    s.write('#include <string>\n')
    s.write('#include <vector>\n')
    s.write('#include <ostream>\n')
    s.write('#include "ros/serialization.h"\n')
    s.write('#include "ros/builtin_message_traits.h"\n')
    s.write('#include "ros/message_operations.h"\n')
    s.write('#include "ros/message.h"\n')
    s.write('#include "ros/time.h"\n\n')
    
def write_includes(s, spec):
    """
    Writes the message-specific includes
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec to iterate over
    @type spec: roslib.msgs.MsgSpec
    """
    for field in spec.parsed_fields():
        if (not field.is_builtin):
            if (field.is_header):
                s.write('#include "roslib/Header.h"\n')
            else:
                (pkg, name) = roslib.names.package_resource_name(field.base_type)
                pkg = pkg or spec.package # convert '' to package
                s.write('#include "%s/%s.h"\n'%(pkg, name))
                
    s.write('\n') 
    
    
def write_struct(s, spec, cpp_name_prefix):
    """
    Writes the entire message struct: declaration, constructors, members, constants and (deprecated) member functions
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param cpp_name_prefix: The C++ prefix to use when referring to the message, e.g. "std_msgs::"
    @type cpp_name_prefix: str
    """
    
    msg = spec.short_name
    s.write('template <class ContainerAllocator>\n')
    s.write('struct %s_ : public ros::Message\n{\n'%(msg))
    s.write('  typedef %s_<ContainerAllocator> Type;\n\n'%(msg))
    
    write_constructors(s, spec, cpp_name_prefix)
    write_members(s, spec)
    write_constant_declarations(s, spec)
    write_deprecated_member_functions(s, spec)
    
    (cpp_msg_unqualified, cpp_msg_with_alloc, cpp_msg_base) = cpp_message_declarations(cpp_name_prefix, msg)
    s.write('  typedef boost::shared_ptr<%s> Ptr;\n'%(cpp_msg_with_alloc))
    s.write('  typedef boost::shared_ptr<%s const> ConstPtr;\n'%(cpp_msg_with_alloc))
    s.write('}; // struct %s\n'%(msg))
    
    s.write('typedef %s_<std::allocator<void> > %s;\n\n'%(cpp_msg_base, msg))
    s.write('typedef boost::shared_ptr<%s> %sPtr;\n'%(cpp_msg_base, msg))
    s.write('typedef boost::shared_ptr<%s const> %sConstPtr;\n\n'%(cpp_msg_base, msg))

def default_value(type):
    """
    Returns the value to initialize a message member with.  0 for integer types, 0.0 for floating point, false for bool,
    empty string for everything else
    
    @param type: The type
    @type type: str
    """
    if type in ['byte', 'int8', 'int16', 'int32', 'int64',
                'char', 'uint8', 'uint16', 'uint32', 'uint64']:
        return '0'
    elif type in ['float32', 'float64']:
        return '0.0'
    elif type == 'bool':
        return 'false'
        
    return ""

def takes_allocator(type):
    """
    Returns whether or not a type can take an allocator in its constructor.  False for all builtin types except string.
    True for all others.
    
    @param type: The type
    @type: str
    """
    return not type in ['byte', 'int8', 'int16', 'int32', 'int64',
                        'char', 'uint8', 'uint16', 'uint32', 'uint64',
                        'float32', 'float64', 'bool', 'time', 'duration']

def write_initializer_list(s, spec, container_gets_allocator):
    """
    Writes the initializer list for a constructor
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param container_gets_allocator: Whether or not a container type (whether it's another message, a vector, array or string)
        should have the allocator passed to its constructor.  Assumes the allocator is named _alloc.
    @type container_gets_allocator: bool
    """
    
    i = 0
    for field in spec.parsed_fields():
        if (i == 0):
            s.write('  : ')
        else:
            s.write('  , ')
            
        val = default_value(field.base_type)
        use_alloc = takes_allocator(field.base_type)
        if (field.is_array):
            if (field.array_len is None and container_gets_allocator):
                s.write('%s(_alloc)\n'%(field.name))
            else:
                s.write('%s()\n'%(field.name))
        else:
            if (container_gets_allocator and use_alloc):
                s.write('%s(_alloc)\n'%(field.name))
            else:
                s.write('%s(%s)\n'%(field.name, val))
        i = i + 1
        
def write_fixed_length_assigns(s, spec, container_gets_allocator, cpp_name_prefix):
    """
    Initialize any fixed-length arrays
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param container_gets_allocator: Whether or not a container type (whether it's another message, a vector, array or string)
        should have the allocator passed to its constructor.  Assumes the allocator is named _alloc.
    @type container_gets_allocator: bool
    @param cpp_name_prefix: The C++ prefix to use when referring to the message, e.g. "std_msgs::"
    @type cpp_name_prefix: str
    """
    # Assign all fixed-length arrays their default values
    for field in spec.parsed_fields():
        if (not field.is_array or field.array_len is None):
            continue
        
        val = default_value(field.base_type)
        if (container_gets_allocator and takes_allocator(field.base_type)):
            (cpp_msg_unqualified, cpp_msg_with_alloc, _) = cpp_message_declarations(cpp_name_prefix, field.base_type)
            s.write('    %s.assign(%s(_alloc));\n'%(field.name, cpp_msg_with_alloc))
        elif (len(val) > 0):
            s.write('    %s.assign(%s);\n'%(field.name, val))

def write_constructors(s, spec, cpp_name_prefix):
    """
    Writes any necessary constructors for the message
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param cpp_name_prefix: The C++ prefix to use when referring to the message, e.g. "std_msgs::"
    @type cpp_name_prefix: str
    """
    
    msg = spec.short_name
    
    # Default constructor
    s.write('  %s_()\n'%(msg))
    write_initializer_list(s, spec, False)
    s.write('  {\n')
    write_fixed_length_assigns(s, spec, False, cpp_name_prefix)
    s.write('  }\n\n')
    
    # Constructor that takes an allocator constructor
    s.write('  %s_(const ContainerAllocator& _alloc)\n'%(msg))
    write_initializer_list(s, spec, True)
    s.write('  {\n')
    write_fixed_length_assigns(s, spec, True, cpp_name_prefix)
    s.write('  }\n\n')

def write_members(s, spec):
    """
    Write all the member declarations
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    """
    [write_member(s, field) for field in spec.parsed_fields()]
        
def write_constant_declaration(s, constant):
    """
    Write a constant value as a static member
    
    @param s: The stream to write to
    @type s: stream
    @param constant: The constant
    @type constant: roslib.msgs.Constant
    """
    
    # integral types get their declarations as enums to allow use at compile time
    if (constant.type in ['byte', 'int8', 'int16', 'int32', 'int64', 'char', 'uint8', 'uint16', 'uint32', 'uint64']):
        s.write('  enum { %s = %s };\n'%(constant.name, constant.val))
    else:
        s.write('  static const %s %s;\n'%(msg_type_to_cpp(constant.type), constant.name))
        
def write_constant_declarations(s, spec):
    """
    Write all the constants from a spec as static members
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    """
    [write_constant_declaration(s, constant) for constant in spec.constants]
    s.write('\n')
    
def write_constant_definition(s, spec, constant):
    """
    Write a constant value as a static member
    
    @param s: The stream to write to
    @type s: stream
    @param constant: The constant
    @type constant: roslib.msgs.Constant
    """
    
    # integral types do not need a definition, since they've been defined where they are declared
    if (constant.type not in ['byte', 'int8', 'int16', 'int32', 'int64', 'char', 'uint8', 'uint16', 'uint32', 'uint64', 'string']):
        s.write('template<typename ContainerAllocator> const %s %s_<ContainerAllocator>::%s = %s;\n'%(msg_type_to_cpp(constant.type), spec.short_name, constant.name, constant.val))
    elif (constant.type == 'string'):
        s.write('template<typename ContainerAllocator> const %s %s_<ContainerAllocator>::%s = "%s";\n'%(msg_type_to_cpp(constant.type), spec.short_name, constant.name, escape_string(constant.val)))
        
def write_constant_definitions(s, spec):
    """
    Write all the constants from a spec as static members
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    """
    [write_constant_definition(s, spec, constant) for constant in spec.constants]
    s.write('\n')
        
def is_fixed_length(spec):
    """
    Returns whether or not the message is fixed-length
    
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param package: The package of the
    @type package: str
    """
    types = []
    for field in spec.parsed_fields():
        if (field.is_array and field.array_len is None):
            return False
        
        if (field.base_type == 'string'):
            return False
        
        if (not field.is_builtin):
            types.append(field.base_type)
            
    types = set(types)
    for type in types:
        type = roslib.msgs.resolve_type(type, spec.package)
        (_, new_spec) = roslib.msgs.load_by_type(type, spec.package)
        if (not is_fixed_length(new_spec)):
            return False
        
    return True

def is_hex_string(str):
    for c in str:
        if c not in '0123456789abcdefABCDEF':
            return False
        
    return True
def write_operations(s, spec, cpp_name_prefix):
    (cpp_msg_unqualified, cpp_msg_with_alloc, _) = cpp_message_declarations(cpp_name_prefix, spec.short_name)
    s.write('namespace ros\n{\n')
    s.write('namespace message_operations\n{\n')
    
    # Write the Printer operation
    s.write('\ntemplate<class ContainerAllocator>\nstruct Printer<%s>\n{\n'%(cpp_msg_with_alloc))
    s.write('  template<typename Stream> static void stream(Stream& s, const std::string& indent, const %s& v) \n  {\n'%cpp_msg_with_alloc)
    for field in spec.parsed_fields():
        cpp_type = msg_type_to_cpp(field.base_type)
        if (field.is_array):
            s.write('    s << indent << "%s[]" << std::endl;\n'%(field.name))
            s.write('    for (size_t i = 0; i < v.%s.size(); ++i)\n    {\n'%(field.name))
            s.write('      s << indent << "  %s[" << i << "]: ";\n'%field.name)
            indent_increment = '  '
            if (not field.is_builtin):
                s.write('      s << std::endl;\n')
                s.write('      s << indent;\n')
                indent_increment = '    ';
            s.write('      Printer<%s>::stream(s, indent + "%s", v.%s[i]);\n'%(cpp_type, indent_increment, field.name))
            s.write('    }\n')
        else:
            s.write('    s << indent << "%s: ";\n'%field.name)
            indent_increment = '  '
            if (not field.is_builtin or field.is_array):
                s.write('s << std::endl;\n')
            s.write('    Printer<%s>::stream(s, indent + "%s", v.%s);\n'%(cpp_type, indent_increment, field.name))
    s.write('  }\n')
    s.write('};\n\n')
        
    s.write('\n')
        
    s.write('} // namespace message_operations\n')
    s.write('} // namespace ros\n\n')
    
    
def write_ostream_operator(s, spec, cpp_name_prefix):
    (cpp_msg_unqualified, cpp_msg_with_alloc, _) = cpp_message_declarations(cpp_name_prefix, spec.short_name)
    s.write('template<typename ContainerAllocator>\nstd::ostream& operator<<(std::ostream& s, const %s& v)\n{\n'%(cpp_msg_with_alloc))
    s.write('  ros::message_operations::Printer<%s>::stream(s, "", v);\n  return s;}\n\n'%(cpp_msg_with_alloc))

def write_serialize_length(s, v, is_array=False):
    #t2
    var = '__ros_arr_len' if is_array else '__ros_str_len'
    
    s.write('(let ((%s (length %s)))'%(var, v))
    with Indent(s):
        for x in range(0, 32, 8):
            s.write('(write-byte (ldb (byte 8 %s) %s) ostream)'%(x, var))
    s.write(')', indent=False)


def write_serialize_bits(s, v, num_bytes): 
    for x in range(0, num_bytes*8, 8):
        s.write('(write-byte (ldb (byte 8 %s) %s) ostream)'%(x, v))

# t2: can get rid of this lookup_slot stuff        
def write_serialize_builtin(s, f, var='msg', lookup_slot=True):
    v = '(slot-value %s \'%s)'%(var, f.name) if lookup_slot else var
    if f.base_type == 'string':
        write_serialize_length(s, v)
        s.write('(map nil #\'(lambda (c) (write-byte (char-code c) ostream)) %s)'%v)
    elif f.base_type == 'float32':
        s.write('(let ((bits %s))'%'(roslisp-utils:encode-single-float-bits %s)'%v)
        with Indent(s):
            write_serialize_bits(s, 'bits', 4)
        s.write(')', False)
    elif f.base_type == 'float64':
        s.write('(let ((bits %s))'%'(roslisp-utils:encode-double-float-bits %s)'%v)
        with Indent(s):
            write_serialize_bits(s, 'bits', 8)
        s.write(')', False)
    elif f.base_type == 'bool':
        s.write('(write-byte (ldb (byte 8 0) (if %s 1 0)) ostream)'%v)
    elif f.base_type in ['byte', 'char']:
        s.write('(write-byte (ldb (byte 8 0) %s) ostream)'%v)
    elif f.base_type in ['duration', 'time']:
        s.write('(let ((__sec (floor %s))'%v)
        s.write('      (__nsec (round (* 1e9 (- %s (floor %s))))))'%(v,v))
        with Indent(s):
            write_serialize_bits(s, '__sec', 4)
            write_serialize_bits(s, '__nsec', 4)
            s.write(')', False)
    elif is_signed_int(f.base_type):
        write_serialize_bits(s, v, NUM_BYTES[f.base_type])
    elif is_unsigned_int(f.base_type):
        write_serialize_bits(s, v, NUM_BYTES[f.base_type])        
    else:
        raise ValueError('Unknown type: %s', f.base_type)
    
def write_serialize_field(s, f):
    slot = '(slot-value msg \'%s)'%f.name
    if f.is_array:
        if not f.array_len:
            write_serialize_length(s, slot, True)
        s.write('(map nil #\'(lambda (ele) ')
        var = 'ele'
        s.block_next_indent()
        lookup_slot = False
    else:
        var='msg'
        lookup_slot = True

    if f.is_builtin:
        write_serialize_builtin(s, f, var, lookup_slot=lookup_slot)
    else:
        to_write = slot if lookup_slot else var #t2
        s.write('(serialize %s ostream)'%to_write)

    if f.is_array:
        s.write(')', False)
        s.write(' %s)'%slot)
    
def write_serialize(s, spec):
    """
    Write the serialize method
    """
    s.write('(defmethod serialize ((msg %s) ostream)'%message_class(spec))
    with Indent(s):
        s.write('"Serializes a message object of type \'%s"'%message_class(spec))
        for f in spec.parsed_fields():
            write_serialize_field(s, f)
    s.write(')')
    

# t2 can get rid of is_array
def write_deserialize_length(s, is_array=False):
    var = '__ros_arr_len' if is_array else '__ros_str_len'
    s.write('(let ((%s 0))'%var)
    with Indent(s):
        for x in range(0, 32, 8):
            s.write('(setf (ldb (byte 8 %s) %s) (read-byte istream))'%(x, var))

def write_deserialize_bits(s, v, num_bytes):
    for x in range(0, num_bytes*8, 8):
        s.write('(setf (ldb (byte 8 %s) %s) (read-byte istream))'%(x, v))
        
    
    
def write_deserialize_builtin(s, f, v):
    # t0 possibly write setf once
    if f.base_type == 'string':
        write_deserialize_length(s)
        with Indent(s):
            s.write('(setf %s (make-string __ros_str_len))'%v)
            s.write('(dotimes (__ros_str_idx __ros_str_len msg)')
            with Indent(s):
                s.write('(setf (char %s __ros_str_idx) (code-char (read-byte istream)))))'%v)
    elif f.base_type == 'float32':
        s.write('(let ((bits 0))')
        with Indent(s):
            write_deserialize_bits(s, 'bits', 4)
        s.write('(setf %s (roslisp-utils:decode-single-float-bits bits)))'%v)
    elif f.base_type == 'float64':
        s.write('(let ((bits 0))')
        with Indent(s):
            write_deserialize_bits(s, 'bits', 8)
        s.write('(setf %s (roslisp-utils:decode-double-float-bits bits)))'%v)
    elif f.base_type == 'bool':
        s.write('(setf %s (not (zerop (read-byte istream))))'%v)
    elif f.base_type in ['byte', 'char']:
        s.write('(setf (ldb (byte 8 0) %s) (read-byte istream))'%v)
    elif f.base_type in ['duration', 'time']:
        s.write('(let ((__sec 0) (__nsec 0))')
        with Indent(s):
            write_deserialize_bits(s, '__sec', 4)
            write_deserialize_bits(s, '__nsec', 4)
            s.write('(setf %s (+ (coerce __sec \'double-float) (/ __nsec 1e9))))'%v)
    elif is_signed_int(f.base_type):
        write_deserialize_bits(s, v, NUM_BYTES[f.base_type])
    elif is_unsigned_int(f.base_type):
        write_deserialize_bits(s, v, NUM_BYTES[f.base_type])
    else:
        raise ValueError('%s unknown'%f.base_type)


def write_deserialize_field(s, f, pkg):
    slot = '(slot-value msg \'%s)'%f.name
    var = slot
    if f.is_array:
        if not f.array_len:
            write_deserialize_length(s, True)
            length = '__ros_arr_len'
        else:
            length = '%s'%f.array_len
            
        s.write('(setf %s (make-array %s))'%(slot, length))
        s.write('(let ((vals %s))'%slot) # t2
        var = '(aref vals i)'
        with Indent(s):
            s.write('(dotimes (i %s)'%length)

    if f.is_builtin:
        write_deserialize_builtin(s, f, var)
    else:
        if f.is_array:
            s.write('(setf %s (make-instance \'%s))'%(var, msg_type(f)))
        s.write('(deserialize %s istream)'%var)

    if f.is_array:
        s.write('))', False)
        if not f.array_len:
            s.write(')', False)


def write_deserialize(s, spec):
    """
    Write the deserialize method
    """
    s.write('(defmethod deserialize ((msg %s) istream)'%message_class(spec))
    with Indent(s):
        s.write('"Deserializes a message object of type \'%s"'%message_class(spec))
        for f in spec.parsed_fields():
            write_deserialize_field(s, f, spec.package)
        s.write('msg')
    s.write(')')

def write_ros_datatype(s, spec):
    c = message_class(spec)
    s.write('(defmethod ros-datatype ((msg (eql \'%s)))'%c)
    with Indent(s):
        s.write('"Returns string type for a message object of type \'%s"'%c)
        s.write('"%s")'%spec.full_name)

def write_md5sum(s, spec):
    gendeps_dict = roslib.gentools.get_dependencies(spec, spec.package,
                                                    compute_files=False)
    md5sum = roslib.gentools.compute_md5(gendeps_dict)
    c = message_class(spec)
    s.write('(defmethod md5sum ((type (eql \'%s)))'%c)
    with Indent(s):
        s.write('"Returns md5sum for a message object of type \'%s"'%c)
        s.write('"%s")'%md5sum)

def write_message_definition(s, spec):
    c = message_class(spec)
    s.write('(defmethod message-definition ((type (eql \'%s)))'%c)
    with Indent(s):
        s.write('"Returns full string definition for message of type \'%s"'%c)
        s.write('(format nil "')
        gendeps_dict = roslib.gentools.get_dependencies(spec, spec.package, compute_files=False)
        definition = roslib.gentools.compute_full_text(gendeps_dict)
        lines = definition.split('\n')
        for line in lines:
            l = line.replace('\\', '\\\\')
            l = l.replace('"', '\\"')
            s.write('%s~%%'%l, indent=False)
        s.write('~%', indent=False)
        s.write('"))', indent=False)

def write_builtin_length(s, f, var='msg'):
    if f.base_type in ['int8', 'uint8']:
        s.write('1')
    elif f.base_type in ['int16', 'uint16']:
        s.write('2')
    elif f.base_type in ['int32', 'uint32', 'float32']:
        s.write('4')
    elif f.base_type in ['int64', 'uint64', 'float64', 'duration', 'time']:
        s.write('8')
    elif f.base_type == 'string':
        s.write('4 (length %s)'%var)
    elif f.base_type in ['bool', 'byte', 'char']:
        s.write('1')
    else:
        raise ValueError('Unknown: %s', f.base_type)

def write_serialization_length(s, spec):
    c = message_class(spec)
    s.write('(defmethod serialization-length ((msg %s))'%c)
    with Indent(s):
        s.write('(+ 0')
        with Indent(s, 3):
            for field in spec.parsed_fields():
                slot = '(slot-value msg \'%s)'%field.name
                if field.is_array:
                    l = '0' if field.array_len else '4'
                    s.write('%s (reduce #\'+ %s :key #\'(lambda (ele) (declare (ignorable ele)) (+ '%(l, slot))
                    var = 'ele'
                    s.block_next_indent()
                else:
                    var = slot
                    
                if field.is_builtin:
                    write_builtin_length(s, field, var)
                else:
                    s.write('(serialization-length %s)'%var)

                if field.is_array:
                    s.write(')))', False)
    s.write('))')


def write_list_converter(s, spec):
    c = message_class(spec)
    s.write('(defmethod ros-message-to-list ((msg %s))'%c)
    with Indent(s):
        s.write('"Converts a ROS message object to a list"')
        s.write('(list \'%s'%c)
        with Indent(s):
            for f in spec.parsed_fields():
                s.write('(cons \':%s (%s-val msg))'%(f.name, f.name))
    s.write('))')

def generate(msg_path):
    """
    Generate a message
    
    @param msg_path: The path to the .msg file
    @type msg_path: str
    """
    (package_dir, package) = roslib.packages.get_dir_pkg(msg_path)
    (_, spec) = roslib.msgs.load_from_file(msg_path, package)
    
    io = StringIO()
    s =  IndentedWriter(io)
    write_begin(s, spec, msg_path)
    write_defclass(s, spec, package)
    write_serialize(s, spec)
    write_deserialize(s, spec)
    write_ros_datatype(s, spec)
    write_md5sum(s, spec)
    write_message_definition(s, spec)
    write_serialization_length(s, spec)
    write_list_converter(s, spec)
    
    output_dir = '%s/msg_gen/lisp'%package_dir
    if (not os.path.exists(output_dir)):
        # if we're being run concurrently, the above test can report false but os.makedirs can still fail if
        # another copy just created the directory
        try:
            os.makedirs(output_dir)
        except OSError, e:
            pass
         
    f = open('%s/%s.lisp'%(output_dir, spec.short_name), 'w')
    print >> f, io.getvalue()
    
    io.close()

if __name__ == "__main__":
    roslib.msgs.set_verbose(False)
    generate(sys.argv[1])

