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
## Converts ROS .msg and .srv files in a package into Lisp source code

## t0: needed for script to work
## t1: for reference; remove once running
## t2: can be changed once we remove strict diff-compatibility requirement with old version of genmsg_lisp

import sys
import os
import traceback
import re

import roslib.msgs
import roslib.srvs
import roslib.packages
import roslib.gentools
from roslib.msgs import MsgSpec
from roslib.srvs import SrvSpec

try:
    from cStringIO import StringIO #Python 2.x
except ImportError:
    from io import StringIO #Python 3.x

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
        return '(cl:vector %s)'%elt_type
    else:
        return elt_type

def parse_msg_type(f):
    if f.base_type == 'Header':
        return ('std_msgs', 'Header')
    else:
        return f.base_type.split('/')

# t2 no need for is_array    
def msg_type(f):
    (pkg, msg) = parse_msg_type(f)
    return '%s-msg:%s'%(pkg, msg)

def lisp_type(t):
    if is_fixnum(t):
        return 'cl:fixnum'
    elif is_integer(t):
        return 'cl:integer'
    elif is_bool(t):
        return 'cl:boolean'
    elif is_float(t):
        return 'cl:float'
    elif is_time(t):
        return 'cl:real'
    elif is_string(t):
        return 'cl:string'
    else:
        raise ValueError('%s is not a recognized primitive type'%t)

def field_initform(f):
    if f.is_builtin:
        initform = lisp_initform(f.base_type)
        elt_type = lisp_type(f.base_type)
    else:
        initform = '(cl:make-instance \'%s)'%msg_type(f)
        elt_type = msg_type(f)
    if f.is_array:
        len = f.array_len or 0
        return '(cl:make-array %s :element-type \'%s :initial-element %s)'%(len, elt_type, initform)
    else:
        return initform

def lisp_initform(t):
    if is_integer(t):
        return '0'
    elif is_bool(t):
        return 'cl:nil'
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
                for i in range(self.indentation):
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

    

def write_begin(s, spec, path, is_service=False):
    "Writes the beginning of the file: a comment saying it's auto-generated and the in-package form"
    
    s.write('; Auto-generated. Do not edit!\n\n\n', newline=False)
    suffix = 'srv' if is_service else 'msg'
    s.write('(cl:in-package %s-%s)\n\n\n'%(spec.package, suffix), newline=False)

def write_html_include(s, spec, is_srv=False):
    
    s.write(';//! \\htmlinclude %s.msg.html\n'%spec.actual_name, newline=False) # t2

def write_slot_definition(s, field):
    "Write the definition of a slot corresponding to a single message field"

    s.write('(%s'%field.name)
    with Indent(s, 1):
        s.write(':reader %s'%field.name)
        s.write(':initarg :%s'%field.name)
        s.write(':type %s'%field_type(field))
    i = 0 if field.is_array else 1 # t2
    with Indent(s, i):
        s.write(':initform %s)'%field_initform(field))

def write_deprecated_readers(s, spec):
    suffix = 'srv' if spec.component_type == 'service' else 'msg'
    for field in spec.parsed_fields():
        s.newline()
        s.write('(cl:ensure-generic-function \'%s-val :lambda-list \'(m))' % field.name)
        s.write('(cl:defmethod %s-val ((m %s))'%(field.name, message_class(spec)))
        with Indent(s):
            s.write('(roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader %s-%s:%s-val is deprecated.  Use %s-%s:%s instead.")'%(spec.package, suffix, field.name, spec.package, suffix, field.name))
            s.write('(%s m))'%field.name)
            
            

def write_defclass(s, spec):
    "Writes the defclass that defines the message type"
    cl = message_class(spec)
    new_cl = new_message_class(spec)
    suffix = 'srv' if spec.component_type == 'service' else 'msg'
    s.write('(cl:defclass %s (roslisp-msg-protocol:ros-message)'%cl)
    with Indent(s):
        s.write('(')
        with Indent(s, inc=1, indent_first=False):
            for field in spec.parsed_fields():
                write_slot_definition(s, field)
        s.write(')', indent=False)
    s.write(')')
    s.newline()
    s.write('(cl:defclass %s (%s)'%(new_cl, cl))
    with Indent(s):
        s.write('())')
    s.newline()
    s.write('(cl:defmethod cl:initialize-instance :after ((m %s) cl:&rest args)'%cl)
    with Indent(s):
        s.write('(cl:declare (cl:ignorable args))')
        s.write('(cl:unless (cl:typep m \'%s)'%new_cl)
        with Indent(s):
            s.write('(roslisp-msg-protocol:msg-deprecation-warning "using old message class name %s-%s:%s is deprecated: use %s-%s:%s instead.")))'%(spec.package, suffix, cl, spec.package, suffix, new_cl))
    
    

def message_class(spec):
    """
    Return the CLOS class name for this message type
    """
    return '<%s>'%spec.actual_name

def new_message_class(spec):
    return spec.actual_name

    
def write_serialize_length(s, v, is_array=False):
    #t2
    var = '__ros_arr_len' if is_array else '__ros_str_len'
    
    s.write('(cl:let ((%s (cl:length %s)))'%(var, v))
    with Indent(s):
        for x in range(0, 32, 8):
            s.write('(cl:write-byte (cl:ldb (cl:byte 8 %s) %s) ostream)'%(x, var))
    s.write(')', indent=False)


def write_serialize_bits(s, v, num_bytes): 
    for x in range(0, num_bytes*8, 8):
        s.write('(cl:write-byte (cl:ldb (cl:byte 8 %s) %s) ostream)'%(x, v))

def write_serialize_bits_signed(s, v, num_bytes):
    num_bits = num_bytes*8
    s.write('(cl:let* ((signed %s) (unsigned (cl:if (cl:< signed 0) (cl:+ signed %s) signed)))'%(v, 2**num_bits))
    with Indent(s):
        write_serialize_bits(s, 'unsigned', num_bytes)
        s.write(')')



# t2: can get rid of this lookup_slot stuff        
def write_serialize_builtin(s, f, var='msg', lookup_slot=True):
    v = '(cl:slot-value %s \'%s)'%(var, f.name) if lookup_slot else var
    if f.base_type == 'string':
        write_serialize_length(s, v)
        s.write('(cl:map cl:nil #\'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) %s)'%v)
    elif f.base_type == 'float32':
        s.write('(cl:let ((bits %s))'%'(roslisp-utils:encode-single-float-bits %s)'%v)
        with Indent(s):
            write_serialize_bits(s, 'bits', 4)
        s.write(')', False)
    elif f.base_type == 'float64':
        s.write('(cl:let ((bits %s))'%'(roslisp-utils:encode-double-float-bits %s)'%v)
        with Indent(s):
            write_serialize_bits(s, 'bits', 8)
        s.write(')', False)
    elif f.base_type == 'bool':
        s.write('(cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if %s 1 0)) ostream)'%v)
    elif f.base_type in ['byte', 'char']:
        s.write('(cl:write-byte (cl:ldb (cl:byte 8 0) %s) ostream)'%v)
    elif f.base_type in ['duration', 'time']:
        s.write('(cl:let ((__sec (cl:floor %s))'%v)
        s.write('      (__nsec (cl:round (cl:* 1e9 (cl:- %s (cl:floor %s))))))'%(v,v))
        with Indent(s):
            write_serialize_bits(s, '__sec', 4)
            write_serialize_bits(s, '__nsec', 4)
            s.write(')', False)
    elif is_signed_int(f.base_type):
        write_serialize_bits_signed(s, v, NUM_BYTES[f.base_type])
    elif is_unsigned_int(f.base_type):
        write_serialize_bits(s, v, NUM_BYTES[f.base_type])        
    else:
        raise ValueError('Unknown type: %s', f.base_type)
    
def write_serialize_field(s, f):
    slot = '(cl:slot-value msg \'%s)'%f.name
    if f.is_array:
        if not f.array_len:
            write_serialize_length(s, slot, True)
        s.write('(cl:map cl:nil #\'(cl:lambda (ele) ')
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
        s.write('(roslisp-msg-protocol:serialize %s ostream)'%to_write)

    if f.is_array:
        s.write(')', False)
        s.write(' %s)'%slot)
    
def write_serialize(s, spec):
    """
    Write the serialize method
    """
    s.write('(cl:defmethod roslisp-msg-protocol:serialize ((msg %s) ostream)'%message_class(spec))
    with Indent(s):
        s.write('"Serializes a message object of type \'%s"'%message_class(spec))
        for f in spec.parsed_fields():
            write_serialize_field(s, f)
    s.write(')')
    

# t2 can get rid of is_array
def write_deserialize_length(s, is_array=False):
    var = '__ros_arr_len' if is_array else '__ros_str_len'
    s.write('(cl:let ((%s 0))'%var)
    with Indent(s):
        for x in range(0, 32, 8):
            s.write('(cl:setf (cl:ldb (cl:byte 8 %s) %s) (cl:read-byte istream))'%(x, var))

def write_deserialize_bits(s, v, num_bytes):
    for x in range(0, num_bytes*8, 8):
        s.write('(cl:setf (cl:ldb (cl:byte 8 %s) %s) (cl:read-byte istream))'%(x, v))

def write_deserialize_bits_signed(s, v, num_bytes):
    s.write('(cl:let ((unsigned 0))')
    num_bits = 8*num_bytes
    with Indent(s):
        write_deserialize_bits(s, 'unsigned', num_bytes)
        s.write('(cl:setf %s (cl:if (cl:< unsigned %s) unsigned (cl:- unsigned %s))))'%(v, 2**(num_bits-1), 2**num_bits))

    
    
def write_deserialize_builtin(s, f, v):
    if f.base_type == 'string':
        write_deserialize_length(s)
        with Indent(s):
            s.write('(cl:setf %s (cl:make-string __ros_str_len))'%v)
            s.write('(cl:dotimes (__ros_str_idx __ros_str_len msg)')
            with Indent(s):
                s.write('(cl:setf (cl:char %s __ros_str_idx) (cl:code-char (cl:read-byte istream)))))'%v)
    elif f.base_type == 'float32':
        s.write('(cl:let ((bits 0))')
        with Indent(s):
            write_deserialize_bits(s, 'bits', 4)
        s.write('(cl:setf %s (roslisp-utils:decode-single-float-bits bits)))'%v)
    elif f.base_type == 'float64':
        s.write('(cl:let ((bits 0))')
        with Indent(s):
            write_deserialize_bits(s, 'bits', 8)
        s.write('(cl:setf %s (roslisp-utils:decode-double-float-bits bits)))'%v)
    elif f.base_type == 'bool':
        s.write('(cl:setf %s (cl:not (cl:zerop (cl:read-byte istream))))'%v)
    elif f.base_type in ['byte', 'char']:
        s.write('(cl:setf (cl:ldb (cl:byte 8 0) %s) (cl:read-byte istream))'%v)
    elif f.base_type in ['duration', 'time']:
        s.write('(cl:let ((__sec 0) (__nsec 0))')
        with Indent(s):
            write_deserialize_bits(s, '__sec', 4)
            write_deserialize_bits(s, '__nsec', 4)
            s.write('(cl:setf %s (cl:+ (cl:coerce __sec \'cl:double-float) (cl:/ __nsec 1e9))))'%v)
    elif is_signed_int(f.base_type):
        write_deserialize_bits_signed(s, v, NUM_BYTES[f.base_type])
    elif is_unsigned_int(f.base_type):
        write_deserialize_bits(s, v, NUM_BYTES[f.base_type])
    else:
        raise ValueError('%s unknown'%f.base_type)


def write_deserialize_field(s, f, pkg):
    slot = '(cl:slot-value msg \'%s)'%f.name
    var = slot
    if f.is_array:
        if not f.array_len:
            write_deserialize_length(s, True)
            length = '__ros_arr_len'
        else:
            length = '%s'%f.array_len
            
        s.write('(cl:setf %s (cl:make-array %s))'%(slot, length))
        s.write('(cl:let ((vals %s))'%slot) # t2
        var = '(cl:aref vals i)'
        with Indent(s):
            s.write('(cl:dotimes (i %s)'%length)

    if f.is_builtin:
        with Indent(s):
            write_deserialize_builtin(s, f, var)
    else:
        if f.is_array:
            with Indent(s):
                s.write('(cl:setf %s (cl:make-instance \'%s))'%(var, msg_type(f)))
        s.write('(roslisp-msg-protocol:deserialize %s istream)'%var)

    if f.is_array:
        s.write('))', False)
        if not f.array_len:
            s.write(')', False)


def write_deserialize(s, spec):
    """
    Write the deserialize method
    """
    s.write('(cl:defmethod roslisp-msg-protocol:deserialize ((msg %s) istream)'%message_class(spec))
    with Indent(s):
        s.write('"Deserializes a message object of type \'%s"'%message_class(spec))
        for f in spec.parsed_fields():
            write_deserialize_field(s, f, spec.package)
        s.write('msg')
    s.write(')')

def write_class_exports(s, pkg):
    "Write the _package.lisp file"
    s.write('(cl:defpackage %s-msg'%pkg, False)
    with Indent(s):
        s.write('(:use )')
        s.write('(:export')
        with Indent(s, inc=1):
            for spec in roslib.msgs.get_pkg_msg_specs(pkg)[0]:
                (p, msg_type) = spec[0].split('/')
                msg_class = '<%s>'%msg_type
                s.write('"%s"'%msg_class.upper())
                s.write('"%s"'%msg_type.upper())
        s.write('))\n\n')

def write_srv_exports(s, pkg):
    "Write the _package.lisp file for a service directory"
    s.write('(cl:defpackage %s-srv'%pkg, False)
    with Indent(s):
        s.write('(:use )')
        s.write('(:export')
        with Indent(s, inc=1):
            for spec in roslib.srvs.get_pkg_srv_specs(pkg)[0]:
                (_, srv_type) = spec[0].split('/')
                s.write('"%s"'%srv_type.upper())
                s.write('"<%s-REQUEST>"'%srv_type.upper())
                s.write('"%s-REQUEST"'%srv_type.upper())
                s.write('"<%s-RESPONSE>"'%srv_type.upper())
                s.write('"%s-RESPONSE"'%srv_type.upper())
        s.write('))\n\n')


def write_asd_deps(s, deps, msgs):
    with Indent(s):
        s.write(':depends-on (:roslisp-msg-protocol :roslisp-utils ')
        with Indent(s, inc=13, indent_first=False):
            for d in sorted(deps):
                s.write(':%s-msg'%d)
    s.write(')') #t2 indentation
    with Indent(s):
        s.write(':components ((:file "_package")')
        with Indent(s):
            for (full_name, _) in msgs:
                (_, name) = full_name.split('/')
                s.write('(:file "%s" :depends-on ("_package_%s"))'%(name, name))
                s.write('(:file "_package_%s" :depends-on ("_package"))'%name)
        s.write('))')
                


def write_srv_asd(s, pkg):
    s.write('(cl:in-package :asdf)')
    s.newline()
    s.write('(defsystem "%s-srv"'%pkg)
    services = roslib.srvs.get_pkg_srv_specs(pkg)[0]

    # Figure out set of depended-upon ros packages
    deps = set()
    for (_, spec) in services:
        for f in spec.request.parsed_fields():
            if not f.is_builtin:
                (p, _) = parse_msg_type(f)
                deps.add(p)
        for f in spec.response.parsed_fields():
            if not f.is_builtin:
                (p, _) = parse_msg_type(f)
                deps.add(p)

    write_asd_deps(s, deps, services)


def write_asd(s, pkg):
    s.write('(cl:in-package :asdf)')
    s.newline()
    s.write('(defsystem "%s-msg"'%pkg)
    msgs = roslib.msgs.get_pkg_msg_specs(pkg)[0]

    # Figure out set of depended-upon ros packages
    deps = set()
    for (_, spec) in msgs:
        for f in spec.parsed_fields():
            if not f.is_builtin:
                (p, _) = parse_msg_type(f)
                deps.add(p)
    if pkg in deps:
        deps.remove(pkg)
    write_asd_deps(s, deps, msgs)   

def write_accessor_exports(s, spec):
    "Write the package exports for this message/service"
    is_srv = isinstance(spec, SrvSpec)
    suffix = 'srv' if is_srv else 'msg'
    s.write('(cl:in-package %s-%s)'%(spec.package, suffix), indent=False)
    s.write('(cl:export \'(')
    if is_srv:
        fields = spec.request.parsed_fields()[:]
        fields.extend(spec.response.parsed_fields())
    else:
        fields = spec.parsed_fields()
    
    with Indent(s, inc=10, indent_first=False):
        for f in fields:
            accessor = '%s-val'%f.name
            s.write('%s'%accessor.upper())
            s.write('%s'%f.name.upper())
    s.write('))')


def write_ros_datatype(s, spec):
    for c in (message_class(spec), new_message_class(spec)):
        s.write('(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql \'%s)))'%c)
        with Indent(s):
            s.write('"Returns string type for a %s object of type \'%s"'%(spec.component_type, c))
            s.write('"%s")'%spec.full_name)

def write_md5sum(s, spec, parent=None):
    if parent is None:
        parent = spec
    gendeps_dict = roslib.gentools.get_dependencies(parent, spec.package,
                                                    compute_files=False)
    md5sum = roslib.gentools.compute_md5(gendeps_dict)
    for c in (message_class(spec), new_message_class(spec)):
        s.write('(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql \'%s)))'%c)
        with Indent(s):
            # t2 this should print 'service' instead of 'message' if it's a service request or response
            s.write('"Returns md5sum for a message object of type \'%s"'%c)
            s.write('"%s")'%md5sum)

def write_message_definition(s, spec):
    for c in (message_class(spec), new_message_class(spec)):
        s.write('(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql \'%s)))'%c)
        with Indent(s):
            s.write('"Returns full string definition for message of type \'%s"'%c)
            s.write('(cl:format cl:nil "')
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
        s.write('4 (cl:length %s)'%var)
    elif f.base_type in ['bool', 'byte', 'char']:
        s.write('1')
    else:
        raise ValueError('Unknown: %s', f.base_type)

def write_serialization_length(s, spec):
    c = message_class(spec)
    s.write('(cl:defmethod roslisp-msg-protocol:serialization-length ((msg %s))'%c)
    with Indent(s):
        s.write('(cl:+ 0')
        with Indent(s, 3):
            for field in spec.parsed_fields():
                slot = '(cl:slot-value msg \'%s)'%field.name
                if field.is_array:
                    l = '0' if field.array_len else '4'
                    s.write('%s (cl:reduce #\'cl:+ %s :key #\'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ '%(l, slot))
                    var = 'ele'
                    s.block_next_indent()
                else:
                    var = slot
                    
                if field.is_builtin:
                    write_builtin_length(s, field, var)
                else:
                    s.write('(roslisp-msg-protocol:serialization-length %s)'%var)

                if field.is_array:
                    s.write(')))', False)
    s.write('))')


def write_list_converter(s, spec):
    c = message_class(spec)
    s.write('(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg %s))'%c)
    with Indent(s):
        s.write('"Converts a ROS message object to a list"')
        s.write('(cl:list \'%s'%new_message_class(spec))
        with Indent(s):
            for f in spec.parsed_fields():
                s.write('(cl:cons \':%s (%s msg))'%(f.name, f.name))
    s.write('))')

def write_constants(s, spec):
    if spec.constants:
        for cls in (message_class(spec), new_message_class(spec)):
            s.write('(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql \'%s)))'%cls)
            with Indent(s):
                s.write('  "Constants for message type \'%s"'%cls)
                s.write('\'(')
                with Indent(s, indent_first=False):
                    for c in spec.constants:
                        s.write('(:%s . %s)'%(c.name.upper(), c.val))
            s.write(')', False)
            s.write(')')


def write_srv_component(s, spec, parent):
    spec.component_type='service'
    write_html_include(s, spec)
    write_defclass(s, spec)
    write_deprecated_readers(s, spec)
    write_constants(s, spec)
    write_serialize(s, spec)
    write_deserialize(s, spec)
    write_ros_datatype(s, spec)
    write_md5sum(s, spec, parent)
    write_message_definition(s, spec)
    write_serialization_length(s, spec)
    write_list_converter(s, spec)


def write_service_specific_methods(s, spec):
    spec.actual_name=spec.short_name
    s.write('(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql \'%s)))'%spec.short_name)
    with Indent(s):
        s.write('\'%s)'%new_message_class(spec.request))
    s.write('(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql \'%s)))'%spec.short_name)
    with Indent(s):
        s.write('\'%s)'%new_message_class(spec.response))
    s.write('(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql \'%s)))'%spec.short_name)
    with Indent(s):
        s.write('"Returns string type for a service object of type \'%s"'%message_class(spec))
        s.write('"%s")'%spec.full_name)


def generate_msg(msg_path):
    """
    Generate a message
    
    @param msg_path: The path to the .msg file
    @type msg_path: str
    """
    (package_dir, package) = roslib.packages.get_dir_pkg(msg_path)
    (_, spec) = roslib.msgs.load_from_file(msg_path, package)
    spec.actual_name=spec.short_name
    spec.component_type='message'

    ########################################
    # 1. Write the .lisp file
    ########################################
    
    io = StringIO()
    s =  IndentedWriter(io)
    write_begin(s, spec, msg_path)
    write_html_include(s, spec)
    write_defclass(s, spec)
    write_deprecated_readers(s, spec)
    write_constants(s, spec)
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
        except OSError as e:
            pass

    with open('%s/%s.lisp'%(output_dir, spec.short_name), 'w') as f:
        f.write(io.getvalue() + "\n")
    io.close()

    ########################################
    # 2. Write the _package file
    # for this message
    ########################################

    io = StringIO()
    s = IndentedWriter(io)
    write_accessor_exports(s, spec)
    with open('%s/_package_%s.lisp'%(output_dir, spec.short_name), 'w') as f:
        f.write(io.getvalue())
    io.close()

    ########################################
    # 3. Write the _package.lisp file
    # This is being rewritten once per msg
    # file, which is inefficient
    ########################################

    io = StringIO()
    s = IndentedWriter(io)
    write_class_exports(s, package)
    with open('%s/_package.lisp'%output_dir, 'w') as f:
        f.write(io.getvalue())
    io.close()

    ########################################
    # 4. Write the .asd file
    # This is being written once per msg
    # file, which is inefficient
    ########################################

    io = StringIO()
    s = IndentedWriter(io)
    write_asd(s, package)
    with open('%s/%s-msg.asd'%(output_dir, package), 'w') as f:
        f.write(io.getvalue())
    io.close()

# t0 most of this could probably be refactored into being shared with messages
def generate_srv(srv_path):
    "Generate code from .srv file"
    (pkg_dir, pkg) = roslib.packages.get_dir_pkg(srv_path)
    (_, spec) = roslib.srvs.load_from_file(srv_path, pkg)
    output_dir = '%s/srv_gen/lisp'%pkg_dir
    if (not os.path.exists(output_dir)):
        # if we're being run concurrently, the above test can report false but os.makedirs can still fail if
        # another copy just created the directory
        try:
            os.makedirs(output_dir)
        except OSError as e:
            pass

    ########################################
    # 1. Write the .lisp file
    ########################################

    io = StringIO()
    s = IndentedWriter(io)
    write_begin(s, spec, srv_path, True)
    spec.request.actual_name='%s-request'%spec.short_name
    spec.response.actual_name='%s-response'%spec.short_name
    write_srv_component(s, spec.request, spec)
    s.newline()
    write_srv_component(s, spec.response, spec)
    write_service_specific_methods(s, spec)
    
    with open('%s/%s.lisp'%(output_dir, spec.short_name), 'w') as f:
        f.write(io.getvalue())
    io.close()

    ########################################
    # 2. Write the _package file
    # for this service
    ########################################

    io = StringIO()
    s = IndentedWriter(io)
    write_accessor_exports(s, spec)
    with open('%s/_package_%s.lisp'%(output_dir, spec.short_name), 'w') as f:
        f.write(io.getvalue())
    io.close()

    ########################################
    # 3. Write the _package.lisp file
    ########################################

    io = StringIO()
    s = IndentedWriter(io)
    write_srv_exports(s, pkg)
    with open('%s/_package.lisp'%output_dir, 'w') as f:
        f.write(io.getvalue())
    io.close()

    ########################################
    # 4. Write the .asd file
    ########################################

    io = StringIO()
    s = IndentedWriter(io)
    write_srv_asd(s, pkg)
    with open('%s/%s-srv.asd'%(output_dir, pkg), 'w') as f:
        f.write(io.getvalue())
    io.close()
    

         

if __name__ == "__main__":
    roslib.msgs.set_verbose(False)
    if sys.argv[1].endswith('.msg'):
        generate_msg(sys.argv[1])
    elif sys.argv[1].endswith('.srv'):
        generate_srv(sys.argv[1])
    else:
        raise ValueError('Invalid filename %s'%sys.argv[1])
