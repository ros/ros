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
# Revision $Id: message.py 3357 2009-01-13 07:13:05Z jfaustwg $

import cStringIO
import math
import struct
import types

from roslib.rostime import Time, Duration
import roslib.genpy

class ROSMessageException(Exception): pass

# we expose the generic message-strify routine for fn-oriented code like rostopic

## convert value to string representation
## @param val Value to convert to string representation. Most likely a Message.
def strify_message(val, indent=''):
    if type(val) in [int, long, float, str, bool] or \
            isinstance(val, Time) or isinstance(val, Duration):        
        return str(val)
    elif type(val) in [list, tuple]:
        if len(val) == 0:
            return str(val)
        val0 = val[0]
        if type(val0) in [int, float, str, bool] or \
               isinstance(val0, Time) or isinstance(val0, Duration) or \
               type(val0) in [list, tuple]: # no array-of-arrays support yet
            return str(val)
        else:
            indent = indent + '  '
            return "["+','.join([strify_message(v, indent) for v in val])+"]"
    elif isinstance(val, Message):
        if indent:
            return '\n'+'\n'.join(['%s%s: %s'%(indent, f, strify_message(getattr(val, f), '  '+indent)) for f in val.__slots__])
        return '\n'.join(['%s%s: %s'%(indent, f, strify_message(getattr(val, f), '  '+indent)) for f in val.__slots__])
    else:
        return str(val) #punt

# check_type mildly violates some abstraction boundaries between .msg
# representation and the python Message representation. The
# alternative is to have the message generator map .msg types to
# python types beforehand, but that would make it harder to do
# width/signed checks.

_widths = {
    'byte': 8, 'char': 8, 'int8': 8, 'uint8': 8,
    'int16': 16, 'uint16': 16, 
    'int32': 32, 'uint32': 32, 
    'int64': 64, 'uint64': 64, 
}

## Dynamic type checker that maps ROS .msg types to python types and
## verifies the python value.  check_type() is not designed to be fast
## and is targeted at error diagnosis.
## @param field_name str: ROS .msg field name
## @param field_type str: ROS .msg field type
## @param field_val Any: field value
## @throws SerializationError if typecheck fails
def check_type(field_name, field_type, field_val):
    if roslib.genpy.is_simple(field_type):
        # check sign and width
        if field_type in ['byte', 'int8', 'int16', 'int32', 'int64']:
            if type(field_val) not in [long, int]:
                raise SerializationError('field [%s] must be an integer type'%field_name)
            maxval = int(math.pow(2, _widths[field_type]-1))
            if field_val >= maxval or field_val <= -maxval:
                raise SerializationError('field [%s] exceeds specified width [%s]'%(field_name, field_type))
        elif field_type in ['char', 'uint8', 'uint16', 'uint32', 'uint64']:
            if type(field_val) not in [long, int] or field_val < 0:
                raise SerializationError('field [%s] must be unsigned integer type'%field_name)
            maxval = int(math.pow(2, _widths[field_type]))
            if field_val >= maxval:
                raise SerializationError('field [%s] exceeds specified width [%s]'%(field_name, field_type))
    elif field_type == 'string':
        if type(field_val) != str:
            raise SerializationError('field [%s] must be of type [str]'%field_name)
    elif field_type == 'time':
        if not isinstance(field_val, Time):
            raise SerializationError('field [%s] must be of type [Time]'%field_name)
    elif field_type == 'duration':
        if not isinstance(field_val, Duration):
            raise SerializationError('field [%s] must be of type [Duration]'%field_name)
    elif field_type.endswith(']'): # array type
        if not type(field_val) in [list, tuple]:
            raise SerializationError('field [%s] must be a list or tuple type'%field_name)
        # use index to generate error if '[' not present
        base_type = field_type[:field_type.index('[')]
        for v in field_val:
            check_type(field_name, base_type, v)
    #else:
    #    if type(field_val) != types.InstanceType:
    #        raise SerializationError('field [%s] must be a [%s] instance instead of a %s'%(field_name, field_type, type(field_val)))
        #TODO: dynamically load message class and do instance compare

##Base class of auto-generated message data objects. 
class Message(object):
    # slots is explicitly both for data representation and
    # performance. Higher-level code assumes that there is a 1-to-1
    # mapping between __slots__ and message fields. In terms of
    # performance, explicitly settings slots eliminates dictionary for
    # new-style object.
    __slots__ = ['_connection_header']
    
    ## Message base constructor. Contains generic initializers for
    ## args-based and kwds-based initialization of message fields,
    ## assuming there is a one-to-one mapping between __slots__ and
    ## message fields.
    def __init__(self, *args, **kwds):
        if args and kwds:
            raise TypeError("Message constructor may only use args OR keywords, not both")
        if args:
            if len(args) != len(self.__slots__):
                raise TypeError, "Invalid number of arguments, args should be %s"%str(self.__slots__)
            for i, k in enumerate(self.__slots__):
                setattr(self, k, args[i])
        else:
            # validate kwds
            for k,v in kwds.iteritems():
                if not k in self.__slots__:
                    raise AttributeError("%s is not an attribute of %s"%(k, self.__class__.__name__))
            # iterate through slots so all fields are initialized.
            # this is important so that subclasses don't reference an
            # uninitialized field and raise an AttributeError.
            for k in self.__slots__:
                if k in kwds:
                    setattr(self, k, kwds[k])
                else:
                    setattr(self, k, None)

    def _get_types(self):
        raise Exception("must be overriden")
    ## Perform dynamic type-checking of Message fields. This is performance intensive
    ## and is meant for post-error diagnosis
    ## @param exc Exception: underlying exception that gave cause for type check. 
    ## @throws roslib.messages.SerializationError if typecheck fails
    def _check_types(self, exc=None):
        for n, t in zip(self.__slots__, self._get_types()):
            check_type(n, t, getattr(self, n))
        if exc: # if exc is set and check_type could not diagnose, raise wrapped error
            raise SerializationError(str(exc))

    def serialize(self, buff):
        pass
    def deserialize(self, str):
        pass
    def __str__(self):
        return strify_message(self)
    # TODO: unit test
    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        for f in self.__slots__:
            try:
                v1 = getattr(self, f) 
                v2 = getattr(other, f)
                if type(v1) in (list, tuple) and type(v2) in (list, tuple):
                    # we treat tuples and lists as equivalent
                    if tuple(v1) != tuple(v2):
                        return False
                elif not v1 == v2:
                    return False
            except AttributeError:
                return False
        return True
    
class ServiceDefinition(object): pass #marker class for auto-generated code

## Message deserialization error
class DeserializationError(ROSMessageException): pass
## Message serialization error
class SerializationError(ROSMessageException): pass

# Utilities for rostopic/rosservice

## Get string representation of msg arguments
## @param msg Message: msg message to fill
## @param prefix str: field name prefix (for verbose printing)
## @return str: printable representation of \a msg args
def get_printable_message_args(msg, buff=None, prefix=''):
    if buff is None:
        buff = cStringIO.StringIO()
    for f in msg.__slots__:
        if isinstance(getattr(msg, f), Message):
            get_printable_message_args(getattr(msg, f), buff=buff, prefix=(prefix+f+'.'))
        else:
            buff.write(prefix+f+' ')
    return buff.getvalue().rstrip()

## Populate message with specified args. 
## @param msg Message: message to fill
## @param msg_args [args]: list of arguments to set fields to
## @param prefix str: field name prefix (for verbose printing)
## @return [args]: unused/leftover message arguments. 
## @throws ROSMessageException if not enough message arguments to fill message
def _fill_message_args(msg, msg_args, prefix=''):
    if not isinstance(msg, Message):
        raise ROSMessageException("msg must be a Message instance: %s"%msg)

    if type(msg_args) == dict:

        for f, v in msg_args.iteritems():
            if not f in msg.__slots__:
                raise ROSMessageException("No field name [%s%s]"%(prefix, f))
            def_val = getattr(msg, f)
            if isinstance(def_val, Message) or isinstance(def_val, roslib.rostime._TVal):
                leftovers = _fill_message_args(def_val, v, prefix=(prefix+f+'.'))
                if leftovers:
                    raise ROSMessageException("Too many arguments for field [%s%s]: %s"%(prefix, f, v))
            else:
                setattr(msg, f, v)
    elif type(msg_args) == list:
        for f in msg.__slots__:
            if not msg_args:
                raise ROSMessageException("Not enough arguments to fill message, trying to fill %s"%f)
            def_val = getattr(msg, f)
            if isinstance(def_val, Message) or isinstance(def_val, roslib.rostime._TVal):
                # if the next msg_arg is a dictionary, then we can assume the dictionary itself represents the message
                if type(msg_args[0]) == dict:
                    next_ = msg_args[0]
                    msg_args = msg_args[1:]
                    _fill_message_args(def_val, next_, prefix=(prefix+f+'.'))
                else:
                    msg_args = _fill_message_args(def_val, msg_args, prefix=(prefix+f+'.'))
            else:
                next_ = msg_args[0]
                msg_args = msg_args[1:]
                if type(next_) == dict:
                    raise ROSMessageException("received dictionary for non-message field[%s%s]: %s"%(prefix, f, next_))
                else:
                    setattr(msg, f, next_)
        return msg_args
    else:
        raise ROSMessageException("invalid message_args type: %s"%msg_args)

## Populate message with specified args. 
## @param msg Message: message to fill
## @param msg_args [args]: list of arguments to set fields to
## @throws ROSMessageException if not enough/too many message arguments to fill message
def fill_message_args(msg, msg_args):
    leftovers = _fill_message_args(msg, msg_args, '')
    if leftovers:
        raise ROSMessageException("received too many arguments for message")

