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

## ROS message source code generation for Python
##
## Converts ROS .msg files into Python source code implementations.

import roslib; roslib.load_manifest('rospy')

import sys
import os
import re
import string
import traceback
import struct

import roslib.gentools 
import roslib.msgs 
import roslib.packages 
import genutil

from roslib.genpy import pack, pack2, unpack, unpack2, reduce_pattern

INDENT = '  '
INDENT2 = '    '

IMPORTS = []

class Special:
    ##
    #  @param constructor str: expression to instantiate new type instance for deserialization
    #  @param postDerialize str: format string for expression to evaluate on type instance after deserialization is complete.
    #    variable name will be passed in as the single argument to format string.
    #  @param importStr str: import to include if type is present
    def __init__(self, constructor, postDeserialize, importStr):
        self.importStr = importStr
        self.constructor = constructor
        self.postDeserialize = postDeserialize
        
SPECIAL_TYPES = {
    roslib.msgs.HEADER:   Special('roslib.msg._Header.Header()',         None,   'import roslib.msg'),
    roslib.msgs.TIME:     Special('roslib.rostime.Time()',     '%s.canon()', 'import roslib.rostime'),
    roslib.msgs.DURATION: Special('roslib.rostime.Duration()', '%s.canon()', 'import roslib.rostime'), 
    }

## wrapper for roslib.msgs.get_registered that wraps unknown types with a MsgGenerationException
def _get_registered(type):
    try:
        return roslib.msgs.get_registered(type)
    except KeyError:
        raise MsgGenerationException("Unknown type [%s]. Please check that the manifest.xml correctly declares dependencies."%type)
    
## Compute the base data type, e.g. for arrays, get the array item type
## @return str: base type
def get_base_type(type):
    if '[' in type:
        return type[:type.find('[')]
    return type

## @param package str: package that type is being imported into
## @param type    str: message type (package resource name)
## @return (str, str): python package and type name
def _compute_pkg_type(package, type):
    splits = type.split(roslib.msgs.SEP)
    if len(splits) == 1:
        return package, splits[0]
    elif len(splits) == 2:
        return splits    
    else:
        raise MsgGenerationException("illegal message type: %s"%type)
    
## compute python import statement for specified message type implementation
#  @param package str: package that type is being imported into
#  @param type    str: message type (package resource name)
def compute_import(package, type):
    type = get_base_type(type)
    if not roslib.msgs.is_registered(type):
        return []
    spec = _get_registered(type)
    if type in SPECIAL_TYPES:
        return [SPECIAL_TYPES[type].importStr]
    pkg, type = _compute_pkg_type(package, type)
    retval = ['import %s.msg'%pkg]
    for t in spec.types:
        retval.extend(compute_import(package, t))
    return retval

## compute python constructor expression for specified message type implementation
## @param package str: package that type is being imported into
## @param type str: message type
def compute_constructor(package, type):
    if not roslib.msgs.is_registered(type):
        return None
    elif type in SPECIAL_TYPES:
        return SPECIAL_TYPES[type].constructor
    else:
        pkg, type = _compute_pkg_type(package, type)
        return '%s.msg.%s()'%(pkg, type)
        
class MsgGenerationException(genutil.GenException): pass

_serial_context = ''
_context_stack = []
def push_context(context):
    """manage field-reference context for serialization,
    e.g. 'self.foo' vs. 'self.bar.foo' vs. 'var.foo'"""
    global _serial_context, _context_stack
    _context_stack.append(_serial_context)
    _serial_context = context

def pop_context():
    global _serial_context, _context_stack
    _serial_context = _context_stack.pop()

def len_serializer_generator(var, is_string, serialize):
    UINT = "I"
    if serialize:
        yield "length = len(%s)"%var
        if not is_string:
            yield pack(UINT, "length")
    else:
        yield "start = end"
        yield "end += 4"
        yield unpack('(length,)', UINT, 'str[start:end]') #4 = struct.calcsize('<i') 
        UINT = "I"
    
def string_serializer_generator(package, type, name, serialize):
    """generator for string types. similar to arrays by struct encoding is more efficient."""
    var = _serial_context+name
    for y in len_serializer_generator(var, True, serialize):
        yield y #serialize string length

    if serialize:
        #serialize length and string together
        yield "#serialize %s"%var
        yield pack2("'<I%ss'%length", "length, %s"%var)
    else:
        yield "#deserialize %s"%var
        yield "pattern = '<%ss'%length"
        yield "start = end"
        yield "end += struct.calcsize(pattern)"
        yield unpack2("(%s,)"%var, 'pattern', 'str[start:end]')
        

#TODO: support for N-dimensional arrays (don't have spec yet)
def array_serializer_generator(package, type, name, serialize):
    """generator for array types
    @raise MsgGenerationException: if array spec is invalid
    """
    if not type.endswith(']'):
        raise MsgGenerationException("Invalid array spec: %s"%type)
    var_length = type.endswith('[]')
    splits = type.split('[')
    if len(splits) <= 1:
        raise MsgGenerationException("Array type missing '[': %s"%type)
    if len(splits) > 2:
        raise MsgGenerationException("Currently only support 1-dimensional array types: %s"%type)
    
    base_type = splits[0]
    #TODO: we don't handle fixed-size byte arrays properly yet
    if var_length and base_type in ['byte', 'uint8']: #treat byte[] as string type
        for y in string_serializer_generator(package, type, name, serialize):
            yield y
        return
    
    var = _serial_context+name
    if serialize:
        yield "#serialize %s"%var
    else:
        yield "#deserialize %s"%var
    try:
        if var_length:
            #Variable Length Array
            #print "Variable Length", type
            for y in len_serializer_generator(var, False, serialize):
                yield y #serialize array length
            length = None
        else:
            #Fixed Length Array
            length = string.atoi(splits[1][:-1])
        
        #optimization for simple arrays                
        if roslib.msgs.is_simple_spec(base_type):
            if var_length:
                pattern = roslib.msgs.get_struct_pattern([base_type])
                yield "pattern = '<%%s%s'%%length"%pattern
                if serialize:
                    yield pack2('pattern', "*"+var)
                else:
                    yield "start = end" 
                    yield "end += struct.calcsize(pattern)"
                    yield unpack2(var, 'pattern', 'str[start:end]')
            else:
                pattern = "%s%s"%(length, roslib.msgs.get_struct_pattern([base_type]))
                if serialize:
                    yield pack(pattern, "*"+var)
                else:
                    yield "start = end"
                    yield "end += %s"%struct.calcsize('<%s'%pattern)
                    yield unpack(var, pattern, 'str[start:end]')
        else:
            #generic recursive serializer
            #NOTE: this is functionally equivalent to the is_registered branch of complex_serializer_generator

            # choose a unique temporary variable for iterating
            loop_var = 'val%s'%len(_context_stack)

            if base_type == 'string':
                push_context('') 
                factory = string_serializer_generator(package, base_type, loop_var, serialize)
            else:
                push_context('%s.'%loop_var) 
                factory = serializer_generator_factory(serialize)(package, _get_registered(base_type))

            if serialize:
                yield 'for %s in %s:'%(loop_var, var)
            else:
                yield '%s = []'%var
                if var_length:
                    yield 'for i in xrange(0, length):'
                else:
                    yield 'for i in xrange(0, %s):'%length
                if base_type != 'string':
                    yield INDENT + '%s = %s'%(loop_var, compute_constructor(package, base_type))
            for y in factory:
                yield INDENT + y
            if not serialize:
                yield INDENT + '%s.append(%s)'%(var, loop_var)
            pop_context()

    except MsgGenerationException:
        raise #re-raise
    except Exception, e:
        #REMOVE?
        traceback.print_exc()
        raise MsgGenerationException(e) #wrap
    
def complex_serializer_generator(package, type, name, serialize):
    """Generator for serializing complex type
    @raise MsgGenerationException: if type is not a valid"""

    #Embedded Message
    if roslib.msgs.is_registered(type):
        # descend data structure ####################
        push_context(_serial_context+name+'.') 
        for y in serializer_generator_factory(serialize)(package, _get_registered(type)):
            yield y #recurs on subtype
        pop_context()
        # done with descend data structure ##########        
    elif type == 'string':
        for y in string_serializer_generator(package, type, name, serialize):
            yield y
    #Array
    elif type.endswith(']'):
        for y in array_serializer_generator(package, type, name, serialize):
            yield y
    #Invalid
    else:
        raise MsgGenerationException("Unknown type: %s"%type)

def serializer_generator_factory(serialize):
    """Python generator that yields un-indented python code for serializing type spec"""
    ## @param package str: name of package the spec is being used in
    def generator(package, spec):
        # Break spec into chunks of simple (primitives) vs. complex (arrays, etc...)
        # Simple types are batch serialized using the python struct module.
        # Complex types are individually serialized 
        if spec is None:
            raise MsgGenerationException("spec is none")
        if serialize and not len(spec.names): #Empty 
            yield "pass"
            return

        names, types = spec.names, spec.types
        def simple(last, end): #primitives that can be handled with struct
            vars = _serial_context + (', '+_serial_context).join(names[last:end])
            pattern = roslib.msgs.get_struct_pattern(types[last:end])
            if serialize:
                yield pack(pattern, vars)
            else:
                yield "start = end"
                yield "end += %s"%struct.calcsize('<%s'%reduce_pattern(pattern))
                yield unpack('(%s,)'%vars, pattern, 'str[start:end]')

        last = 0
        for (i, type) in enumerate(types):
            if not roslib.msgs.is_simple_spec(type):
                if i != last: #yield chunk of simples
                    for y in simple(last, i):
                        yield y
                last = i+1 #increment past current type
                for y in complex_serializer_generator(package, type, names[i], serialize): 
                    yield y #yield the sub-generator
        if last < len(types): #consume rest of simples
            for y in simple(last, len(types)):
                yield y
    return generator

## generator for body of serialize() function
def serialize_fn_generator(package, spec):
    # method-var context #########
    yield "try:"
    push_context('self.')
    #NOTE: we flatten the spec for optimal serialization
    for y in serializer_generator_factory(True)(package, spec.flatten()):
        yield "  "+y 
    pop_context()
    yield "except struct.error, se: self._check_types(se)"
    yield "except TypeError, te: self._check_types(te)" 
    # done w/ method-var context #
    
## generator for body of deserialize() function
def deserialize_fn_generator(package, spec):
    yield "try:"
    
    #Instantiate embedded type classes
    for type, name in spec.fields():
        if roslib.msgs.is_registered(type):
            yield "  if self.%s is None:"%name
            yield "    self.%s = %s"%(name, compute_constructor(package, type))
    yield "  end = 0" #initialize var

    # method-var context #########
    push_context('self.')
    #NOTE: we flatten the spec for optimal serialization    
    for y in serializer_generator_factory(False)(package, spec.flatten()):
        yield "  "+y
    pop_context()
    # done w/ method-var context #
    for type, name in spec.fields():
        if type in SPECIAL_TYPES:
            s = SPECIAL_TYPES[type]
            if s.postDeserialize:
                varname = "self.%s"%name
                yield "  %s"%(s.postDeserialize%varname)
    
    yield "  return self"
    yield "except struct.error, e:"
    yield "  raise roslib.message.DeserializationError(e) #most likely buffer underfill"

# #671
## Compute default value for \a field_type
## @param default_package str: default package
## @param field_type str: ROS .msg field type
def _default_value(field_type, default_package):
    if field_type in ['byte', 'int8', 'int16', 'int32', 'int64',\
                          'char', 'uint8', 'uint16', 'uint32', 'uint64']:
        return '0'
    elif field_type in ['float32', 'float64']:
        return '0.'
    elif field_type == 'string':
        return "''"
    elif field_type.endswith(']'): # array type
        return '[]'
    else:
        return compute_constructor(default_package, field_type)
        
def msg_generator(package, name, spec):
    spec_names = spec.names

    yield '# autogenerated by genmsg_py from %s.msg. Do not edit.'%name
    yield 'import roslib.message\nimport struct\n'
    import_strs = []
    for t in spec.types:
        import_strs.extend(compute_import(package, t))
    import_strs = set(import_strs)
    for i in import_strs:
        if i:
            yield i

    yield '## \htmlinclude %s%s.html'%(name,roslib.msgs.EXT) #doxygen    
    
    yield ''
    
    fulltype = '%s%s%s'%(package, roslib.msgs.SEP, name)

    gendeps_dict = roslib.gentools.get_dependencies(spec, package)
    md5 = roslib.gentools.compute_md5(gendeps_dict)
    msg_definition = roslib.gentools.compute_full_text(gendeps_dict)
    # escape the msg_definition to be safe for our """ string quoting
    msg_definition.replace('"""', r'\"\"\"')
    #Yield data class first, e.g. Point2D
    yield 'class %s(roslib.message.Message):'%name
    yield '  _md5sum = "%s"'%md5
    yield '  _type = "%s"'%fulltype
    yield '  _has_header = %s #flag to mark the presence of a Header object'%spec.has_header()
    yield '  _full_text = """%s"""'%msg_definition

    if spec.constants:
        yield '  # Pseudo-constants'
        for c in spec.constants:
            if c.type == 'string':
                val = c.val
                if '"' and "'" in val:
                    # crude escaping of \ and "
                    escaped = c.val.replace('\\', '\\\\')
                    escaped = escaped.replace('\"', '\\"')
                    yield '  %s = "%s"'%(c.name, escaped)                    
                elif '"' in val: #use raw encoding for prettiness
                    yield "  %s = r'%s'"%(c.name, val)
                elif "'" in val: #use raw encoding for prettiness
                    yield '  %s = r"%s"'%(c.name, val)
            else:
                yield '  %s = %s'%(c.name, c.val)
        yield ''

    if len(spec_names):
        yield "  __slots__ = ['"+"','".join(spec_names)+"']"
        yield "  _slot_types = ['"+"','".join(spec.types)+"']"        
    else:
        yield "  __slots__ = []"
        yield "  _slot_types = []"

    yield """
  ## Constructor. Any message fields that are implicitly/explicitly
  ## set to None will be assigned a default value. The recommend
  ## use is keyword arguments as this is more robust to future message
  ## changes.  You cannot mix in-order arguments and keyword arguments.
  ##
  ## The available fields are:
  ##   %s
  ##
  ## @param args: complete set of field values, in .msg order
  ## @param kwds: use keyword arguments corresponding to message field names
  ## to set specific fields. 
  def __init__(self, *args, **kwds):
    super(%s, self).__init__(*args, **kwds)"""%(','.join(spec_names), name)

    if len(spec_names):
        yield "    #message fields cannot be None, assign default values for those that are"
        for (t, s) in zip(spec.types, spec_names):
            yield "    if self.%s is None:"%s
            yield "      self.%s = %s"%(s, _default_value(t, package))

    yield """
  ## internal API method
  def _get_types(self): return %s._slot_types

  ## serialize message into buffer
  ## @param buff StringIO: buffer
  def serialize(self, buff):"""%name
    for y in serialize_fn_generator(package, spec):
        yield "    "+ y
    yield """
  ## unpack serialized message in str into this message instance
  ## @param str str: byte array of serialized message
  def deserialize(self, str):"""
    for y in deserialize_fn_generator(package, spec):
        yield "    " + y
    yield ""

class MsgGenerator(genutil.Generator):
    def __init__(self):
        super(MsgGenerator, self).__init__(
            'genmsg_py', 'messages', roslib.msgs.EXT, roslib.packages.MSG_DIR, MsgGenerationException)

    def generate(self, package, f, outdir):
        verbose = True
        #doing this lazy saves 0.05s on up-to-date builds
        if not _reserved_words:
            _load_reserved_words()
        f = os.path.abspath(f)
        infileName = os.path.basename(f)
        outfile_name = self.outfile_name(outdir, infileName)

        (name, spec) = roslib.msgs.load_from_file(f)
        spec.names = [_remap_reserved(n) for n in spec.names]
        
        self.write_gen(outfile_name, msg_generator(package, name, spec), verbose)

        roslib.msgs.register(name, spec)
        return outfile_name

def _remap_reserved(field_name):
    if field_name in _reserved_words:
        return field_name + "_"
    return field_name

_reserved_words = []
## load python reserved words file. these reserved words cannot be field names without being altered first
def _load_reserved_words():
    global _reserved_words
    rospy_dir = roslib.packages.get_pkg_dir('rospy')
    try:
        reserved_words_file = os.path.join(rospy_dir, 'scripts', 'python-reserved-words.txt')
        if not os.path.exists(reserved_words_file):
            print >> sys.stderr, "Cannot load python reserved words file [%s]"%reserved_words_file
            return False
        f = open(reserved_words_file, 'r')
        _reserved_words = [w.strip() for w in f.readlines() if w]
        return True
    finally:
        f.close()
    
if __name__ == "__main__":
    roslib.msgs.set_verbose(False)
    genutil.genmain(sys.argv, MsgGenerator())
