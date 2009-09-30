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
# Revision $Id: genpy.py 3357 2009-01-13 07:13:05Z jfaustwg $

## Library for Python message generators.
## The structure of the serialization descends several levels of serializers:
##  - msg_generator: generator for an individual msg file
##   - serialize_fn_generator: generator for msg.serialize()
##     - serializer_generator
##       - field-type-specific serializers
##   - deserialize_fn_generator: generator for msg.deserialize()
##     - serializer_generator
##       - field-type-specific serializers

# NOTE: genpy must be in the roslib package as placing it in rospy
# creates circular deps

import os
import shutil
import atexit
import itertools
import sys
import tempfile
import traceback
import struct

from cStringIO import StringIO

import roslib.exceptions
import roslib.gentools
import roslib.msgs
import roslib.packages #for get_pkg_dir

# indent width
INDENT = '  '

## Exception type for errors in roslib.genpy
class MsgGenerationException(roslib.exceptions.ROSLibException): pass

## wrapper for roslib.msgs.get_registered that wraps unknown types with a MsgGenerationException
## @param type_ str: ROS message type
def get_registered_ex(type_):
    try:
        return roslib.msgs.get_registered(type_)
    except KeyError:
        raise MsgGenerationException("Unknown type [%s]. Please check that the manifest.xml correctly declares dependencies."%type_)

################################################################################
# Primitive type handling for ROS builtin types

SIMPLE_TYPES_DICT = { #see python module struct
    'int8': 'b', 
    'uint8': 'B',
    # Python 2.6 adds in '?' for C99 _Bool, which appears equivalent to an uint8,
    # thus, we use uint8
    'bool': 'B',    
    'int16' : 'h',
    'uint16' : 'H',
    'int32' : 'i',
    'uint32' : 'I',
    'int64' : 'q',
    'uint64' : 'Q',
    'float32': 'f',
    'float64': 'd',
    # deprecated
    'char' : 'B', #unsigned
    'byte' : 'b', #signed
    }

## Simple types are primitives with fixed-serialization length
SIMPLE_TYPES = SIMPLE_TYPES_DICT.keys()

## @return bool: True if type is a 'simple' type, i.e. is of
## fixed/known serialization length. This is effectively all primitive
## types except for string"
def is_simple(type_):
    return type_ in SIMPLE_TYPES

## @return True if \a type_ is a special type (i.e. builtin represented as a class instead of a primitive)
def is_special(type_):
    return type_ in _SPECIAL_TYPES

## @internal
## @return Special special type handler for \a type_ or None
def get_special(type_):
    return _SPECIAL_TYPES.get(type_, None)
                     
################################################################################
# Special type handling for ROS builtin types that are not primitives

class Special:

    ## @param constructor str: expression to instantiate new type instance for deserialization
    ## @param post_Deserialize str: format string for expression to evaluate on type instance after deserialization is complete.
    ##   variable name will be passed in as the single argument to format string.
    ## @param import_str str: import to include if type is present
    def __init__(self, constructor, post_deserialize, import_str):
        self.constructor = constructor
        self.post_deserialize = post_deserialize
        self.import_str = import_str
        
    ## @return Post-deserialization code to executed (unindented) or
    ## None if no post-deserialization is required
    def get_post_deserialize(self, varname):
        if self.post_deserialize:
            return self.post_deserialize%varname
        else:
            return None
        
_SPECIAL_TYPES = {
    roslib.msgs.HEADER:   Special('roslib.msg._Header.Header()',         None,   'import roslib.msg'),
    roslib.msgs.TIME:     Special('roslib.rostime.Time()',     '%s.canon()', 'import roslib.rostime'),
    roslib.msgs.DURATION: Special('roslib.rostime.Duration()', '%s.canon()', 'import roslib.rostime'), 
    }

################################################################################
# utilities

# #671
## Compute default value for \a field_type
## @param default_package str: default package
## @param field_type str: ROS .msg field type
def default_value(field_type, default_package):
    if field_type in ['byte', 'int8', 'int16', 'int32', 'int64',\
                          'char', 'uint8', 'uint16', 'uint32', 'uint64']:
        return '0'
    elif field_type in ['float32', 'float64']:
        return '0.'
    elif field_type == 'string':
        # strings, byte[], and uint8s are all optimized to be strings
        return "''"
    elif field_type.endswith(']'): # array type
        base_type, is_array, array_len = roslib.msgs.parse_type(field_type)
        if base_type in ['byte', 'uint8']:
            # strings, byte[], and uint8s are all optimized to be strings
            if array_len is not None:
                return "chr(0)*%s"%array_len
            else:
                return "''"
        elif array_len is None: #var-length
            return '[]'
        else: # fixed-length, fill values
            def_val = default_value(base_type, default_package)
            return '[' + ','.join(itertools.repeat(def_val, array_len)) + ']'
    else:
        return compute_constructor(default_package, field_type)

## Flattens the msg spec so that embedded message fields become
## direct references. The resulting MsgSpec isn't a true/legal
## MsgSpec and should only be used for serializer generation
## @param msg MsgSpec: msg to flatten
## @return MsgSpec flatten message
def flatten(msg):
    new_types = []
    new_names = []
    for t, n in zip(msg.types, msg.names):
        #flatten embedded types - note: bug #59
        if roslib.msgs.is_registered(t):
            msg_spec = flatten(roslib.msgs.get_registered(t))
            new_types.extend(msg_spec.types)
            for n2 in msg_spec.names:
                new_names.append(n+'.'+n2)
        else:
            #I'm not sure if it's a performance win to flatten fixed-length arrays
            #as you get n __getitems__ method calls vs. a single *array call
            new_types.append(t)
            new_names.append(n)
    return roslib.msgs.MsgSpec(new_types, new_names, msg.constants, msg.text)

## Remap field/constant names in \a spec to avoid collision with Python reserved words. 
## @param spec MsgSpec: msg spec to map to new, python-safe field names
## @return MsgSpec: python-safe message specification
def make_python_safe(spec):
    new_c = [roslib.msgs.Constant(c.type, _remap_reserved(c.name), c.val, c.val_text) for c in spec.constants]
    return roslib.msgs.MsgSpec(spec.types, [_remap_reserved(n) for n in spec.names], new_c, spec.text)

## @internal
## map \a field_name to a python-safe representation, if necessary
def _remap_reserved(field_name):
    #doing this lazy saves 0.05s on up-to-date builds
    if not _reserved_words:
        _load_reserved_words()
    if field_name in _reserved_words:
        return field_name + "_"
    return field_name

_reserved_words = []
## load python reserved words file. these reserved words cannot be field names without being altered first
def _load_reserved_words():
    global _reserved_words
    roslib_dir = roslib.packages.get_pkg_dir('roslib')
    reserved_words_file = os.path.join(roslib_dir, 'scripts', 'python-reserved-words.txt')
    if not os.path.exists(reserved_words_file):
        print >> sys.stderr, "Cannot load python reserved words file [%s]"%reserved_words_file
        return False
    f = open(reserved_words_file, 'r')
    try:
        _reserved_words = [w.strip() for w in f.readlines() if w]
        return True
    finally:
        f.close()
    
################################################################################
# (de)serialization routines

## @param [str]: type names
## @return str: format string for struct if types are all simple. Otherwise, return None
def compute_struct_pattern(types):
    if not types: #important to filter None and empty first
        return None
    try: 
        return ''.join([SIMPLE_TYPES_DICT[t] for t in types])
    except:
        return None

## compute post-deserialization code for \a type_, if necessary
## @return str: code to execute post-deserialization (unindented), or None if not necessary.
def compute_post_deserialize(type_, varname):
    s = get_special(type_)
    if s is not None:
        return s.get_post_deserialize(varname)

## compute python constructor expression for specified message type implementation
## @param package str: package that type is being imported into. Used
## to resolve \a type_ if package is not specified.
## @param type_ str: message type
def compute_constructor(package, type_):
    if is_special(type_):
        return get_special(type_).constructor
    else:
        base_pkg, base_type_ = compute_pkg_type(package, type_)
        if not roslib.msgs.is_registered("%s/%s"%(base_pkg,base_type_)):
            return None
        else:
            return '%s.msg.%s()'%(base_pkg, base_type_)

## @param package str: package that type is being imported into
## @param type    str: message type (package resource name)
## @return (str, str): python package and type name
def compute_pkg_type(package, type_):
    splits = type_.split(roslib.msgs.SEP)
    if len(splits) == 1:
        return package, splits[0]
    elif len(splits) == 2:
        return tuple(splits)
    else:
        raise MsgGenerationException("illegal message type: %s"%type_)
    
## compute python import statement for specified message type implementation
## @param package str: package that type is being imported into
## @param type_ str: message type (package resource name)
## @return [str]: list of import statements (no newline) required to use \a type_ from \a package
def compute_import(package, type_):
    # orig_base_type is the unresolved type
    orig_base_type = roslib.msgs.base_msg_type(type_) # strip array-suffix
    # resolve orig_base_type based on the current package context.
    # base_type is the resolved type stripped of any package name.
    # pkg is the actual package of type_.
    pkg, base_type = compute_pkg_type(package, orig_base_type)
    type_str = "%s/%s"%(pkg, base_type) # compute fully-qualified type
    # important: have to do is_builtin check first. We do this check
    # against the unresolved type builtins/specials are never
    # relative. This requires some special handling for Header, which has
    # two names (Header and roslib/Header).
    if roslib.msgs.is_builtin(orig_base_type) or \
           roslib.msgs.is_header_type(orig_base_type):
        # of the builtin types, only special types require import
        # handling. we switch to base_type as special types do not
        # include package names.
        if is_special(base_type):
            retval = [get_special(base_type).import_str]
        else:
            retval = []
    elif not roslib.msgs.is_registered(type_str):
        retval = []
    else:
        retval = ['import %s.msg'%pkg]
        for t in get_registered_ex(type_str).types:
            if '/' in t or is_special(t): # don't need to include relative types or primitive builtins
                sub = compute_import(package, t)
                retval.extend([x for x in sub if not x in retval])
    return retval

## Same as roslib.gentools.compute_full_text, except that the resulting text is escaped to be safe for """ string quoting
## @param get_deps_dict dict: dictionary returned by get_dependencies call
## @return str concatenated text for msg/srv file and embedded msg/srv types. Text will be escaped for """ string quoting
def compute_full_text_escaped(gen_deps_dict):
    msg_definition = roslib.gentools.compute_full_text(gen_deps_dict)
    msg_definition.replace('"""', r'\"\"\"')
    return msg_definition

## optimize the struct format pattern. 
def reduce_pattern(pattern):
    if not pattern or len(pattern) == 1 or '%' in pattern:
        return pattern
    prev = pattern[0]
    count = 1
    new_pattern = ''
    nums = [str(i) for i in range(0, 9)]
    for c in pattern[1:]:
        if c == prev and not c in nums:
            count += 1
        else:
            if count > 1:
                new_pattern = new_pattern + str(count) + prev
            else:
                new_pattern = new_pattern + prev
            prev = c
            count = 1
    if count > 1:
        new_pattern = new_pattern + str(count) + c
    else:
        new_pattern = new_pattern + prev
    return new_pattern

## @param expr str: string python expression that is evaluated for serialization
## @return str: python call to write value returned by expr to serialization buffer
def serialize(expr):
    return "buff.write(%s)"%expr
    
#NOTE: '<' = little endian
def pack(pattern, vars):
    """create struct.pack call for when pattern is a string pattern
    @param pattern: string pattern for pack
    @param vars: name of variables to pack"""
    return serialize("struct.pack('<"+reduce_pattern(pattern)+"', "+vars+")")
def pack2(pattern, vars):
    """create struct.pack call for when pattern is the name of a variable
    @param pattern: name of variable storing string pattern
    @param vars: name of variables to pack"""
    return serialize("struct.pack(%s, %s)"%(pattern, vars))
def unpack(var, pattern, buff):
    """create struct.unpack call for when pattern is a string pattern"""
    return var + " = struct.unpack('<"+reduce_pattern(pattern)+"',"+buff+")"
def unpack2(var, pattern, buff):
    """create struct.unpack call for when pattern refers to variable
    @param var: variable the stores the result of unpack call
    @param pattern: name of variable that unpack will read from
    @param buff: buffer that the unpack reads from"""
    return "%s = struct.unpack(%s, %s)"%(var, pattern, buff)

################################################################################
# numpy support

# this could obviously be directly generated, but it's nice to abstract

## maps ros msg types to numpy types
_NUMPY_DTYPE = {
    'float32': 'numpy.float32',
    'float64': 'numpy.float64',
    'int8': 'numpy.int8',
    'int16': 'numpy.int16',
    'int32': 'numpy.int32',
    'int64': 'numpy.int64',
    'uint8': 'numpy.uint8',
    'uint16': 'numpy.uint16',
    'uint32': 'numpy.uint32',
    'uint64': 'numpy.uint64',
    # deprecated type
    'char' : 'numpy.uint8',
    'byte' : 'numpy.int8',
    }
# TODO: this doesn't explicitly specify little-endian byte order on the numpy data instance
## generate unpack code for numpy array
def unpack_numpy(var, count, dtype, buff):
    """create numpy deserialization code"""
    return var + " = numpy.frombuffer(%s, dtype=%s, count=%s)"%(buff, dtype, count)

## generate pack code for numpy array
def pack_numpy(var):
    """create numpy serialization code
    @param pattern: string pattern for pack
    @param vars: name of variables to pack"""
    return serialize("%s.tostring()"%var)

################################################################################
# (De)serialization generators

_serial_context = ''
_context_stack = []
## Push new variable context onto context stack.  The context stack
## manages field-reference context for serialization, e.g. 'self.foo'
## vs. 'self.bar.foo' vs. 'var.foo'"""
def push_context(context):
    global _serial_context, _context_stack
    _context_stack.append(_serial_context)
    _serial_context = context

## Pop variable context from context stack.  The context stack manages
## field-reference context for serialization, e.g. 'self.foo'
## vs. 'self.bar.foo' vs. 'var.foo'"""
def pop_context():
    global _serial_context
    _serial_context = _context_stack.pop()

# These are the workhorses of the message generation. The generators
# are implemented as iterators, where each iteration value is a line
# of Python code. The generators will invoke underlying generators,
# using the context stack to manage any changes in variable-naming, so
# that code can be reused as much as possible.
    
## Generator for array-length serialization (32-bit, little-endian unsigned integer)
## @param var str: variable name
## @param is_string bool: if True, variable is a string type
## @param serialize bool: if True, generate code for
## serialization. Other, generate code for deserialization
def len_serializer_generator(var, is_string, serialize):
    if serialize:
        yield "length = len(%s)"%var
        # NOTE: it's more difficult to save a call to struct.pack with
        # the array length as we are already using *array_val to pass
        # into struct.pack as *args. Although it's possible that
        # Python is not optimizing it, it is potentially worse for
        # performance to attempt to combine
        if not is_string:
            yield pack("I", "length")
    else:
        yield "start = end"
        yield "end += 4"
        yield unpack('(length,)', "I", 'str[start:end]') #4 = struct.calcsize('<i') 
    
## Generator for string types. similar to arrays, but with more
## efficient call to struct.pack.
## @param name str: spec field name
## @param serialize bool: if True, generate code for
## serialization. Other, generate code for deserialization
def string_serializer_generator(package, type_, name, serialize):
    var = _serial_context+name

    # the length generator is a noop if serialize is True as we
    # optimize the serialization call.
    base_type, is_array, array_len = roslib.msgs.parse_type(type_)
    # - don't serialize length for fixed-length arrays of bytes
    if base_type not in ['uint8', 'byte'] or array_len is None:
        for y in len_serializer_generator(var, True, serialize):
            yield y #serialize string length

    if serialize:
        #serialize length and string together
        yield "#serialize %s"%var

        #check to see if its a uint8/byte type, in which case we need to convert to string before serializing
        base_type, is_array, array_len = roslib.msgs.parse_type(type_)
        if base_type in ['uint8', 'byte']:
            yield "# - if encoded as a list instead, serialize as bytes instead of string"
            if array_len is None:
                yield "if type(%s) in [list, tuple]:"%var
                yield INDENT+pack2("'<I%sB'%length", "length, *%s"%var)
                yield "else:"
                yield INDENT+pack2("'<I%ss'%length", "length, %s"%var)
            else:
                yield "if type(%s) in [list, tuple]:"%var
                yield INDENT+pack('%sB'%array_len, "*%s"%var)
                yield "else:"
                yield INDENT+pack('%ss'%array_len, var)
        else:
            yield pack2("'<I%ss'%length", "length, %s"%var)
    else:
        yield "#deserialize %s"%var
        if array_len is not None:
            yield "pattern = '<%ss'"%array_len
        else:
            yield "pattern = '<%ss'%length"            
        yield "start = end"
        yield "end += struct.calcsize(pattern)"
        yield unpack2("(%s,)"%var, 'pattern', 'str[start:end]')
        
## generator for array types
## @throws MsgGenerationException: if array spec is invalid
def array_serializer_generator(package, type_, name, serialize, is_numpy):
    base_type, is_array, array_len = roslib.msgs.parse_type(type_)
    if not is_array:
        raise MsgGenerationException("Invalid array spec: %s"%type_)
    var_length = array_len is None
    
    # handle fixed-size byte arrays could be slightly more efficient
    # as we recalculated the length in the generated code.
    if base_type in ['byte', 'uint8']: #treat unsigned int8 arrays as string type
        for y in string_serializer_generator(package, type_, name, serialize):
            yield y
        return
    
    var = _serial_context+name
    # yield code comment
    if serialize:
        yield "#serialize %s"%var
    else:
        yield "#deserialize %s"%var
    try:
        # yield length serialization, if necessary
        if var_length:
            for y in len_serializer_generator(var, False, serialize):
                yield y #serialize array length
            length = None
        else:
            length = array_len
        
        #optimization for simple arrays
        if is_simple(base_type):
            if var_length:
                pattern = compute_struct_pattern([base_type])
                yield "pattern = '<%%s%s'%%length"%pattern
                if serialize:
                    if is_numpy:
                        yield pack_numpy(var)                        
                    else:
                        yield pack2('pattern', "*"+var)
                else:
                    yield "start = end" 
                    yield "end += struct.calcsize(pattern)"
                    if is_numpy:
                        dtype = _NUMPY_DTYPE[base_type]
                        yield unpack_numpy(var, 'length', dtype, 'str[start:end]') 
                    else:
                        yield unpack2(var, 'pattern', 'str[start:end]')
            else:
                pattern = "%s%s"%(length, compute_struct_pattern([base_type]))
                if serialize:
                    if is_numpy:
                        yield pack_numpy(var)
                    else:
                        yield pack(pattern, "*"+var)
                else:
                    yield "start = end"
                    yield "end += %s"%struct.calcsize('<%s'%pattern)
                    if is_numpy:
                        dtype = _NUMPY_DTYPE[base_type]
                        yield unpack_numpy(var, length, dtype, 'str[start:end]') 
                    else:
                        yield unpack(var, pattern, 'str[start:end]')
        else:
            #generic recursive serializer
            #NOTE: this is functionally equivalent to the is_registered branch of complex_serializer_generator

            # choose a unique temporary variable for iterating
            loop_var = 'val%s'%len(_context_stack)

            # compute the variable context and factory to use
            if base_type == 'string':
                push_context('') 
                factory = string_serializer_generator(package, base_type, loop_var, serialize)
            else:
                push_context('%s.'%loop_var) 
                factory = serializer_generator(package, get_registered_ex(base_type), serialize, is_numpy)

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
        raise MsgGenerationException(e) #wrap
    
## Generator for serializing complex type
## @param serialize bool: if True, generate serialization
## code. Otherwise, deserialization code.
## @param is_numpy bool: if True, generate serializer code for numpy
## datatypes instead of Python lists
## @raise MsgGenerationException: if type is not a valid
def complex_serializer_generator(package, type_, name, serialize, is_numpy):
    # ordering of these statements is important as we mutate the type
    # string we are checking throughout. parse_type strips array
    # brackets, then we check for the 'complex' builtin types (string,
    # time, duration, Header), then we canonicalize it to an embedded
    # message type.
    _, is_array, _ = roslib.msgs.parse_type(type_)

    #Array
    if is_array:
        for y in array_serializer_generator(package, type_, name, serialize, is_numpy):
            yield y
    #Embedded Message
    elif type_ == 'string':
        for y in string_serializer_generator(package, type_, name, serialize):
            yield y
    else:
        if not is_special(type_):
            # canonicalize type
            pkg, base_type = compute_pkg_type(package, type_)
            type_ = "%s/%s"%(pkg, base_type)
        if roslib.msgs.is_registered(type_):
            # descend data structure ####################
            push_context(_serial_context+name+'.') 
            for y in serializer_generator(package, get_registered_ex(type_), serialize, is_numpy):
                yield y #recurs on subtype
            pop_context()
        else:
            #Invalid
            raise MsgGenerationException("Unknown type: %s. Package context is %s"%(type_, package))

## Generator (de)serialization code for multiple fields from \a spec
## @param spec MsgSpec
## @param start int: first field to serialize
## @param end int: last field to serialize
def simple_serializer_generator(spec, start, end, serialize): #primitives that can be handled with struct
    vars_ = _serial_context + (', '+_serial_context).join(spec.names[start:end])
    pattern = compute_struct_pattern(spec.types[start:end])
    if serialize:
        yield pack(pattern, vars_)
    else:
        yield "start = end"
        yield "end += %s"%struct.calcsize('<%s'%reduce_pattern(pattern))
        yield unpack('(%s,)'%vars_, pattern, 'str[start:end]')

## Python generator that yields un-indented python code for
## (de)serializing MsgSpec. The code this yields is meant to be
## included in a class method and cannot be used
## standalone. serialize_fn_generator and deserialize_fn_generator
## wrap method to provide appropriate class field initializations.
## @param package str: name of package the spec is being used in
## @param serialize bool: if True, yield serialization
## code. Otherwise, yield deserialization code.
## @param is_numpy bool: if True, generate serializer code for numpy datatypes instead of Python lists
def serializer_generator(package, spec, serialize, is_numpy):
    # Break spec into chunks of simple (primitives) vs. complex (arrays, etc...)
    # Simple types are batch serialized using the python struct module.
    # Complex types are individually serialized 
    if spec is None:
        raise MsgGenerationException("spec is none")
    names, types = spec.names, spec.types
    if serialize and not len(names): #Empty 
        yield "pass"
        return

    # iterate through types. whenever we encounter a non-simple type,
    # yield serializer for any simple types we've encountered until
    # then, then yield the complex type serializer
    curr = 0
    for (i, type_) in enumerate(types):
        if not is_simple(type_):
            if i != curr: #yield chunk of simples
                for y in simple_serializer_generator(spec, curr, i, serialize):
                    yield y
            curr = i+1
            for y in complex_serializer_generator(package, type_, names[i], serialize, is_numpy): 
                yield y 
    if curr < len(types): #yield rest of simples
        for y in simple_serializer_generator(spec, curr, len(types), serialize):
            yield y

## generator for body of serialize() function
## @param is_numpy bool: if True, generate serializer code for numpy
## datatypes instead of Python lists
def serialize_fn_generator(package, spec, is_numpy=False):
    # method-var context #########
    yield "try:"
    push_context('self.')
    #NOTE: we flatten the spec for optimal serialization
    for y in serializer_generator(package, flatten(spec), True, is_numpy):
        yield "  "+y 
    pop_context()
    yield "except struct.error, se: self._check_types(se)"
    yield "except TypeError, te: self._check_types(te)" 
    # done w/ method-var context #
    
## generator for body of deserialize() function
## @param is_numpy bool: if True, generate serializer code for numpy
## datatypes instead of Python lists
def deserialize_fn_generator(package, spec, is_numpy=False):
    yield "try:"
    
    #Instantiate embedded type classes
    for type_, name in spec.fields():
        if roslib.msgs.is_registered(type_):
            yield "  if self.%s is None:"%name
            yield "    self.%s = %s"%(name, compute_constructor(package, type_))
    yield "  end = 0" #initialize var

    # method-var context #########
    push_context('self.')
    #NOTE: we flatten the spec for optimal serialization    
    for y in serializer_generator(package, flatten(spec), False, is_numpy):
        yield "  "+y
    pop_context()
    # done w/ method-var context #

    # generate post-deserialization code 
    for type_, name in spec.fields():
        code = compute_post_deserialize(type_, "self.%s"%name)
        if code:
            yield "  %s"%code
    
    yield "  return self"
    yield "except struct.error, e:"
    yield "  raise roslib.message.DeserializationError(e) #most likely buffer underfill"

## Python code generator for .msg files. Takes in a package name,
## message name, and message specification and generates a Python
## message class.
## @param package str: name of package for message
## @param name str: base type name of message, e.g. 'Empty', 'String'
## @param spec MsgSpec: parsed .msg specification
def msg_generator(package, name, spec):
    spec = make_python_safe(spec) # remap spec names to be Python-safe
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

    # generate dependencies dictionary. omit files calculation as we
    # rely on in-memory MsgSpecs instead so that we can generate code
    # for older versions of msg files
    gendeps_dict = roslib.gentools.get_dependencies(spec, package, compute_files=False)
    #Yield data class first, e.g. Point2D
    yield 'class %s(roslib.message.Message):'%name
    yield '  _md5sum = "%s"'%roslib.gentools.compute_md5(gendeps_dict)
    yield '  _type = "%s"'%fulltype
    yield '  _has_header = %s #flag to mark the presence of a Header object'%spec.has_header()
    # note: we introduce an extra newline to protect the escaping from quotes in the message
    yield '  _full_text = """%s\n"""'%compute_full_text_escaped(gendeps_dict)

    if spec.constants:
        yield '  # Pseudo-constants'
        for c in spec.constants:
            if c.type == 'string':
                val = c.val
                if '"' in val and "'" in val:
                    # crude escaping of \ and "
                    escaped = c.val.replace('\\', '\\\\')
                    escaped = escaped.replace('\"', '\\"')
                    yield '  %s = "%s"'%(c.name, escaped)                    
                elif '"' in val: #use raw encoding for prettiness
                    yield "  %s = r'%s'"%(c.name, val)
                elif "'" in val: #use raw encoding for prettiness
                    yield '  %s = r"%s"'%(c.name, val)
                else:
                    yield "  %s = '%s'"%(c.name, val)
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
            yield "      self.%s = %s"%(s, default_value(t, package))

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

    yield """
  ## serialize message with numpy array types into buffer
  ## @param buff StringIO: buffer
  ## @param numpy module: numpy python module
  def serialize_numpy(self, buff, numpy):"""
    for y in serialize_fn_generator(package, spec, is_numpy=True):
        yield "    "+ y
    yield """
  ## unpack serialized message in str into this message instance using numpy for array types
  ## @param str str: byte array of serialized message
  ## @param numpy module: numpy python module
  def deserialize_numpy(self, str, numpy):"""
    for y in deserialize_fn_generator(package, spec, is_numpy=True):
        yield "    " + y
    yield ""
    
################################################################################
# dynamic generation of deserializer

## @param dep_msg str: text of dependent .msg definition
## @return str, MsgSpec: type name, message spec
## @throws MsgGenerationException if dep_msg is improperly formatted
def _generate_dynamic_specs(specs, dep_msg):
    line1 = dep_msg.find('\n')
    msg_line = dep_msg[:line1]
    if not msg_line.startswith("MSG: "):
        raise MsgGenerationException("invalid input to generate_dynamic: dependent type is missing 'MSG:' type declaration header")
    dep_type = msg_line[5:].strip()
    dep_pkg, dep_base_type = roslib.names.package_resource_name(dep_type)
    dep_spec = roslib.msgs.load_from_string(dep_msg[line1+1:], dep_pkg)
    return dep_type, dep_spec
    
## Modify pkg/base_type name so that it can safely co-exist with
## statically generated files.
## @return str: name to use for pkg/base_type for dynamically generated message class. 
def _gen_dyn_name(pkg, base_type):
    return "_%s__%s"%(pkg, base_type)

## Modify the generated code to rewrite names such that the code can
## safely co-exist with messages of the same name.
## @param py_text str: genmsg_py-generated Python source code
## @return str: updated text
def _gen_dyn_modify_references(py_text, types):
    for t in types:
        pkg, base_type = roslib.names.package_resource_name(t)
        gen_name = _gen_dyn_name(pkg, base_type)
        
        # Several things we have to rewrite:
        # - remove any import statements
        py_text = py_text.replace("import %s.msg"%pkg, '')
        # - rewrite any references to class
        py_text = py_text.replace("%s.msg.%s"%(pkg, base_type), gen_name)
        # - class declaration
        py_text = py_text.replace('class %s('%base_type, 'class %s('%gen_name)
        # - super() references for __init__
        py_text = py_text.replace('super(%s,'%base_type, 'super(%s,'%gen_name)
    # roslib/Header also has to be rewritten to be a local reference
    py_text = py_text.replace('roslib.msg._Header.Header', _gen_dyn_name('roslib', 'Header'))
    return py_text

## Dymamically generate message classes from \a msg_cat .msg text
## gendeps dump. This method modifies sys.path to include a temp file
## directory.
## @param core_type str: top-level ROS message type of concatenanted .msg text
## @param msg_cat str: concatenation of full message text (output of gendeps --cat)
## @throws MsgGenerationException if dep_msg is improperly formatted
def generate_dynamic(core_type, msg_cat):
    core_pkg, core_base_type = roslib.names.package_resource_name(core_type)
    
    # separate msg_cat into the core message and dependencies
    splits = msg_cat.split('\n'+'='*80+'\n')
    core_msg = splits[0]
    deps_msgs = splits[1:]

    # create MsgSpec representations of .msg text
    specs = { core_type: roslib.msgs.load_from_string(core_msg, core_pkg) }
    # - dependencies
    for dep_msg in deps_msgs:
        # dependencies require more handling to determine type name
        dep_type, dep_spec = _generate_dynamic_specs(specs, dep_msg)
        specs[dep_type] = dep_spec
    
    # clear the message registration table and register loaded
    # types. The types have to be registered globally in order for
    # message generation of dependents to work correctly.
    roslib.msgs.reinit()
    for t, spec in specs.iteritems():
        roslib.msgs.register(t, spec)

    # process actual MsgSpecs: we accumulate them into a single file,
    # rewriting the generated text as needed
    buff = StringIO()
    for t, spec in specs.iteritems():
        pkg, s_type = roslib.names.package_resource_name(t)
        # dynamically generate python message code
        for l in msg_generator(pkg, s_type, spec):
            l = _gen_dyn_modify_references(l, specs.keys())
            buff.write(l + '\n')
    full_text = buff.getvalue()

    # Create a temporary directory
    tmp_dir = tempfile.mkdtemp(prefix='genpy_')

    # Afterwards, we are going to remove the directory so that the .pyc file gets cleaned up if it's still around
    atexit.register(shutil.rmtree, tmp_dir)
    
    # write the entire text to a file and import it
    tmp_file = tempfile.NamedTemporaryFile(suffix=".py",dir=tmp_dir)
    tmp_file.file.write(full_text)
    tmp_file.file.close()

    # import our temporary file as a python module, which requires modifying sys.path
    sys.path.append(os.path.dirname(tmp_file.name))

    # - strip the prefix to turn it into the python module name
    mod = __import__(os.path.basename(tmp_file.name)[:-3])

    # finally, retrieve the message classes from the dynamic module
    messages = {}
    for t in specs.iterkeys():
        pkg, s_type = roslib.names.package_resource_name(t)
        try:
            messages[t] = getattr(mod, _gen_dyn_name(pkg, s_type))
        except AttributeError:
            raise MsgGenerationException("cannot retrieve message class for %s/%s"%(pkg, s_type))
        
    # erase the dirty work we've done
    roslib.msgs.reinit()

    return messages


