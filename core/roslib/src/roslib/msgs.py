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
# Revision $Id: msgspec.py 3357 2009-01-13 07:13:05Z jfaustwg $
# $Author: jfaustwg $
"""
ROS msg library for Python

Implements: U{http://ros.org/wiki/msg}
"""

import cStringIO
import os
import itertools
import sys
import re
import string

import roslib.exceptions
import roslib.manifest
import roslib.packages
import roslib.names
import roslib.resources

VERBOSE = False

## @return: True if msg-related scripts should print verbose output
def is_verbose():
    return VERBOSE

## set whether msg-related scripts should print verbose output
def set_verbose(v):
    global VERBOSE
    VERBOSE = v

EXT = roslib.names.MSG_EXT #alias
SEP = roslib.names.PRN_SEPARATOR #e.g. std_msgs/String
## character that designates a constant assignment rather than a field
CONSTCHAR   = '='
COMMENTCHAR = '#'

class MsgSpecException(roslib.exceptions.ROSLibException): pass

#TODOXXX: unit test
## Compute the base data type, e.g. for arrays, get the underlying array item type
## @param type_ str: ROS msg type (e.g. 'std_msgs/String')
## @return str: base type
def base_msg_type(type_):
    if type_ is None:
        return None
    if '[' in type_:
        return type_[:type_.find('[')]
    return type_

#NOTE: this assumes that we aren't going to support multi-dimensional

## Parse ROS message field type
## @param type_ str: ROS field type
## @return str, bool, int: base_type, is_array, array_length
## @throws MsgSpecException if \a type_ cannot be parsed
def parse_type(type_):
    if not type_:
        raise MsgSpecException("Invalid empty type")
    if '[' in type_:
        var_length = type_.endswith('[]')
        splits = type_.split('[')
        if len(splits) > 2:
            raise MsgSpecException("Currently only support 1-dimensional array types: %s"%type_)
        if var_length:
            return type_[:-2], True, None
        else:
            try:
                length = int(splits[1][:-1])
                return splits[0], True, length
            except ValueError:
                raise MsgSpecException("Invalid array dimension: [%s]"%splits[1][:-1])
    else:
        return type_, False, None
   
################################################################################
# name validation 

## @return bool: True if the name is a syntatically legal message type name
def is_valid_msg_type(x):
    if not x or len(x) != len(x.strip()):
        return False
    base = base_msg_type(x)
    if not roslib.names.is_valid_local_name(base):
        return False
    #parse array indicies
    x = x[len(base):]
    state = 0
    i = 0
    for c in x:
        if state == 0:
            if c != '[':
                return False
            state = 1 #open
        elif state == 1:
            if c == ']':
                state = 0 #closed
            else:
                try:
                    string.atoi(c)
                except:
                    return False
    return state == 0

## @return bool: True if the name is a legal constant type. Only simple types are allowed.
def is_valid_constant_type(x):
    return x in PRIMITIVE_TYPES

## @return bool: True if the name is a syntatically legal message field name
def is_valid_msg_field_name(x):
    return roslib.names.is_valid_local_name(x)

# msg spec representation ##########################################

## Container class for holding a Constant declaration
class Constant(object):
    __slots__ = ['type', 'name', 'val', 'val_text']
    
    ## @param self
    ## @param type str 
    ## @param name str
    ## @param val str
    ## @param val_text str Original text definition of \a val
    def __init__(self, type_, name, val, val_text):
        if type is None or name is None or val is None or val_text is None:
            raise ValueError('Constant must have non-None parameters')
        self.type = type_
        self.name = name.strip() #names are always stripped of whitespace
        self.val = val
        self.val_text = val_text

    def __eq__(self, other):
        if not isinstance(other, Constant):
            return False
        return self.type == other.type and self.name == other.name and self.val == other.val

    def __repr__(self):
        return "%s %s=%s"%(self.type, self.name, self.val)

    def __str__(self):
        return "%s %s=%s"%(self.type, self.name, self.val)

## Convert spec into a string representation. Helper routine for MsgSpec.
## @param indent str: internal use only
## @param buff StringIO: internal use only
## @return str: string representation of spec
def _strify_spec(spec, buff=None, indent=''):
    if buff is None:
        buff = cStringIO.StringIO()
    for c in spec.constants:
        buff.write("%s%s %s=%s\n"%(indent, c.type, c.name, c.val_text))
    for type_, name in zip(spec.types, spec.names):
        buff.write("%s%s %s\n"%(indent, type_, name))
        base_type = base_msg_type(type_)
        if not base_type in BUILTIN_TYPES:
            subspec = get_registered(base_type)
            _strify_spec(subspec, buff, indent + '  ')
    return buff.getvalue()

## Container class for storing loaded msg description files. Field
## types and names are stored in separate lists with 1-to-1
## correspondence. MsgSpec can also return an md5 of the source text.
class MsgSpec(object):

    ## @param self
    ## @param types [str]: list of field types, in order of declaration
    ## @param names [str]: list of field names, in order of declaration    
    ## @param constants [Constant]: Constant declarations
    ## @param text str: text of declaration
    ## @throws MsgSpecException if spec is invalid (e.g. fields with the same name)
    def __init__(self, types, names, constants, text):
        self.types = types
        if len(set(names)) != len(names):
            raise MsgSpecException("Duplicate field names in message: %s"%names)
        self.names = names
        self.constants = constants
        assert len(self.types) == len(self.names), "len(%s) != len(%s)"%(self.types, self.names)
        #Header.msg support
        self.header_present = (HEADER, 'header') in zip(self.types, self.names)
        self.text = text
        
    ## @param self
    ## @return [(str,str),]: zip list of types and names (e.g. [('int32', 'x'), ('int32', 'y')]
    def fields(self):
        return zip(self.types, self.names)

    ## @param self
    ## @return true if msg decription contains a 'Header header'
    ## declaration at the beginning
    def has_header(self):
        return self.header_present
    def __eq__(self, other):
        if not other or not isinstance(other, MsgSpec):
            return False 
        return self.types == other.types and self.names == other.names and \
               self.constants == other.constants and self.text == other.text
    def __ne__(self, other):
        if not other or not isinstance(other, MsgSpec):
            return True
        return not self.__eq__(other)

    def __repr__(self):
        return "MsgSpec[%s, %s]"%(repr(self.types), repr(self.names))

    def __str__(self):
        return _strify_spec(self)
    
# msg spec loading utilities ##########################################

## reinitialize roslib.msgs. This API is for message generators
## (e.g. genpy) that need to re-initialize the registration table.
def reinit():
    global _initialized , _loaded_packages
    # unset the initialized state and unregister everything 
    _initialized = False
    del _loaded_packages[:]
    REGISTERED_TYPES.clear()
    _init()
    
_initialized = False
def _init():
    #lazy-init
    global _initialized
    if _initialized:
        return

    fname = '%s%s'%(HEADER, EXT)
    roslib_dir = roslib.packages.get_pkg_dir('roslib')
    if roslib_dir is None:
        raise MsgSpecException("Unable to locate roslib: %s files cannot be loaded"%EXT)
    
    header = os.path.join(roslib_dir, roslib.packages.MSG_DIR, fname)
    if not os.path.isfile(header):
        print >> sys.stderr, "ERROR: cannot locate %s. Excepted to find it at '%s'"%(fname, header)
        return False

    # register Header under both contexted and de-contexted name
    _, spec = load_from_file(header, '')
    register(HEADER, spec)
    register('roslib/'+HEADER, spec)    
    for k, spec in EXTENDED_BUILTINS.iteritems():
        register(k, spec)
        
    _initialized = True

# .msg file routines ################################################################    

## @internal
## predicate for filtering directory list. matches message files
def _msg_filter(f):
    return os.path.isfile(f) and f.endswith(EXT)

# also used by doxymaker
## list all messages in the specified package
## @param package str: name of package to search
## @param include_depends bool: if True, will also list messages in package dependencies
## @return [str]: message type names
def list_msg_types(package, include_depends):
    types = roslib.resources.list_package_resources(package, include_depends, roslib.packages.MSG_DIR, _msg_filter)
    return [x[:-len(EXT)] for x in types]

## Determine the file system path for the specified .msg
## resource. .msg resource does not have to exist.
## @param package str: name of package .msg file is in
## @param type str: type name of message, e.g. 'Point2DFloat32'
## @return str: file path of .msg file in specified package
def msg_file(package, type):
    return roslib.packages.resource_file(package, roslib.packages.MSG_DIR, type+EXT)

## List all messages that a package contains
## @param depend Depend: manifest.Depend object representing package
## to load messages from
## @param name_context str: package prefix for message type names
## @return [(str,MsgSpec), [str]]: list of message type names and specs for package, as well as a list
##     of message names that could not be processed. 
def get_pkg_msg_specs(depend, name_context):
    _init()
    types = list_msg_types(depend.package, False)
    specs = [] #no fancy list comprehension as we want to show errors
    failures = []
    for t in types:
        try: 
            typespec = load_from_file(msg_file(depend.package, t), name_context)
            specs.append(typespec)
        except Exception, e:
            failures.append(t)
            print "ERROR: unable to load %s"%t
    return specs, failures

## Register all messages that the specified package depends on.
def load_package_dependencies(package):
    global _loaded_packages
    _init()    
    p = roslib.manifest.Depend(package)
    if VERBOSE:
        print "Load dependencies for package", p
        
    manifest_file = roslib.manifest.manifest_file(package, True)
    m = roslib.manifest.parse_file(manifest_file)
    depends = m.depends[:] # #391
    if not p in depends:
        depends.append(p)
    msgs = []
    failures = []
    for d in depends:
        if VERBOSE:
            print "Load dependency", d
        #check if already loaded
        # - we are dependent on manifest.getAll returning first-order dependencies first
        if d.package in _loaded_packages or d.package == package: 
            continue
        _loaded_packages.append(d.package)
        if d != p:
            context = d.package
        else:
            context = ''
        specs, failed = get_pkg_msg_specs(d, context)
        msgs.extend(specs)
        failures.extend(failed)
    for key, spec in msgs:
        register(key, spec)

## Load package into the local registered namespace. All messages found
#  in the package will be registered if they are successfully
#  loaded. This should only be done with one package (i.e. the 'main'
#  package) per Python instance.
def load_package(package):
    global _loaded_packages
    _init()    
    p = roslib.manifest.Depend(package)
    if VERBOSE:
        print "Load package", p
        
    #check if already loaded
    # - we are dependent on manifest.getAll returning first-order dependencies first
    if package in _loaded_packages:
        if VERBOSE:
            print "Package %s is already loaded"%p
        return

    _loaded_packages.append(package)
    specs, failed = get_pkg_msg_specs(p, '')
    if VERBOSE:
        print "Package contains the following messages: %s"%specs
    for key, spec in specs:
        #register spec under both local and fully-qualified key
        register(key, spec)
        register(package + roslib.names.PRN_SEPARATOR + key, spec)        

## @internal
## convert constant value declaration to python value. Does not do
## type-checking, so ValueError or other exceptions may be raised.
## @param type_ str: ROS field type
## @param val str: string representation of constant
## @raise ValueError if unable to convert to python representation
## @raise MsgSpecException if value exceeds specified integer width
def _convert_val(type_, val):
    if type_ in ['float32','float64']:
        return float(val)
    elif type_ in ['string']:
        return val.strip() #string constants are always stripped 
    elif type_ in ['int8', 'uint8', 'int16','uint16','int32','uint32','int64','uint64', 'char', 'byte']:
        # bounds checking
        bits = [('int8', 8), ('uint8', 8), ('int16', 16),('uint16', 16),\
                ('int32', 32),('uint32', 32), ('int64', 64),('uint64', 64),\
                ('byte', 8), ('char', 8)]
        b = [b for t, b in bits if t == type_][0]
        import math
        if type_[0] == 'u' or type_ == 'char':
            lower = 0
            upper = int(math.pow(2, b)-1)
        else:
            upper = int(math.pow(2, b-1)-1)   
            lower = -upper - 1 #two's complement min
        val = int(val) #python will autocast to long if necessary
        if val > upper or val < lower:
            raise MsgSpecException("cannot coerce [%s] to %s (out of bounds)"%(val, type_))
        return val
    else if type == 'bool':
        # TODO: need to nail down constant spec for bool
        return True if eval(val) else False
    raise MsgSpecException("invalid constant type: [%s]"%type_)
        
## Load message specification for specified type
#  @param package_context: package name to use for the type name or
#      '' to use the local (relative) naming convention.
#  @return (str, L{MsgSpec}): Message type name and message specification
def load_by_type(msgtype, package_context=''):
    pkg, basetype = roslib.names.package_resource_name(msgtype)
    pkg = pkg or package_context # convert '' -> local package
    try:
        m_f = msg_file(pkg, basetype)
    except roslib.packages.InvalidROSPkgException:
        raise MsgSpecException("Cannot locate message type [%s], package does not exist"%msgtype) 
    return load_from_file(m_f, pkg)

## Load message specification from a string.
#  @param text str: .msg text 
#  @param package_context: package name to use for the type name or
#      '' to use the local (relative) naming convention.
#  @return MsgSpec: Message message specification
#  @throws MsgSpecException: if syntax errors or other problems are detected in file
def load_from_string(text, package_context=''):
    types = []
    names = []
    constants = []
    for orig_line in text.split('\n'):
        l = orig_line.split(COMMENTCHAR)[0].strip() #strip comments
        if not l:
            continue #ignore empty lines
        splits = filter(lambda s: s, [x.strip() for x in l.split(" ")]) #split type/name, filter out empties
        type_ = splits[0]
        if not is_valid_msg_type(type_):
            raise MsgSpecException("%s is not a legal message type"%type_)
        if CONSTCHAR in l:
            if not is_valid_constant_type(type_):
                raise MsgSpecException("%s is not a legal constant type"%type_)
            if type_ == 'string':
                # strings contain anything to the right of the equals sign, there are no comments allowed
                idx = orig_line.find(CONSTCHAR)
                name = orig_line[orig_line.find(' ')+1:idx]
                val = orig_line[idx+1:]
            else:
                splits = [x.strip() for x in ' '.join(splits[1:]).split(CONSTCHAR)] #resplit on '='
                if len(splits) != 2:
                    raise MsgSpecException("Invalid declaration: %s"%l)
                name = splits[0]
                val = splits[1]
            try:
                val_converted  = _convert_val(type_, val)
            except Exception, e:
                raise MsgSpecException("Invalid declaration: %s"%e)
            constants.append(Constant(type_, name, val_converted, val.strip()))
        else:
            if len(splits) != 2:
                raise MsgSpecException("Invalid declaration: %s"%l)
            name = splits[1]
            if not is_valid_msg_field_name(name):
                raise MsgSpecException("%s is not a legal message field name"%name)
            if package_context and not SEP in type_:
                if not base_msg_type(type_) in RESERVED_TYPES:
                    #print "rewrite", type_, "to", "%s/%s"%(package_context, type_)
                    type_ = "%s/%s"%(package_context, type_)
            types.append(type_)
            names.append(name)
    return MsgSpec(types, names, constants, text)

## Convert the .msg representation in the file to a MsgSpec instance.
#  This does *not* register the object.
#  @param file_path str: path of file to load from
#  @param package_context str: package name to prepend to type name or
#    '' to use local (relative) naming convention.
#  @return (str, L{MsgSpec}): Message type name and message specification
#  @throws MsgSpecException: if syntax errors or other problems are detected in file
def load_from_file(file_path, package_context=''):
    if VERBOSE:
        if package_context:
            print "Load spec from", file_path, "into package [%s]"%package_context
        else:
            print "Load spec from", file_path
    fileName = os.path.basename(file_path)
    type = fileName[:-len(EXT)]
    # determine the type name
    if package_context:
        while package_context.endswith(SEP):
            package_context = package_context[:-1] #strip message separators
        type = "%s%s%s"%(package_context, SEP, type)
    if not roslib.names.is_valid_local_name(type):
        raise MsgSpecException("%s: %s is not a legal type name"%(file_path, type))
    
    f = open(file_path, 'r')
    try:
        try:
            text = f.read()
            return (type, load_from_string(text, package_context))
        except MsgSpecException, e:
            raise MsgSpecException('%s: %s'%(fileName, e))
    finally:
        f.close()

# data structures and builtins specification ###########################

# adjustable constants, in case we change our minds
HEADER   = 'Header'
TIME     = 'time'
DURATION = 'duration'

## @param type_ str: message type name
## @return bool: True if \a type_ refers to the ROS Header type
def is_header_type(type_):
    return type_ in [roslib.msgs.HEADER, 'roslib/Header']
        
# time and duration types are represented as aggregate data structures
# for the purposes of serialization from the perspective of
# roslib.msgs. genmsg_py will do additional special handling is required
# to convert them into rospy.msg.Time/Duration instances.

## time as msg spec. time is unsigned 
TIME_MSG     = "uint32 secs\nuint32 nsecs"
## duration as msg spec. duration is just like time except signed
DURATION_MSG = "int32 secs\nint32 nsecs"

## extended builtins are builtin types that can be represented as MsgSpec instances
EXTENDED_BUILTINS = { TIME : load_from_string(TIME_MSG), DURATION: load_from_string(DURATION_MSG) }

## primitive types are those for which we allow constants, i.e. have  primitive representation
PRIMITIVE_TYPES = ['int8','uint8','int16','uint16','int32','uint32','int64','uint64','float32','float64',
                   'string',
                   'bool',
                   # deprecated:
                   'char','byte']
BUILTIN_TYPES = PRIMITIVE_TYPES + EXTENDED_BUILTINS.keys()

RESERVED_TYPES  = BUILTIN_TYPES + [HEADER]

REGISTERED_TYPES = { } 
_loaded_packages = [] #keep track of packages so that we only load once (note: bug #59)

## @param msg_type_name str: name of message type
## @return bool: True if \a msg_type_name is a builtin/primitive type
def is_builtin(msg_type_name):
    return msg_type_name in BUILTIN_TYPES

## @param msg_type_name str: name of message type
## @return bool: True if msg spec for specified msg type name is
## registered. NOTE: builtin types are not registered.
def is_registered(msg_type_name):
    return REGISTERED_TYPES.has_key(msg_type_name)

## @param msg_type_name str: name of message type
## @return MsgSpec: msg spec for msg type name
def get_registered(msg_type_name, default_package=None):
    if msg_type_name in REGISTERED_TYPES:
        return REGISTERED_TYPES[msg_type_name]
    elif default_package:
        # if msg_type_name has no package specifier, try with default package resolution
        p, n = roslib.names.package_resource_name(msg_type_name)
        if not p:
            return REGISTERED_TYPES[roslib.names.resource_name(default_package, msg_type_name)]
    raise KeyError(msg_type_name)

## Load MsgSpec into the type dictionary
## @param msg_type_name str: name of message type
## @param msg_spec MsgSpec: spec to load
def register(msg_type_name, msg_spec):
    if VERBOSE:
        print "Register msg %s"%msg_type_name
    REGISTERED_TYPES[msg_type_name] = msg_spec

