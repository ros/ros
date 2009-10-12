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

import os
import sys

import roslib.exceptions
from roslib.rosenv import ROS_NAMESPACE

#TODO: why are these here?
MSG_EXT = '.msg'
SRV_EXT = '.srv'

#TODO: deprecate PRN_SEPARATOR
PRN_SEPARATOR = '/'
TYPE_SEPARATOR = PRN_SEPARATOR #alias
SEP = '/'
GLOBALNS = '/'
PRIV_NAME = '~'
REMAP = ":="
ANYTYPE = '*'

## Base exception type for errors in roslib.names routines
class ROSNameException(roslib.exceptions.ROSLibException): pass

## @return str: ROS namespace of current program
def get_ros_namespace(environ=os.environ):
    #we force command-line-specified namespaces to be globally scoped 
    return make_global_ns(environ.get(ROS_NAMESPACE, GLOBALNS))

## Resolve a local name to the caller ID based on ROS environment settings (i.e. ROS_NAMESPACE)
## @param name str: local name to calculate caller ID from, e.g. 'camera', 'node'
## @return str: caller ID based on supplied local \a name
def make_caller_id(name):
    return make_global_ns(ns_join(get_ros_namespace(), name))

## @param name str: ROS resource name. Cannot be a ~name.
## @return str: \a name as a global name, e.g. 'foo' -> '/foo/'.
## This does NOT resolve a name.
## @throws ROSNameException if \a name is a ~name
def make_global_ns(name):
    if is_private(name):
        raise ROSNameException("cannot turn [%s] into a global name"%name)
    if not is_global(name):
        name = SEP + name
    if name[-1] != SEP:
        name = name + SEP
    return name

## @param name str: must be a legal name in canonical form
## @return bool: True if \a name is a globally referenced name (i.e. /ns/name)
def is_global(name):
    return name and name[0] == SEP

## @param name str: must be a legal name in canonical form
## @return bool: True if \a name is a privately referenced name (i.e. ~name)
def is_private(name):
    return name and name[0] == PRIV_NAME

## Get the namespace of name. The namespace is returned with a
## trailing slash in order to favor easy concatenation and easier use
## within the global context.
##     
## @param name str: name to return the namespace of. Must be a legal
## name. NOTE: an empty name will return the global namespace.
## @return str: Namespace of name. For example, '/wg/node1' returns '/wg/'. The
## global namespace is '/'. 
## @throws ValueError if \a name is invalid
def namespace(name):
    "map name to its namespace"
    if name is None: 
        raise ValueError('name')
    if not isinstance(name, basestring):
        raise TypeError('name')
    if not name:
        return SEP
    elif name[-1] == SEP:
        name = name[:-1]
    return name[:name.rfind(SEP)+1] or SEP

## Join a namespace and name. If name is unjoinable (i.e. ~private or
## /global) it will be returned without joining
##
## @param ns str: namespace ('/' and '~' are both legal)
## @param name str: a legal name
## @return str: \a name concatenated to \a ns, or \a name if it is
## unjoinable.
def ns_join(ns, name):
    if is_private(name) or is_global(name):
        return name
    if ns == PRIV_NAME:
        return PRIV_NAME + name
    if ns[-1] == SEP:
        return ns + name
    return ns + SEP + name

## Load name mappings encoded in command-line arguments. This will filter
## out any parameter assignment mappings (see roslib.param.load_param_mappings()).
## @param argv [str]: command-line arguments
## @return dict {str: str}: name->name remappings. 
def load_mappings(argv):
    mappings = {}
    for arg in argv:
        if REMAP in arg:
            try:
                src, dst = [x.strip() for x in arg.split(REMAP)]
                if src and dst:
                    if len(src) > 1 and src[0] == '_' and src[1] != '_':
                        #ignore parameter assignment mappings
                        pass
                    else:
                        mappings[src] = dst
            except:
                print >> sys.stderr, "ERROR: Invalid remapping argument '%s'"%arg
    return mappings

#######################################################################
# RESOURCE NAMES
# resource names refer to entities in a file system

## Convert package name + resource into a fully qualified resource name
## @param res_pkg_name str: name of package resource is located in
## @param name str: resource base name
## @param my_pkg str: name of package resource is being referred to
## in. If specified, name will be returned in local form if \a
## res_pkg_name is \a my_pkg
## @return str: name for resource 
def resource_name(res_pkg_name, name, my_pkg=None):
    if res_pkg_name != my_pkg:
        return res_pkg_name+PRN_SEPARATOR+name
    return name

## Convert fully qualified resource name into the package-less resource name
## @param name str: package resource name, e.g. 'std_msgs/String'
## @return str: resource name sans package-name scope
def resource_name_base(name):
    """pkg/typeName -> typeName, typeName -> typeName"""
    return name[name.rfind(PRN_SEPARATOR)+1:]

## @param name str: package resource name, e.g. 'std_msgs/String'
## @return str: package name of resource
def resource_name_package(name):
    """pkg/typeName -> pkg, typeName -> None"""
    if not PRN_SEPARATOR in name:
        return None
    return name[:name.find(PRN_SEPARATOR)]

## Split a name into its package and resource name parts, e.g. 'std_msgs/String -> std_msgs, String'
## @param name str: package resource name, e.g. 'std_msgs/String'
## @return str: package name, resource name
## @raise ROSNameException if \a name is invalid
def package_resource_name(name):
    if PRN_SEPARATOR in name:
        val = tuple(name.split(PRN_SEPARATOR))
        if len(val) != 2:
            raise ROSNameException("invalid name [%s]"%name)
        else:
            return val
    else:
        return '', name

def _is_safe_name(name, type_name):
    #windows long-file name length is 255
    if not isinstance(name, basestring) or not name or len(name) > 255:
        return False
    return is_legal_resource_name(name)

def is_valid_local_name(name):
    return _is_safe_name(name, 'name')
    
# TODO: redo this, this is very old
import re
NAME_LEGAL_CHARS_P = re.compile('^[A-Za-z][\w_\/]*$') #ascii char followed by (alphanumeric, _, /)
def is_legal_resource_name(name):
    if not name: #None or empty
        return False
    if len(name) != len(name.strip()):
        return False
    return NAME_LEGAL_CHARS_P.match(name) is not None
# same legal rules 
is_legal_name = is_legal_resource_name
