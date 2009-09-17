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

"""
Support for ROS Names

See: U{http://www.ros.org/wiki/Names}
"""

import sys
import os
from itertools import ifilter

from roslib.names import namespace, get_ros_namespace, ns_join, make_global_ns, load_mappings, \
     SEP, GLOBALNS, TYPE_SEPARATOR, REMAP, ANYTYPE, \
     is_global, is_private

from rospy.exceptions import ROSException
from rospy.validators import ParameterInvalid

TOPIC_ANYTYPE = ANYTYPE #indicates that a subscriber will connect any datatype given to it
SERVICE_ANYTYPE = ANYTYPE #indicates that a service client does not have a fixed type

def canonicalize_name(name):
    """
    Put name in canonical form. Double slashes '//' are removed and
    name is returned without any trailing slash, e.g. /foo/bar
    @param name: ROS name
    @type name: str
    """
    if not name or name == SEP:
        return name
    elif name[0] == SEP:
        return '/' + '/'.join([x for x in name.split(SEP) if x])
    else:
        return '/'.join([x for x in name.split(SEP) if x])        
    ##if len(name) > 1 and name[-1] == SEP:
    ##    return name[:-1]
    ##return name

# Mappings override name resolution by substituting fully-qualified
# names in for local name references. They override any name
# reference, with exception of '.local' names. We load remapping args
# as soon as client API is referenced so that they are initialized
# before Topic constructors are invoked.
_mappings = load_mappings(sys.argv)

def get_mappings():
    """
    @return: command-line remappings {name: name}
    @rtype: {str: str}
    """
    return _mappings


def resolve_name(name, caller_id=None, remap=True):
    """
    Resolve a ROS name to its global, canonical form. Private ~names
    are resolved relative to the node name. 

    @param name: name to resolve.
    @type name: str
    @param caller_id: caller_id to resolve name relative to. To
    resolve to local namespace, omit this parameter (or use None)
    @type caller_id: str
    @param remap: If False, remapping is turned off. This is mainly
    used to prevent circular remappings.
    @type remap: bool
    @return: Resolved name. If name is empty/None, resolve_name
    returns parent namespace. If namespace is empty/None,
    @rtype: str
    """
    if not caller_id:
        caller_id = get_caller_id()
    if not name: #empty string resolves to namespace
        return namespace(caller_id)

    #Mappings override general namespace-based resolution
    # - do this before canonicalization as remappings are meant to
    #   match the name as specified in the code
    if remap and name in _mappings:
        return resolve_name(_mappings[name], caller_id, remap=False)
    name = canonicalize_name(name)
    #Check for global name: /foo/name resolves to /foo/name
    if name[0] == SEP: 
        return name
    #Check for private name: ~name resolves to /caller_id/name
    elif is_private(name):
        return ns_join(caller_id, name[1:])
    return namespace(caller_id) + name

def remap_name(name, caller_id=None):
    """
    Remap a ROS name. This API should be used to instead of
    resolve_name for APIs in which you don't wish to resolve the name
    unless it is remapped.
    @param name: name to remap
    @type name: str
    @return: Remapped name
    @rtype: str
    """
    if not caller_id:
        caller_id = get_caller_id()
    if name in _mappings:
        return resolve_name(_mappings[name], caller_id, remap=False)
    return name

def scoped_name(caller_id, name):
    """
    Convert the global caller_id to a relative name within the namespace. For example, for
    namespace '/foo' and name '/foo/bar/name', the return value will
    be 'bar/name'

    WARNING: scoped_name does not validate that name is actually within
    the supplied namespace.
    @param caller_id: caller ID, in canonical form
    @type caller_id: str
    @param name: name to scope
    @type name: str
    @return: name scoped to the caller_id's namespace. 
    @rtype: str
    """
    if not is_global(caller_id):
        raise ROSException("caller_id must be global")
    return canonicalize_name(name)[len(namespace(caller_id)):]


###################################################
# Name validators      ############################

#Technically XMLRPC will never send a None, but I don't want to code masterslave.py to be
#XML-RPC specific in this way.

def empty_or_valid_name(param_name):
    """
    empty or valid graph resource name.
    Validator that resolves names unless they an empty string is supplied, in which case
    an empty string is returned.
    """
    def validator(param_value, caller_id):
        if not isinstance(param_value, basestring):
            raise ParameterInvalid("ERROR: parameter [%s] must be a string"%param_name)              
        if not param_value:
            return ''
        #return resolve_name(param_value, namespace(caller_id))
        return resolve_name(param_value, caller_id)
    return validator

def valid_name_validator_resolved(param_name, param_value, caller_id):
    if not param_value or not isinstance(param_value, basestring):
        raise ParameterInvalid("ERROR: parameter [%s] must be a non-empty string"%param_name)            
    #TODO: actual validation of chars
    # I added the colon check as the common error will be to send an URI instead of name
    if ':' in param_value or ' ' in param_value:
        raise ParameterInvalid("ERROR: parameter [%s] contains illegal chars"%param_name) 
    #return resolve_name(param_value, namespace(caller_id))
    return resolve_name(param_value, caller_id)
def valid_name_validator_unresolved(param_name, param_value, caller_id):
    if not param_value or not isinstance(param_value, basestring):
        raise ParameterInvalid("ERROR: parameter [%s] must be a non-empty string"%param_name)            
    #TODO: actual validation of chars        
    # I added the colon check as the common error will be to send an URI instead of name
    if ':' in param_value or ' ' in param_value:
        raise ParameterInvalid("ERROR: parameter [%s] contains illegal chars"%param_name) 
    return param_value
    
def valid_name(param_name, resolve=True):
    """
    Validator that resolves names and also ensures that they are not empty
    @param param_name: name
    @type param_name: str
    @param resolve: if True/omitted, the name will be resolved to
       a global form. Otherwise, no resolution occurs.
    @type resolve: bool
    @return: resolved parameter value
    @rtype: str
    """
    def validator(param_value, caller_id):
        if resolve:
            return valid_name_validator_resolved(param_name, param_value, caller_id)
        return valid_name_validator_unresolved(param_name, param_value, caller_id)        
    return validator

def global_name(param_name):
    """"
    Validator that checks for valid, global graph resource name.
    @return: parameter value
    @rtype: str
    """
    def validator(param_value, caller_id):
        if not param_value or not isinstance(param_value, basestring):
            raise ParameterInvalid("ERROR: parameter [%s] must be a non-empty string"%param_name)
        #TODO: actual validation of chars
        if not is_global(param_value):
            raise ParameterInvalid("ERROR: parameter [%s] must be a globally referenced name"%param_name)            
        return param_value
    return validator

def valid_type_name(param_name):
    """validator that checks the type name is specified correctly"""
    def validator(param_value, caller_id):
        if param_value == TOPIC_ANYTYPE:
            return param_value
        if not param_value or not isinstance(param_value, basestring):
            raise ParameterInvalid("ERROR: parameter [%s] must be a non-empty string"%param_name)            
        if not len(param_value.split(TYPE_SEPARATOR)) == 2:
            raise ParameterInvalid("ERROR: parameter [%s] is not a valid package resource name"%param_name)
        #TODO: actual validation of chars
        return param_value
    return validator


#########################################################
#Global Namespace Routines
# - Global state, e.g. singletons and namespace

_caller_namespace = get_ros_namespace()
_caller_id = _caller_namespace+'unnamed' #default for non-node. 

def get_namespace():
    """
    Get namespace of local node. 
    @return: fully-qualified name of local node or '' if not applicable
    @rtype: str
    """
    return _caller_namespace

def get_caller_id():
    """
    Get fully resolved name of local node. If this is not a node,
    use empty string
    @return: fully-qualified name of local node or '' if not applicable
    @rtype: str
    """    
    return _caller_id

def _set_caller_id(caller_id):
    """
    Internal API.
    Set the global name (i.e. caller_id) and namespace. Methods can
    check what the name of the current node is by calling get_caller_id.

    The caller_id is important as it is the first parameter to any API
    call on a remote node.  Invoked by ROSNode constructor
    @param caller_id: new caller ID
    @type caller_id: str
    """    
    global _caller_id, _caller_namespace
    _caller_id = caller_id
    _caller_namespace = namespace(caller_id)

