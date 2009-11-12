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
# $Author$

"""
Common ros script utilities, such as methods convenience methods for
creating master xmlrpc proxies and executing rospack. This library
is relatively immature and much of the functionality here will
likely be moved elsewhere as the API solidifies.
"""

import itertools
import os
import re
import string
import subprocess
import sys

import roslib.exceptions
import roslib.launcher
import roslib.message
import roslib.msgs 
import roslib.names 
import roslib.network
import roslib.packages
import roslib.rosenv 

PRODUCT = 'ros'

## caller ID for master calls where caller ID is not vital
_GLOBAL_CALLER_ID = '/script'


import warnings
def deprecated(func):
    """This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emmitted
    when the function is used."""
    def newFunc(*args, **kwargs):
        warnings.warn("Call to deprecated function %s." % func.__name__,
                      category=DeprecationWarning, stacklevel=2)
        return func(*args, **kwargs)
    newFunc.__name__ = func.__name__
    newFunc.__doc__ = func.__doc__
    newFunc.__dict__.update(func.__dict__)
    return newFunc

def myargv(argv=None):
    """
    Remove ROS remapping arguments from sys.argv arguments.
    @return: copy of sys.argv with ROS remapping arguments removed
    @rtype: [str]
    """
    if argv is None:
        argv = sys.argv
    return [a for a in argv if not roslib.names.REMAP in a]

def script_resolve_name(script_name, name):
    """
    Name resolver for scripts. Supports ROS_NAMESPACE.  Does not
    support remapping arguments.
    @param name: name to resolve
    @type  name: str
    @param script_name: name of script. script_name must not
    contain a namespace.
    @type  script_name: str
    @return: resolved name
    @rtype: str
    """
    if not name: #empty string resolves to namespace
        return roslib.names.get_ros_namespace()
    #Check for global name: /foo/name resolves to /foo/name
    if roslib.names.is_global(name):
        return name
    #Check for private name: ~name resolves to /caller_id/name
    elif roslib.names.is_private(name):
        return ns_join(roslib.names.make_caller_id(script_name), name[1:])
    return roslib.names.get_ros_namespace() + name

@deprecated
def rospackexec(args):
    """
    @return: result of executing rospack command (via subprocess). string will be strip()ed.
    @rtype: str
    @raise roslib.exceptions.ROSLibException: if rospack command fails
    """

    val = (subprocess.Popen(['rospack'] + args, stdout=subprocess.PIPE).communicate()[0] or '').strip()
    if val.startswith('rospack:'): #rospack error message
        raise roslib.exceptions.ROSLibException(val)
    return val

@deprecated
def rospack_depends_on_1(pkg):
    """
    @param pkg: package name
    @type  pkg: str
    @return: A list of the names of the packages which depend directly on pkg
    @rtype: list
    """
    return rospackexec(['depends-on1', pkg]).split()

@deprecated
def rospack_depends_on(pkg):
    """
    @param pkg: package name
    @type  pkg: str
    @return: A list of the names of the packages which depend on pkg
    @rtype: list
    """
    return rospackexec(['depends-on', pkg]).split()

@deprecated
def rospack_depends_1(pkg):
    """
    @param pkg: package name
    @type  pkg: str
    @return: A list of the names of the packages which pkg directly depends on
    @rtype: list    
    """
    return rospackexec(['deps1', pkg]).split()

@deprecated
def rospack_depends(pkg):
    """
    @param pkg: package name
    @type  pkg: str
    @return: A list of the names of the packages which pkg depends on
    @rtype: list    
    """
    return rospackexec(['deps', pkg]).split()

@deprecated
def rospack_plugins(pkg):
    """
    @param pkg: package name
    @type  pkg: str
    @return: A list of the names of the packages which provide a plugin for pkg
    @rtype: list    
    """
    val = rospackexec(['plugins', '--attrib=plugin', pkg])
    if val:
      return [tuple(x.split(' ')) for x in val.split('\n')]
    else:
      return []

@deprecated
def rosstackexec(args):
    """
    @return: result of executing rosstack command (via subprocess). string will be strip()ed.
    @rtype:  str
    @raise roslib.exceptions.ROSLibException: if rosstack command fails
    """
    val = (subprocess.Popen(['rosstack'] + args, stdout=subprocess.PIPE).communicate()[0] or '').strip()
    if val.startswith('rosstack:'): #rospack error message
        raise Exception(val)
    return val

@deprecated
def rosstack_depends_on(s):
    """
    @param s: stack name
    @type  s: str
    @return: A list of the names of the stacks which depend on s
    @rtype: list
    """
    return rosstackexec(['depends-on', s]).split()

@deprecated
def rosstack_depends_on_1(s):
    """
    @param s: stack name
    @type  s: str
    @return: A list of the names of the stacks which depend directly on s
    @rtype: list
    """
    return rosstackexec(['depends-on1', s]).split()

@deprecated
def rosstack_depends(s):
    """
    @param s: stack name
    @type  s: str
    @return: A list of the names of the stacks which s depends on 
    @rtype: list
    """
    return rosstackexec(['depends', s]).split()

@deprecated
def rosstack_depends_1(s):
    """
    @param s: stack name
    @type  s: str
    @return: A list of the names of the stacks which s depends on directly
    @rtype: list
    """
    return rosstackexec(['depends1', s]).split()

def get_master():
    """
    @return: XML-RPC proxy to ROS master
    @rtype: xmlrpclib.ServerProxy
    """
    import xmlrpclib
    # #1730 validate URL for better error messages
    uri = roslib.rosenv.get_master_uri()
    try:
        roslib.network.parse_http_host_and_port(uri)
    except ValueError:
        raise roslib.exceptions.ROSLibException("invalid master URI: %s"%uri)
    return xmlrpclib.ServerProxy(uri)


def get_param_server():
    """
    @return: ServerProxy XML-RPC proxy to ROS parameter server
    @rtype: xmlrpclib.ServerProxy
    """
    return get_master()

## @deprecated
get_message_class = roslib.message.get_message_class

## @deprecated
get_service_class = roslib.message.get_service_class

def is_subscriber(topic, subscriber_id):
    """
    Check whether or not master think subscriber_id subscribes to topic
    @return: True if still register as a subscriber
    @rtype: bool
    @raise roslib.exceptions.ROSLibException: if communication with master fails
    """
    m = get_master()
    code, msg, state = m.getSystemState(_GLOBAL_CALLER_ID)
    if code != 1:
        raise roslib.exceptions.ROSLibException("Unable to retrieve master state: %s"%msg)
    _, subscribers, _ = state
    for t, l in subscribers:
        if t == topic:
            return subscriber_id in l
    else:
        return False

def is_publisher(topic, publisher_id):
    """
    Predicate to check whether or not master think publisher_id
    publishes topic
    @return: True if still register as a publisher
    @rtype: bool
    @raise roslib.exceptions.ROSLibException: if communication with master fails
    """
    m = get_master()
    code, msg, state = m.getSystemState(_GLOBAL_CALLER_ID)
    if code != 1:
        raise roslib.exceptions.ROSLibException("Unable to retrieve master state: %s"%msg)
    pubs, _, _ = state
    for t, l in pubs:
        if t == topic:
            return publisher_id in l
    else:
        return False

