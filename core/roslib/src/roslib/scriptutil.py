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
# Revision $Id: scriptutil.py 3544 2009-01-24 00:09:21Z sfkwc $
# $Author: sfkwc $

## Common ros script utilities, such as methods convenience methods
## for creating master xmlrpc proxies and executing rospack. Use of
## these utilities is highly encouraged as they will isolate code from
## future potential architectural reworkings.

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
import roslib.packages
import roslib.rosenv 

PRODUCT = 'ros'

## caller ID for master calls where caller ID is not vital
_GLOBAL_CALLER_ID = '/script'

## Name resolver for scripts. Supports ROS_NAMESPACE.  Does not
## support remapping arguments.
## @param name str: name to resolve
## @param script_name str: name of script. script_name must not
## contain a namespace.
## @return str: resolved name
def script_resolve_name(script_name, name):
    if not name: #empty string resolves to namespace
        return roslib.names.get_ros_namespace()
    #Check for global name: /foo/name resolves to /foo/name
    if roslib.names.is_global(name):
        return name
    #Check for private name: ~name resolves to /caller_id/name
    elif roslib.names.is_private(name):
        return ns_join(roslib.names.make_caller_id(script_name), name[1:])
    return roslib.names.get_ros_namespace() + name

## @return str: result of executing command (via subprocess). string will be strip()ed.
def rospackexec(args):
    val = (subprocess.Popen(['rospack'] + args, stdout=subprocess.PIPE).communicate()[0] or '').strip()
    if val.startswith('rospack:'): #rospack error message
        raise Exception(val)
    return val

## @return list: A list of the names of the packages which depend directly on pkg
def rospack_depends_on_1(pkg):
    return rospackexec(['depends-on1', pkg]).split()

## @return list: A list of the names of the packages which depend on pkg
def rospack_depends_on(pkg):
    return rospackexec(['depends-on', pkg]).split()

## @return list: A list of the names of the packages which pkg directly depends on
def rospack_depends_1(pkg):
    return rospackexec(['deps1', pkg]).split()

## @return list: A list of the names of the packages which pkg depends on
def rospack_depends(pkg):
    return rospackexec(['deps', pkg]).split()

## @return list: A list of the names of the packages which provide a plugin for pkg
def rospack_plugins(pkg):
    val = rospackexec(['plugins', '--attrib=plugin', pkg])
    if val:
      return [tuple(x.split(' ')) for x in val.split('\n')]
    else:
      return []


## @return str: result of executing command (via subprocess). string will be strip()ed.
def rosstackexec(args):
    val = (subprocess.Popen(['rosstack'] + args, stdout=subprocess.PIPE).communicate()[0] or '').strip()
    if val.startswith('rosstack:'): #rospack error message
        raise Exception(val)
    return val

## @return list: A list of the names of the packages which depend directly on pkg
def rosstack_depends_on_1(s):
    return rosstackexec(['depends-on1', s]).split()

## @return ServerProxy XML-RPC proxy to ROS master
def get_master():
    import xmlrpclib
    return xmlrpclib.ServerProxy(roslib.rosenv.get_master_uri())

## @return ServerProxy XML-RPC proxy to ROS parameter server
def get_param_server():
    return get_master()

## @deprecated
get_message_class = roslib.message.get_message_class

## @deprecated
get_service_class = roslib.message.get_service_class

## check whether or not master think subscriber_id subscribes to topic
## @return bool: True if still register as a subscriber
## @throws roslib.exceptions.ROSLibException: if communication with master fails
def is_subscriber(topic, subscriber_id):
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

## predicate to check whether or not master think publisher_id
## publishes topic
## @return bool: True if still register as a publisher
## @throws roslib.exceptions.ROSLibException: if communication with master fails
def is_publisher(topic, publisher_id):
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

