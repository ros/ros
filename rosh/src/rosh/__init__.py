#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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
ROS Shell (rosh).

Higher-level, interactive scripting environment for ROS.
"""

from __future__ import with_statement
import roslib; roslib.load_manifest('rosh')

# declare shell globals

# ROS Master
master = None
# Detected ROS cameras
cameras = None

# topic, services, and parameters
t = None
topics = None
s = None
services = None
p = None
parameters = None

# - by default, all publishers latch
latching = True

# support libraries, give them private symbols 
import rospy as _rospy
import rostopic as _rostopic
import rosparam as _rosparam
import rosservice as _rosservice
import rosmsg as _rosmsg
import threading as _threading

def _resolve_name(name, ns='/'):
    # TODO:write real resolver. This is just a stub resolver to write against
    return roslib.names.resolve_name(name, ns)

class ROSHException(Exception): pass

class Service(object):

    def __init__(self, name):
        self.type = None
        self.name = name

    # TODO: convert to property
    def type(self):
        if self.type is None:
            self.type = _rosservice.get_service_class_by_name(self.name)
        return self.type._type
        
    def __call__(self, *args, **kwds):
        """
        Call service
        """
        # lazy-init
        if self.type is None:
            self.type = _rosservice.get_service_class_by_name(self.name)
        request = self.type._request_class(*args, **kwds)
        return _rospy.ServiceProxy(self.name, self.type)(request)
    
#TODO: merge back into services, or have it take a reference to the services class to share the same cache
class ServiceNamespace(object):
    
    def __init__(self, ns, cache):
        self.ns = ns
        self._cache = cache
        
    def trait_names(self):
        return list(set([s[len(self.ns):].split('/')[1] for s in _rosservice.get_service_list(namespace=self.ns)]))
    
    def __getattr__(self, key):
        if key in self.__dict__ or key.startswith('_'):
            return object.__getattr__(self, key)
        else:
            return self.__getitem__(key)

    def __iter__(self):
        return _rosservice.get_service_list(namespace=self.ns).__iter__()

    def __getitem__(self, key):
        """
        Dictionary-style accessor for services
        """
        key = roslib.names.ns_join(self.ns, key)
        if key in self._cache:
            return self._cache[key]
        else:
            with _rosh_lock:
                if key in self._cache:
                    obj = self._cache[key]
                elif key in _rosservice.get_service_list():
                    obj = Service(key)
                    self._cache[key] = obj
                else:
                    return ServiceNamespace(key, self._cache)
            return obj
    
class Services(object):
    
    def __init__(self, master):
        self._master = master
        self._cache = {}

    def trait_names(self):
        return list(set([s.split('/')[1] for s in _rosservice.get_service_list()]))
        
    def __iter__(self):
        return _rosservice.get_service_list().__iter__()
        
    def __getattr__(self, key):
        if key in self.__dict__ or key.startswith('_'):
            return object.__getattr__(self, key)
        else:
            return self.__getitem__(key)

    def __getitem__(self, key):
        """
        Dictionary-style accessor for services
        """
        key = _resolve_name(key)
        if key in self._cache:
            return self._cache[key]
        else:
            with _rosh_lock:
                if key in self._cache:
                    obj = self._cache[key]
                elif key in _rosservice.get_service_list():
                    obj = Service(key)
                    self._cache[key] = obj
                else:
                    return ServiceNamespace(key, self._cache)
            return obj

class Parameters(object):
    def __init__(self, master):
        self._master = master

    def __getitem__(self, key):
        return self._master.getParam(_resolve_name(key))

    def __setitem__(self, key, val):
        """
        Set parameter value on Parameter Server
        @param key: parameter key
        @type key: str
        @param val: parameter value
        @type val: XMLRPC legal value
        """
        self._master.setParam(_resolve_name(key), val)
    
# initialize globals
def _init_master():
    global master
    import roslib.masterapi
    master = roslib.masterapi.Master('rosh')
    return master.is_online()

def _init_node():
    _rospy.init_node('rosh', anonymous=True)

def _init_topics():
    global t, topics
    from rosh.topics import Topics
    t = Topics(master, _rosh_lock)
    topics = t
    
def _init_services():
    global s, service
    s = Services(master)
    services = s

def init():
    print 'rosh.init()'
    if _init_master():
        print 'master is', master.getUri()
        _init_node()
        print 'node initialized'
        _init_topics()
        print 'topics initialized'
        #_init_services()
    else:
        raise ROSHException("ERROR: ROS Master is offline")
    
# initialize privates

# - lock for member/global initialization
_rosh_lock = _threading.Lock()

