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
Topic API and implementation for ROSH
"""

from __future__ import with_statement

#from collections import deque
import time
import sys

import rostopic
import rospy

from rosh.namespace import Namespace, NamespaceConfig

class _TopicIdxCallback(object):
    def __init__(self, idx):
        self.count = 0
        self.idx = idx
        self.msg = None
        
    def __call__(self, msg):
        self.count += 1
        if self.count == self.idx:
            self.msg = msg

class _TopicSliceCallback(object):
    
    def __init__(self, start, stop, step, last_msg=None):
        # TODO: figure out if we can yield instead of buffering, which would allow infinite slice
        if stop is None:
            raise ValueError("slice end must be specified")
        
        self.start = start
        self.stop = stop
        self.step = step
        self.count = 0
        self.slice = None
        self.buff = []

        #TODO: figure out semantics of slices starting at 0 if last_msg is not initialized
        if self.start == 0 or self.start == None:
            if last_msg is not None:
                self.buff.append(last_msg)
            else:
                self.start = 1
                self.stop += 1
        
    def __call__(self, msg):
        self.count += 1
        if self.slice is None:
            if self.count%self.step == 0 and self.count >= self.start:
                self.buff.append(msg)
            if self.count >= self.stop:
                self.slice = self.buff
        

            
from contextlib import contextmanager
@contextmanager
def subscribe(ns_obj, cb):
    """
    subscribe/unsubscribe using with statement
    """
    ns_obj._add_subscriber_callback(cb)
    yield
    ns_obj._remove_subscriber_callback(cb)
    
class TopicConfig(object):
    """
    TopicConfig is a configuration object for a L{Topic} instance,
    storing common data structures and objects that all L{Topic}
    instances need.
    """
    
    def __init__(self, master, lock):
        self.cache = {}
        self.master = master
        self.lock = lock

class Topic(object):
    """
    L{Topic} provides lightweight accessors to a ROS topic and its data. The behavior of the namespace
    object is determined by a L{TopicConfig} instance.
    """
    
    def __init__(self, ns, config):
        self._ns = ns
        self._config = config
        self._type_name = None
        self._type = None
        self._sub = None
        self._pub = None
        self._callbacks = []
        self._last_msg = None

    def _list(self,namespace):
        return (v[0] for v in self._config.master.getPublishedTopics(namespace))
    
    def trait_names(self):
        """
        iPython auto-complete
        """
        topic_set = set([s[len(self._ns):].split('/')[0] for s in self._list(self._ns)])
        # precedence is fields before namespaces
        self._init_type()
        if self._type is not None:
            return self._type.__slots__ + list(topic_set)
        else:
            return topic_set

    def _cb(self, msg):
        """Message subscription callback"""
        self._last_msg = msg

    def _init_type(self):
        if self._ns == '/':
            return
        if self._type is None:
            self._type_name = rostopic.get_topic_type(self._ns, blocking=False)[0]
            if self._type_name:
                self._type = roslib.message.get_message_class(self._type_name)
    
    def _init_sub(self):
        """
        Lazy-init subscriber for topic
        """
        # This initialization is fairly fragile as it depends on the
        # type being introspectable.  I'll need to think of a way to
        # allow users to declare the type of a topic, but allow lazy
        # behavior as well.
        if self._sub is None:
            with self._config.lock:
                if self._sub is None:
                    #TODO: change Master APIs so that subscribers declare type
                    print "init_sub"
                    self._init_type()
                    if self._type is None:
                        #TODO: offer corrective advice (e.g. please call init(this, type)
                        raise ROSHException("cannot subscribe to [%s] because type is not known")
                    self._sub = rospy.Subscriber(self._ns, self._type, self._cb)

    def _add_subscriber_callback(self, cb):
        """
        Add callback to be invoked when new messages arrive
        """
        if self._sub is None:
            self._init_sub()
        self._sub.impl.add_callback(cb, None)

    def _remove_subscriber_callback(self, cb):
        self._sub.impl.remove_callback(cb, None)        
    
    def __getattribute__(self, key):
        if key.startswith('_') or key == 'trait_names':
            return object.__getattribute__(self, key)
        else:
            return self.__getitem__(key)

    def __iter__(self):
        return self._list(self._ns).__iter__()

    def __call__(self, *args, **kwds):
        """
        Publish with a topic-style namespace object. Implements call
        functionality for ROSNamespace objects when used with topics.

        @param args: *args
        @param kwds: **kwds
        """
        # lazy-init ns_obj
        if self._pub is None:
            self._init_pub()
        self._pub(*args, **kwds)

    def _slice(self, slice_obj):
        start, stop, step = slice_obj.start, slice_obj.stop, slice_obj.step
        if start > stop:
            raise ValueError("stop index must be greater than end index")
        step = step or 1
        if step < 1:
            raise ValueError("step size must be positive")

        #TODO:
        # test for negative stop
        if start is None or start >= 0:
            cb = _TopicSliceCallback(start, stop, step, self._last_msg)
            with subscribe(self, cb):
                while not cb.slice:
                    time.sleep(0.01) # yield
            return cb.slice
        else:
            #TODO
            #check history buffer
            raise IndexError("[%s] outside of history buffer"%slice_obj)
        
    def _idx(self, idx):
        self._init_sub()
        if idx < 0:
            #TODO
            #check history buffer
            raise IndexError("[%s] outside of history buffer"%idx)
        elif idx == 0:
            return self._last_msg
        else:
            #TODO
            #receive next N (idx) messages and return the last
            print "creating idx callback"
            cb = _TopicIdxCallback(idx)
            with subscribe(self, cb):
                while cb.msg is None:
                    time.sleep(0.01) # yield
            return cb.msg

    def __getitem__(self, key):
        """
        Dictionary-style accessor for services
        """
        #TODO: this is going to need some heavy performance tuning
        #TODO: I'm getting closer to thinking that rosh will require explicit subscribing
        if self._type is None:
            self._init_type()
            
        if type(key) == slice:
            return self._slice(key)
        elif type(key) == int:
            return self._idx(key)
            
        # precedence is fields before namespaces
        #  - get field
        elif self._type and key in self._type.__slots__:
            if self._last_msg:
                return getattr(self._last_msg, key)
            else:
                self._init_sub()
                return None
        # - get namespace
        else:
            key = roslib.names.ns_join(self._ns, key)
            config = self._config
            cache = config.cache

            if key in cache:
                return cache[key]
            else:
                with config.lock:
                    if key in cache:
                        obj = cache[key]
                    else:
                        #print "CREATING", key
                        obj = Topic(key, config)
                        cache[key] = obj
                return obj

class Topics(object):
    
    def __init__(self, master, lock):
        self._master = master
        self._cache = {}
        config = TopicConfig(master, lock)
        self._root = Topic('/', config)

    def trait_names(self):
        return self._root.trait_names()
        
    def __iter__(self):
        return self._root.__iter__()
        
    def __getattribute__(self, key):
        if key.startswith('_') or key == 'trait_names':
            return object.__getattribute__(self, key)
        else:
            return self._root.__getattribute__(key)

    def __getitem__(self, key):
        """
        Dictionary-style accessor for topics
        """
        return self._root.__getitem__(key)

if 0:
    test = Topics(None, None)
    test[:-1]
    test[:5]
    test[1:5]
    test[-1]

# test[:-5] - last five messages, throw error if it exceeds buffer length
