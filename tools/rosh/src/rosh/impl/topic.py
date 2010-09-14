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


import os
import sys
import time
from collections import deque
from threading import Lock

import roslib.message
import roslib.names
from roslib.packages import get_pkg_dir

import rosmsg
import rostopic
import rospy

import rosh.impl.proc
from rosh.impl.exceptions import ROSHException
from rosh.impl.namespace import Namespace, Concept

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
    
    def __init__(self, ns_obj, start, stop, step, last_msg=None):
        if step == 0:
            raise ValueError("slice step cannot be zero")

        self.ns_obj = ns_obj

        # step_off is necessary for doing our step calculation
        # with the quirkiness of last_msg
        self.step_off = 0
        self.start = start
        self.stop = stop
        self.step = step or 1
        self.count = 0
        
        self.done = False
        self.started = False
        
        self.buff = deque()
        self.lock = Lock()

        # Python's behavior with out-of-bounds slices is to silently
        # succeed. This seems to mean that if last_msg is not
        # initialized and the start is 0, we ignore.
        if self.start == 0 or self.start == None:
            if last_msg is not None and self.stop > 0:
                self.started = True
                self.buff.append(last_msg)
                self.step_off = 1
            self.start = 1

        self.step_off = self.step_off - self.start
        if stop is not None and stop <= self.start:
            # note that we switch to the original start idx here to include last_msg
            self.done = True
        
    def __iter__(self):
        with subscribe(self.ns_obj, self):
            while not self.done:
                if self.buff:
                    # deques are threadsafe
                    yield self.buff.popleft()
                time.sleep(0.01) 
        # yield remainder
        while self.buff:
            yield self.buff.popleft()
        
    def __call__(self, msg):
        if not self.done:
            self.count += 1
            if self.count >= self.start:
                if not self.started or (self.count + self.step_off) % self.step == 0:
                    self.started = True
                    self.buff.append(msg)
            if self.stop is not None and self.count >= self.stop - 1:
                self.done = True
            
from contextlib import contextmanager
@contextmanager
def subscribe(ns_obj, cb):
    """
    subscribe/unsubscribe using with statement
    """
    ns_obj._add_subscriber_callback(cb)
    yield
    ns_obj._remove_subscriber_callback(cb)
    
def next_msg(ns_obj, timeout=None):
    """
    Get the next message on the topic

    @param ns_obj: Topic namespace instance
    @type  ns_obj: L{Topic}
    @param timeout: timeout in millis
    @type  timeout: float
    @return: message
    @rtype: roslib.message.Message
    """
    cb = _TopicIdxCallback(1)
    
    if timeout is not None:
        timeout_t = time.time() + timeout
        with subscribe(ns_obj, cb):
            while cb.msg is None and time.time() < timeout_t:
                time.sleep(0.01) # yield
    else:
        with subscribe(ns_obj, cb):
            while cb.msg is None:
                time.sleep(0.01) # yield
    return cb.msg    

class _Relay(object):

    def __init__(self, from_topic, to_topic):
        self.from_topic = from_topic
        self.to_topic   = to_topic

    def __call__(self, msg):
        # check for race condition
        to_topic = self.to_topic
        if to_topic is None:
            return

        # lazy-init new topic
        if to_topic._type_name is None:
            try:
                to_topic._init(msg._type)
            except:
                #TODO: log properly
                print >> sys.stderr, "FAILED to initialize, destroying relay [%s] | [%s]"%(self.from_topic, self.to_topic)
                self._unrelay()

        if msg._type == to_topic._type_name:
            to_topic(msg)
        else:
            #TODO: WARN user once
            pass

    def _unrelay(self):
        if self.from_topic is not None:
            self.from_topic._remove_subscriber_callback(self)
        self.from_topic = self.to_topic = None
        
    def __del__(self):
        self._unrelay()
        
class _FilteredRelay(_Relay):

    def __init__(self, from_topic, to_topic, filter_fn):
        super(_FilteredRelay, self).__init__(from_topic, to_topic)
        self.filter_fn = filter_fn

    def __call__(self, msg):
        msg = self.filter_fn(msg)
        super(_FilteredRelay, self).__call__(msg)

def relay(from_topic, to_topic, filter_fn=None):
    if filter_fn:
        relay_obj = _FilteredRelay(from_topic, to_topic, filter_fn)
    else:
        #TODO: replace with topic_tools launch
        relay_obj = _Relay(from_topic, to_topic)
    from_topic._add_subscriber_callback(relay_obj)
    return relay_obj

def unrelay(relay_obj):
    relay_obj._unrelay()
    
    
class Topic(Namespace):
    """
    L{Topic} provides lightweight accessors to a ROS topic and its data. The behavior of the namespace
    object is determined by a L{TopicConfig} instance.
    """
    
    def __init__(self, ns, config):
        """
        @param config: Namespace configuration instance. 
        @type  config: L{NamespaceConfig}
        """
        super(Topic,self).__init__(ns, config)
        self._type_name = None
        self._type = None
        self._sub = None
        self._pub = None
        self._callbacks = []
        self._last_msg = None
        self._buffer = None
        self._bufferlen = None
        self._buffer_lock = None
        self._latch = True

    def _list(self):
        return (v[0] for v in self._config.master.getPublishedTopics(self._name))

    def _props(self):
        return ['buffer']

    def _set_buffer(self, val):
        """
        Initialize the buffer of this subscriber instance to the
        specified size. This call can have multiple side-effects:
        
         * it will start a subscription to the topic
         * if buffer is already initialized, this call will empty the buffer.

        @param val: length of subscriber callback buffer
        @type  val: int
        """
        self._bufferlen = val
        if val is not None:
            if self._buffer_lock is None:
                self._buffer_lock = Lock()
                
            # - unfortunately 'maxlen' is only supported in Python 2.6
            if self._buffer is not None:
                oldeque = self._buffer
                self._buffer = deque([])
                oldeque.clear()
            else:
                self._buffer = deque([])
        self._init_sub()

    def _show(self):
        """
        Standard Namespace._show() hook. Visualize this topic/namespace.
        """
        #TODO: make this pluggable so that visualizers can be attached to any type name
        if self._type is None:
            self._init_type()
        # any type_name checks must occur before type is None checks
        if self._type_name == 'nav_msgs/OccupancyGrid':
            if self._type is not None:
                show_occ_grid(self)
        elif self._type is None:
            # namespace
            show_graph(self)
        else:
            show_rostopic(self)
        return True
    
    def _getAttributeNames(self):
        """
        iPython auto-complete
        """
        topic_set = set(super(Topic, self)._getAttributeNames())
        # precedence is fields before namespaces
        self._init_type()
        if self._type is not None:
            return self._type.__slots__ + list(topic_set)
        else:
            return topic_set

    def _cb(self, msg):
        """Message subscription callback"""
        self._last_msg = msg
        if self._buffer is not None:
            # have to lock due to potential to slice in separate thread
            b = self._buffer
            with self._buffer_lock:
                while len(b) > self._bufferlen - 1:
                    b.popleft()
                self._buffer.append(msg)

    def __repr__(self):
        return self.__str__()
    
    def __str__(self):
        if self._type_name is None:
            self._init_type()
        if self._type_name is None:
            return self._ns
        else:
            return rosmsg.get_msg_text(self._type_name)
    
    def _init_type(self):
        if self._ns == '/':
            return
        if self._type is None:
            self._type_name = rostopic.get_topic_type(self._ns, blocking=False)[0]
            if self._type_name:
                self._type = roslib.message.get_message_class(self._type_name)
                if self._type is None:
                    pkg, base_type = roslib.names.package_resource_name(self._type_name)
                    print >> sys.stderr, "\ncannot retrieve type [%s].\nPlease type 'rosmake %s'"%(self._type_name, pkg)
    
    def _init_sub(self, block=False):
        """
        Lazy-init subscriber for topic
        """
        # This initialization is fairly fragile as it depends on the
        # type being introspectable.  I'll need to think of a way to
        # allow users to declare the type of a topic, but allow lazy
        # behavior as well.
        while self._sub is None:
            with self._config.lock:
                if self._sub is None:
                    #print "init_sub", self._ns
                    self._init_type()
                    if self._type is None:
                        if not block:
                            raise ROSHException("please init() this instance with the topic type")
                    else:
                        self._sub = rospy.Subscriber(self._ns, self._type, self._cb)
                        return
            if not block:
                return
            time.sleep(0.1)

    def _cleanup(self):
        with self._config.lock:
            if self._pub is not None:
                self._pub.unregister()
            if self._sub is not None:
                self._sub.unregister()
        
    def __delattr__(self):
        self._cleanup()
                
    def _init_pub(self):
        """
        Lazy-init publisher for topic
        """
        # This initialization is fairly fragile as it depends on the
        # type being introspectable.  I'll need to think of a way to
        # allow users to declare the type of a topic, but allow lazy
        # behavior as well.
        if self._pub is None:
            with self._config.lock:
                if self._pub is None:
                    #print "init_pub", self._ns
                    self._init_type()
                    if self._type:
                        self._pub = rospy.Publisher(self._ns, self._type, latch=self._latch)
                    else:
                        raise ROSHException("please init() this instance with the topic type")            

    def _init(self, *args):
        """
        Standard init hook. Must be called with desired topic type. topic type can be a
        string with the message type name, a roslib.message.Message, or a Msg
        """
        if len(args) != 1:
            raise ROSHException("please initialize publisher with message type")
        t = args[0]
        if type(t) == str:
            t = roslib.message.get_message_class(t)
            if t is None:
                raise ValueError("cannot load message type: %s"%(t))
        else:
            if not type(t) == type:
                raise ValueError("invalid message type: %s"%(type(t)))
            if not issubclass(t, roslib.message.Message):
                raise ValueError("invalid message type: %s"%(t.__class__.__name__))

        # finish initalization
        with self._config.lock:
            if self._type is not None and t != self._type:
                self._cleanup()
            self._type = t
            self._type_name = t._type
        
    def _add_subscriber_callback(self, cb):
        """
        Add callback to be invoked when new messages arrive
        """
        if self._sub is None:
            self._init_sub()
        self._sub.impl.add_callback(cb, None)

    def _remove_subscriber_callback(self, cb):
        self._sub.impl.remove_callback(cb, None)        
    
    def __call__(self, *args, **kwds):
        """
        Publish with a topic-style namespace object. Implements call
        functionality for ROSNamespace objects when used with topics.

        @param args: *args
        @param kwds: **kwds
        """
        # lazy-init ns_obj
        if self._pub is None:
            # lazy-init type if user calls with a message class arg
            if len(args) == 1 and isinstance(args[0], roslib.message.Message):
                self._type = args[0].__class__
            self._init_pub()
        self._pub.publish(*args, **kwds)

    def _slice(self, slice_obj):
        self._init_sub(block=True)
        
        start, stop, step = slice_obj.start, slice_obj.stop, slice_obj.step
        step = step or 1
        if step < 1:
            raise ValueError("step size must be positive")

        if start is None or start >= 0:
            if stop is not None and start > stop:
                raise ValueError("stop index must be greater than end index")

            cb = _TopicSliceCallback(self, start, stop, step, self._last_msg)
            return cb
        else:

            # print to console
            if self._buffer is None:
                print >> sys.stderr, "This topic is not buffered. Use props(obj, buffer=N) to enable."
                return []

            # we don't allow this because there are race conditions that are hard to protect against
            if stop is not None and stop > 0:
                raise IndexError("cannot slice from past to future")
            
            # we interpret 0 to mean the last message, so, for the
            # purposes of a slice, it's the end of the history buffer
            if stop == 0:
                stop = None
                
            # we use python's interpretation of start being
            # out-of-bounds, which is to just silently succeed
            # - do this under lock as cb is happening in separate
            #   thread
            with self._buffer_lock:
                retval = list(self._buffer)[start:stop:step]
            return retval
        
    def _idx(self, idx):
        self._init_sub()
        if idx < 0:
            #TODO
            #check history buffer
            raise IndexError("[%s] outside of history buffer"%idx)
        elif idx == 0:
            return self._last_msg
        else:
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
            return super(Topic, self).__getitem__(key)

class Topics(Concept):
    
    def __init__(self, ctx, lock):
        super(Topics, self).__init__(ctx, lock, Topic)
        
    def _show(self):
        show_graph(self._root)

def show_graph(ns_obj):
    success = rosh.impl.proc.run(ns_obj._config, ['rxgraph', '--topicns', ns_obj._ns])
    if success:
        print "showing graph of topics in namespace %s"%ns_obj._ns
    else:
        print >> sys.stderr, "failed to launch rxgraph"
    
def show_occ_grid(ns_obj):
    # internal py_image_view can convert occ grids to images
    mod = os.path.join(get_pkg_dir('rosh'), 'src', 'rosh', 'impl', 'py_image_view.py')
    rosh.impl.proc.run(ns_obj._config, ['python', mod, '--map', '-t', ns_obj._name])
    print "running py_image_viewer, this may be slow over a wireless network"

def show_rostopic(ns_obj):
    # rostopic echo in a separate window
    ros_root = roslib.rosenv.get_ros_root()
    success = rosh.impl.proc.run(ns_obj._config, ['xterm', '-e', os.path.join(ros_root, 'bin', 'rostopic'), 'echo', ns_obj._name], stdout=False)
    if success:
        print "running rostopic echo in a separate window"
    else:
        print >> sys.stderr, "unable to run rostopic echo in a separate window"
