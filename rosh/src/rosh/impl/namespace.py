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
Generic namespace model for ROS. This powers most of rosh by enabling
iPython tab-complete on a ROS namespace'd resource (e.g. topics,
services, parameters).
"""

#TODO: remapping support

from __future__ import with_statement

import roslib.names

class Context(object):
    """
    ROSH context instance. Instead of globals, we need a context
    instance that implementation can store state in.
    """
    
    def __init__(self):
        # only guaranteed property
        self.master = None

class NamespaceConfig(object):
    """
    NamespaceConfig is a configuration object for a L{Namespace} instance,
    storing common data structures and objects that all L{Namespace}
    instances need.
    """
    
    def __init__(self, ctx, lock):
        """
        @param ctx: rosh context object
        @type  ctx: dict
        @param lock: lock for controlling access to cache
        """
        self.cache = {}
        self.lock = lock
        self.ctx = ctx
        self.master = ctx.master

class Namespace(object):
    """
    Namespace provides lightweight accessors to a ROS namespace and its data. The behavior of the namespace
    object is determined by a L{NamespaceConfig} instance.
    """
    
    def __init__(self, name, config):
        """
        ctor.
        @param config: Namespace configuration instance. 
        @type  config: L{NamespaceConfig}
        """
        self._name = name
        self._ns = name + '/'
        self._config = config
        
    def _list(self):
        """
        Subclasses should override.
        """
        raise NotImplemented

    def _props(self):
        """
        Subclasses should override if they have props
        """
        return []
    
    def _init(self, *args):
        """
        Subclasses should override.
        """
        raise NotImplemented
    
    def _getAttributeNames(self):
        return list(set([s[len(self._ns):].split('/')[0] for s in self._list()]))
    
    def __getattribute__(self, key):
        if key.startswith('_') or key == 'trait_names':
            return object.__getattribute__(self, key)
        else:
            return self.__getitem__(key)

    def __iter__(self):
        return (getattr(self, k) for k in self._getAttributeNames())

    def _get_entry(self, key):
        """
        By default, creates a new Namespace instance of the correct
        subclass. Subclasses may wish to override.
        """
        cache = self._config.cache
        with self._config.lock:
            if key in cache:
                obj = cache[key]
            else:
                # create a new instance of ourselves. This requires
                # subclasses to have same constructor args.
                obj = self.__class__(key, self._config)
                cache[key] = obj
        return obj

    def __getitem__(self, key):
        """
        Dictionary-style accessor 
        """
        key = roslib.names.ns_join(self._ns, key)
        if key in self._config.cache:
            return self._config.cache[key]
        else:
            val = self._get_entry(key)
            return val
    
class Concept(object):

    def __init__(self, ctx, lock, type_):
        self._master = ctx.master

        self._config = NamespaceConfig(ctx, lock)
        self._root = type_('', self._config)
        
    def _init(self, *args):
        """
        Subclasses should override (if necessary)
        """
        raise NotImplemented

    def _getAttributeNames(self):
        return self._root._getAttributeNames()
        
    def _props(self):
        """
        Subclasses should override if they have props
        """
        return []
    
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
