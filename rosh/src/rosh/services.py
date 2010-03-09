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

from __future__ import with_statement

import rosservice

from rosh.namespace import Namespace, NamespaceConfig

def service_list_fn(namespace):
    return rosservice.get_service_list(namespace=namespace)

def service_call_fn(ns_obj, args, kwds):
    """
    Implements call functionality for ROSNamespace objects when used with services
    """
    # lazy-init ns_obj
    if ns_obj._type is None:
        ns_obj._type = rosservice.get_service_class_by_name(ns_obj._ns)
    request = self.type._request_class(*args, **kwds)
    return _rospy.ServiceProxy(self.name, self.type)(request)

def service_slice_fn():
    raise TypeError("services do not support numeric indexing")

class Services(object):
    
    def __init__(self, master, lock):
        self._master = master
        self._cache = {}
        config = NamespaceConfig(master, lock, service_list_fn, service_call_fn)
        self._root = Namespace('/', config)

    def trait_names(self):
        return list(set([s.split('/')[1] for s in rosservice.get_service_list()]))
        
    def __iter__(self):
        return self._root.__iter__()
        
    def __getattr__(self, key):
        if key in self.__dict__ or key.startswith('_'):
            return object.__getattr__(self, key)
        else:
            return self._root.__getattr__(key)

    def __getitem__(self, key):
        """
        Dictionary-style accessor for services
        """
        return self._root.__getitem__(key)

