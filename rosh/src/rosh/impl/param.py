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

import roslib.packages
import rosparam as _rosparam
import rospy

from rosh.impl.exceptions import ROSHException
from rosh.impl.namespace import Namespace, Concept

def rosparam(*args):
    """
    Load rosparam YAML file. This can either be called with
    rosparam(filename) or rosparam(pkg, filename)
    
    @return: loaded data
    """
    #TODO: should this do a search to find filename
    if len(args) == 2:
        pkg, filename = args
        d = roslib.packages.get_pkg_dir(pkg)
        matches = roslib.packages.find_resource(pkg, filename)
        if matches:
            filename = matches[0]
        else:
            filename = None
    elif len(args) == 1:
        filename = args[0]
    else:
        raise ValueError(args)
    
    if not filename:
        #TODO: what should the error behavior of ROSH be?
        return None

    #TODO: this strip any namespace in the file
    return _rosparam.load_file(filename)[0][0]

def rosparam_str(yaml_str):
    """
    Load rosparam YAML string 
    @return: loaded data
    """
    return _rosparam.load_str(yaml_str, 'rosh')
    
#TODO: delete parameter
class Param(Namespace):

    def __init__(self, name, config):
        """
        ctor.
        @param config: Namespace configuration instance. 
        @type  config: L{NamespaceConfig}
        """
        super(Param, self).__init__(name, config)
        self._master = config.master

    def _list(self):
        """
        Override Namespace._list()
        """
        try:
            val = self._master.getParam(self._ns)
            return [self._ns+k for k in val.iterkeys()]
        except:
            return []

    def __repr__(self):
        return self.__str__()
        
    def __str__(self):
        try:
            return str(self._master.getParam(self._ns))
        except:
            return '<uninitialized>'

    def __setitem__(self, key, value):
        if key.startswith('_'):
            return object.__setitem__(key, value)
        else:
            key = roslib.names.ns_join(self._ns, key)
            try:
                self._master.setParam(key, value)
                return True
            except:
                return False
        
    def __delitem__(self, key):
        if key.startswith('_'):
            return object.__delitem__(key)
        else:
            key = roslib.names.ns_join(self._ns, key)
            try:
                self._master.deleteParam(key)
                return True
            except:
                raise KeyError(key)

    def __setattr__(self, key, value):
        if key.startswith('_'):
            return object.__setattr__(self, key, value)
        else:
            return self.__setitem__(key, value)
        
    def __delattr__(self, key):
        if key.startswith('_'):
            return object.__delattr__(self, key)
        else:
            return self.__delitem__(key)

    def __contains__(self, key):
        return self._master.hasParam(key)

    def __call__(self):
        """
        @return: current parameter value
        """
        try:
            return self._master.getParam(self._ns)
        except:
            raise KeyError(self._ns)
    
class Parameters(Concept):

    def __init__(self, ctx, lock):
        super(Parameters, self).__init__(ctx, lock, Param)
        
    def __setattr__(self, key, value):
        if key.startswith('_'):
            return object.__setattr__(self, key, value)
        else:
            return self._root.__setitem__(key, value)
        
    def __call__(self):
        return self._root.__call__()

    def __delitem__(self, key):
        if key.startswith('_'):
            return object.__delitem__(key)
        else:
            return self._root.__delitem__(key)

    def __delattr__(self, key):
        if key.startswith('_'):
            return object.__delattr__(self, key)
        else:
            return self._root.__delitem__(key)

