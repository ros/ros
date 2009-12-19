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
Utilities for accessing the Parameter Server
"""

import sys
import xmlrpclib

import roslib.rosenv
from roslib.names import REMAP

_param_server = None

def load_command_line_node_params(argv):
    """
    Load node param mappings (aka private parameters) encoded in
    command-line arguments, e.g. _foo:=bar. See also roslib.names.load_mappings.
    @param argv: command-line arguments
    @param argv: [str]
    @return: param->value remappings. 
    @rtype: {str: val}
    """    
    try:
        import yaml
    except ImportError, e:
        print >> sys.stderr, "Cannot import yaml. Please make sure the pyyaml system dependency is installed"
        raise e
    mappings = {}
    for arg in argv:
        if REMAP in arg:
            try:
                src, dst = [x.strip() for x in arg.split(REMAP)]
                if src and dst:
                    if len(src) > 1 and src[0] == '_' and src[1] != '_':
                        mappings[src[1:]] = yaml.load(dst)
            except:
                print >> sys.stderr, "ERROR: Invalid remapping argument '%s'"%arg
    return mappings

def get_param(key):
    """
    Retrieve parameter value from the Parameter Server. Each call to this
    routine results in an actual network call to the Parameter Server.
    Client must be an actual ROS node in order to implement a parameter cache.

    @param key: name of parameter to fetch
    @type  key: str
    @raise KeyError: if parameter is not set
    """
    global _param_server
    if _param_server is None:
        _param_server = xmlrpclib.ServerProxy(roslib.rosenv.get_master_uri())
    code, status, value = _param_server.getParam('/roslib', key)
    if code != 1: #unwrap value with Python semantics
        raise KeyError(key)
    return value    
            
