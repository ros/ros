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

import roslib
from rosh.impl.exceptions import ROSHException

# TODO: possibly store other data in here, like plugin help, etc...
class PluginContext(object):

    def __init__(self, rosh_globals, rosh_context, rosh_lock):
        # global symbol table for rosh module
        self.rosh_globals = rosh_globals
        # namespace ctx object of rosh
        self.ctx = rosh_context
        # common lock object for initialization
        self.rosh_lock = rosh_lock

def reentrant_load(loaded_symbols, globals_):
    """
    Utility routine to allow plugins that have already loaded to
    initialize a new globals_ dictionary.
    """
    if loaded_symbols is not None:
        if globals_ is not None:
            globals_.update(loaded_symbols)
        return True
    else:
        return False

def globals_load(plugin_context, globals_, symbols):
    """
    Utility routine for plugins to load their symbols into the
    appropriate global symbol tables
    """
    for g in [plugin_context.rosh_globals, globals_]:
        if g is not None:
            g.update(symbols)
    
_plugin_friendly = {
    'geometry': 'rosh_geometry', 'tf': 'rosh_geometry', 'transforms': 'rosh_geometry',
    'cameras': 'rosh_common', 'actions': 'rosh_common' }
def load_plugin(name, plugin_context, globals_=None):
    """
    Load ROSH plugin from another ROS package.

    load_plugin() loads the plugin into the rosh global symbol
    table. If globals_ is specified, plugin symbols will also be
    loaded into the provided dictionary.

    @param globals_: global symbol table to additionally load plugin to.
    @type  globals_: dict
    """
    
    # remap friendly names to actual implementation. this is a bit of
    # abstraction leakage and should instead be dynamically registered
    # (though much more expensive).
    name = _plugin_friendly.get(name, name)
    
    try:
        roslib.load_manifest(name)
    except roslib.packages.InvalidROSPkgException, e:
        raise ROSHException("invalid plugin [%s]: no ROS package named [%s]"%(name, name))
    try:
        m = __import__(name)
    except ImportError:
        raise ROSHException("invalid plugin [%s]: python module [%s] import failed"%(name, name))
    try:
        loader = getattr(m, 'rosh_plugin_load')
    except AttributeError, e:
        raise ROSHException("invalid plugin [%s]: plugin is missing rosh_plugin_load() entry point"%(name))
    
    plugin_data = loader(plugin_context, globals_)
    if plugin_data is not None:
        # TODO: allow plugin to register topic type handlers
        pass

