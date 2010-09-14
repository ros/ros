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

import sys

import roslib.packages

import rosh
from rosh.impl.exceptions import ROSHException
import rosh.impl.ros_graph

def create_globals(plugin_context, dependencies):
    """
    Generate global symbol table for roshlet

    @param plugin_context: context for loading plugins that roshlet depends on
    @type  plugin_context: [str]
    @param dependencies: names of plugins that roshlet depends on
    @type  dependencies: [str]
    """

    rosh = sys.modules['rosh']
    # TODO: need to have an __all__ table instead so as to not load in extra dependencies
    globals_ = rosh.__dict__.copy()
    globals_['rosh'] = rosh

    # load in default symbols
    # TODO: need common init routine for rosh so that this does not diverge
    rosh.impl.ros_graph.load_rosh_plugin('rosh.impl.ros_graph', plugin_context, globals_)

    for d in dependencies:
        rosh.plugin.load_plugin(d, plugin_context, globals_)
    return globals_

def load_roshlet(filename, plugin_context, dependencies):
    """
    @param filename: name of file with roshlet code
    @type  filename: str
    @param plugin_context: context for loading plugins that roshlet depends on
    @type  plugin_context: [str]
    @param dependencies: names of plugins that roshlet depends on
    @type  dependencies: [str]
    """
    with open(filename) as f:
        str = f.read()
    return load_roshlet_str(str, plugin_context, dependencies, filename=filename)

def load_roshlet_str(str, plugin_context, dependencies, filename="input"):
    """
    @param str: roshlet code
    @type  str: str
    @param plugin_context: context for loading plugins that roshlet depends on
    @type  plugin_context: [str]
    @param dependencies: names of plugins that roshlet depends on
    @type  dependencies: [str]
    """
    globals_ = create_globals(plugin_context, dependencies)
    try:
        c = compile(str, filename, 'exec')
    except SyntaxError, e:
        raise ROSHException("roshlet [%s] has syntax errors: \n%s"%(f, e))
    except TypeError, e:
        raise ROSHException("roshlet [%s] source contains null bytes: \n%s"%(f, e))

    return c, globals_
    
def exec_roshlet(compiled, globals_):
    exec compiled in globals_

def find_roshlet(package, type_):
    """
    @raise roslib.packages.InvalidROSPkgException: If package does not exist
    @raise IOError: If roshlet cannot be located
    """
    script_candidates = roslib.packages.find_resource(package, type_)
    if not script_candidates:
        raise IOError("cannot find roshlet [%s] in package [%s]"%(type_, package))
    # - for now, assume it's the first match
    return script_candidates[0]
    
# TODO: possibly consider executing roshlets at a particular rate
def standalone(name, script, plugins):
    """
    Execute roshlet standalone. This method blocks until roshlet exits.

    @param name: roshlet name
    @type  name: str
    @param package: roshlet package
    @type  package: str
    @param type_: roshlet type
    @type  type_: str
    @param plugins: list of roshlet plugins to load for roshlet
    @type  plugins: [str]
    @raise roslib.packages.InvalidROSPkgException: If package does not exist
    @raise IOError: If roshlet cannot be located
    """

    # load rosh symbols
    rosh.rosh_init()

    # Load roshlet and execute blocking in this thread
    pc = rosh.get_default_plugin_context()
    args = rosh.impl.roshlets.load_roshlet(script, pc, plugins)
    rosh.impl.roshlets.exec_roshlet(*args)
    rosh.serve_forever()
