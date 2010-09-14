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

import os
import sys

import roslib.packages
import roslib.rosenv

# declare shell globals

# These are initialized by rosh.impl.ros_graph but declared here due
# to get_*() accessors
nodes = topics = services = parameters = msg = srv = None

# - by default, all publishers latch
latching = True

import rosh.plugin
import rosh.impl.ros_graph
import rosh.impl.ipy
import rosh.impl.namespace

from rosh.impl.show import show
from rosh.impl.props import props
import rosh.impl.bagy as bagy

# NOTE: this shadows the rosparam module. Is this an issue?
from rosh.impl.param import rosparam, rosparam_str

# client symbols
from rosh.impl.exceptions import ROSHException
from rosh.impl.library import findros
import rosh.impl.bagy as bagy
from rosh.impl.service import service

# TODO: reimplement relay using topic_tools node plus roslaunch
# in fact, mux is probably better than relay because we can keep reconfiguring it.
#from rosh.impl.topic import relay

from roslaunch.core import Node

def init(*args):
    """
    initialize an instance, e.g. init(topics.rosout, msg.rosout.Log)
    """
    if len(args) < 1:
        raise ValueError("init() must be called with an object to initialize and the initialization args")
    obj = args[0]
    obj._init(*args[1:])
    
def get_topics():
    return topics
    
def get_services():
    return services
    
def get_parameters():
    return parameters

def get_msg():
    return msg

def get_srv():
    return srv

# TODO: move elsewhere
def launch(launchable, type_=None, args='', remap={}):
    """
    @param launchable: ROS package name or Node instance
    @type  launchable: str or Node
    @param type_: node type (executable name). If None, defaults to value of pkg parameter.
    @type  type_: str
    @param args: node arguments
    @type  args: str
    @param remap: remapping arguments
    @type  remap: dict
    """
    import rosh.impl.proc
    if type(launchable) == str:
        pkg = launchable
        if type_ is None:
            type_ = pkg
        n = rosh.impl.proc.launch(_ctx, pkg, type_, args, remap=remap)
    elif type(launchable) == Node:
        n = launchable
        if args:
            n.args = args
        if remap:
            raise NotImplemented
        if type_:
            raise ValueError("launch of Node instance does not accept additional args")
        n = rosh.impl.proc.launch_node(_ctx, n)
    # right now, ignore p and instance return the node reference
    # TODO: split apart namespaces
    return nodes[n.name]
    
def kill(obj):
    if hasattr(obj, '_kill'):
        obj._kill()
    elif type(obj) in (list, tuple):
        for x in obj:
            kill(x)
    else:
        raise ValueError("Not a kill-able object")

def load(name, globals_=None):
    """
    Load plugin. load() loads the plugin into the rosh global symbol
    table. If globals_ is specified, plugin symbols will also be
    loaded into the provided dictionary.

    @param globals_: global symbol table to additionally load plugin to.
    @type  globals_: dict
    """
    rosh.plugin.load_plugin(name, _plugin_context, globals_)
        
_plugin_context = None
def get_default_plugin_context():
    return _plugin_context

def ok():
    """
    @return: True if ok to keep executing, False if script should exit.
    """
    return rosh.impl.ros_graph.ok()

def rosh_init():
    # context is the heart of the rosh namespace logic
    global _ctx, _rosh_lock

    _ctx = rosh.impl.namespace.Context()

    import threading
    _rosh_lock = threading.RLock()    

    # plugin context is the loading mechanism for ROSH plugins
    global _plugin_context
    _plugin_context = rosh.plugin.PluginContext(globals(), _ctx, _rosh_lock)

    # load symbols for ROS graph layer
    rosh.impl.ros_graph.load_rosh_plugin('rosh.impl.ros_graph', _plugin_context)

    # initialize IPython Magic
    rosh.impl.ipy.ipy_magic_init()

# initialize privates

# lock for member/global initialization
_rosh_lock = None
# ROSH shared context
ctx = None

