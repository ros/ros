# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
Utility module of roslaunch that computes the command-line arguments
for a node.
"""

import logging
import os
import shlex
import sys
import time

import rospkg
import rosgraph
import rosgraph.names
from rosgraph.names import script_resolve_name

import roslib.packages

from . import substitution_args

from roslaunch.core import setup_env, local_machine, RLException
from roslaunch.config import load_config_default
import roslaunch.xmlloader

class NodeParamsException(Exception):
    """
    Exception to indicate that node parameters were invalid
    """
    pass

def get_node_list(config):
    """
    @param config: roslaunch config
    @type  config: ROSLaunchConfig
    @return: list of node names in config
    @rtype: [str]
    """
    l = [_resolved_name(node) for node in config.nodes] + [_resolved_name(test) for test in config.tests]
    # filter out unnamed nodes
    return [x for x in l if x]

def print_node_list(roslaunch_files):
    """
    Print list of nodes to screen. Will cause system exit if exception
    occurs. This is a subroutine for the roslaunch main handler.

    @param roslaunch_files: list of launch files to load
    @type  roslaunch_files: str
    """
    try:
        loader = roslaunch.xmlloader.XmlLoader(resolve_anon=False)
        config = load_config_default(roslaunch_files, None, loader=loader, verbose=False, assign_machines=False)
        node_list = get_node_list(config)
        print '\n'.join(node_list)
    except RLException as e:
        print >> sys.stderr, str(e)
        sys.exit(1)

def print_node_args(node_name, roslaunch_files):
    """
    Print arguments of node to screen. Will cause system exit if
    exception occurs. This is a subroutine for the roslaunch main
    handler.
    
    @param node_name: node name
    @type  node_name: str
    @param roslaunch_files: list of launch files to load
    @type  roslaunch_files: str
    """
    try:
        node_name = script_resolve_name('roslaunch', node_name)
        args = get_node_args(node_name, roslaunch_files)
        print ' '.join(args)
    except RLException as e:
        print >> sys.stderr, str(e)
        sys.exit(1)
    
def _resolved_name(node):
    if node.name:
        # $(anon id) passthrough
        if node.name.startswith('$'):
            return node.name
        else:
            return rosgraph.names.ns_join(node.namespace, node.name)
    else:
        return None

def print_node_filename(node_name, roslaunch_files):
    try:
        # #2309
        node_name = script_resolve_name('roslaunch', node_name)
        
        loader = roslaunch.xmlloader.XmlLoader(resolve_anon=False)
        config = load_config_default(roslaunch_files, None, loader=loader, verbose=False, assign_machines=False)
        nodes = [n for n in config.nodes if _resolved_name(n) == node_name] + \
            [t for t in config.tests if _resolved_name(t) == node_name]

        if len(nodes) > 1:
            raise RLException("ERROR: multiple nodes named [%s] in [%s].\nPlease fix the launch files as duplicate names are not allowed."%(node_name, ', '.join(roslaunch_files)))
        if not nodes:
            print >> sys.stderr, 'ERROR: cannot find node named [%s]. Run \n\troslaunch --nodes <files>\nto see list of node names.'%(node_name)
        else:
            print nodes[0].filename
        
    except RLException as e:
        print >> sys.stderr, str(e)
        sys.exit(1)

def get_node_args(node_name, roslaunch_files):
    """
    Get the node arguments for a node in roslaunch_files. 

    @param node_name: name of node in roslaunch_files.
    @type  node_name: str
    @param roslaunch_files: roslaunch file names
    @type  roslaunch_files: [str]
    @return: list of command-line arguments used to launch node_name
    @rtype: [str]
    @raise RLException: if node args cannot be retrieved
    """
    
    # we have to create our own XmlLoader so that we can use the same
    # resolution context for substitution args

    loader = roslaunch.xmlloader.XmlLoader(resolve_anon=False)
    config = load_config_default(roslaunch_files, None, loader=loader, verbose=False, assign_machines=False)
    (node_name) = substitution_args.resolve_args((node_name), resolve_anon=False)
    node_name = script_resolve_name('roslaunch', node_name) if not node_name.startswith('$') else node_name
    
    node = [n for n in config.nodes if _resolved_name(n) == node_name] + \
        [n for n in config.tests if _resolved_name(n) == node_name]
    if not node:
        node_list = get_node_list(config)
        node_list_str = '\n'.join([" * %s"%x for x in node_list])
        raise RLException("ERROR: Cannot find node named [%s] in [%s].\nNode names are:\n%s"%(node_name, ', '.join(roslaunch_files), node_list_str))
    elif len(node) > 1:
        raise RLException("ERROR: multiple nodes named [%s] in [%s].\nPlease fix the launch files as duplicate names are not allowed."%(node_name, ', '.join(roslaunch_files)))
    node = node[0]
    
    master_uri = rosgraph.get_master_uri()
    machine = local_machine()
    env = setup_env(node, machine, master_uri)

    # remove setting identical to current environment for easier debugging
    to_remove = []
    for k in env.iterkeys():
        if env[k] == os.environ.get(k, None):
            to_remove.append(k)
    for k in to_remove:
        del env[k]

    # resolve node name for generating args
    args = create_local_process_args(node, machine)
    # join environment vars are bash prefix args
    return ["%s=%s"%(k, v) for k, v in env.iteritems()] + args
    
def _launch_prefix_args(node):
    if node.launch_prefix:
        prefix = node.launch_prefix
        if type(prefix) == unicode:
            prefix = prefix.encode('UTF-8')
        return shlex.split(prefix)
    else:
        return []

def create_local_process_args(node, machine, env=None):
    """
    Subroutine for creating node arguments.

    :param env: override os.environ.  Warning, this does not override
      substitution args in node configuration (for now), ``dict``
    :returns: arguments for node process, ``[str]``
    :raises: :exc:`NodeParamsException` If args cannot be constructed for Node
      as specified (e.g. the node type does not exist)
    """
    if not node.name:
        raise ValueError("node name must be defined")
    if env is None:
        env = os.environ
    
    # - Construct rosrun command
    remap_args = ["%s:=%s"%(src,dst) for src, dst in node.remap_args]
    resolve_dict = {}

    #resolve args evaluates substitution commands
    #shlex parses a command string into a list of args
    # - for the local process args, we *do* resolve the anon tag so that the user can execute
    # - the node name and args must be resolved together in case the args refer to the anon node name
    (node_name) = substitution_args.resolve_args((node.name), context=resolve_dict, resolve_anon=True)
    node.name = node_name
    remap_args.append('__name:=%s'%node_name)
        
    resolved = substitution_args.resolve_args(node.args, context=resolve_dict, resolve_anon=True)
    if type(resolved) == unicode:
        resolved = resolved.encode('UTF-8') #attempt to force to string for shlex/subprocess
    args = shlex.split(resolved) + remap_args
    try:
        #TODO:fuerte: pass through rospack and catkin cache
        rospack = rospkg.RosPack(rospkg.get_ros_paths(env=env))
        matches = roslib.packages.find_node(node.package, node.type, rospack)
    except rospkg.ResourceNotFound as e:
        # multiple nodes, invalid package
        raise NodeParamsException(str(e))
    if not matches:
        raise NodeParamsException("can't locate node [%s] in package [%s]"%(node.type, node.package))
    else:
        # old behavior was to take first, do we want to change this in Fuerte-style?
        cmd = matches[0]
    if not cmd:
        raise NodeParamsException("Cannot locate node of type [%s] in package [%s]"%(node.type, node.package))
    return _launch_prefix_args(node) + [cmd] + args        
    
