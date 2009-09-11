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

# This is a separate front-end for roslaunch that computes the
# command-line arguments for a node. It performs no launching and is
# not related to roslauch itself

import logging
import os
import shlex
import sys
import time

import roslib.packages
import roslib.rosenv
import roslib.substitution_args

from roslaunch.core import setup_env, local_machine, RLException
from roslaunch.config import load_config_default
import roslaunch.xmlloader

## Exception to indicate that node parameters were invalid
class NodeParamsException(Exception): pass

def get_node_list(config):
    l = [_resolved_name(node) for node in config.nodes]
    # filter out unnamed nodes
    return [x for x in l if x]
    
def print_node_list(roslaunch_files):
    try:
        loader = roslaunch.xmlloader.XmlLoader(resolve_anon=False)
        config = load_config_default(roslaunch_files, None, loader=loader, verbose=False)
        node_list = get_node_list(config)
        print '\n'.join(node_list)
    except RLException, e:
        print >> sys.stderr, str(e)
        sys.exit(1)

def print_node_args(node_name, roslaunch_files):
    try:
        args = get_node_args(node_name, roslaunch_files)
        print ' '.join(args)
    except RLException, e:
        print >> sys.stderr, str(e)
        sys.exit(1)
    
def _resolved_name(node):
    if node.name:
        # $(anon id) passthrough
        if node.name.startswith('$'):
            return node.name
        else:
            return roslib.names.ns_join(node.namespace, node.name)
    else:
        return None
            
## Get the node arguments for a node in \a roslaunch_files. NOTE: you
## cannot get the node args for unnamed nodes.
##
## @param node_name str: name of node in \a roslaunch_files.
## @param roslaunch_files [str]: roslaunch file names
## @return [str]: list of command-line arguments used to launch \a node_name
## @throws RLException if node args cannot be retrieved
def get_node_args(node_name, roslaunch_files):
    # we have to create our own XmlLoader so that we can use the same
    # resolution context for substitution args

    loader = roslaunch.xmlloader.XmlLoader(resolve_anon=False)
    config = load_config_default(roslaunch_files, None, loader=loader, verbose=False)
    (node_name) = roslib.substitution_args.resolve_args((node_name), resolve_anon=False)
    node_name = roslib.names.ns_join('/', node_name) if not node_name.startswith('$') else node_name
    
    node = [n for n in config.nodes if _resolved_name(n) == node_name]
    if not node:
        node_list = get_node_list(config)
        node_list_str = '\n'.join([" * %s"%x for x in node_list])
        raise RLException("ERROR: Cannot find node named [%s] in [%s].\nNode names are:\n%s"%(node_name, ', '.join(roslaunch_files), node_list_str))
    elif len(node) > 1:
        raise RLException("ERROR: multiple nodes named [%s] in [%s].\nPlease fix the launch files as duplicate names are not allowed."%(node_name, ', '.join(roslaunch_files)))
    node = node[0]
    
    master_uri = roslib.rosenv.get_master_uri()
    machine = local_machine()

    # don't use create_local_process_env, which merges the env with the full env of the shell
    env = setup_env(node, machine, master_uri)
    to_remove = []
    for k in env.iterkeys():
        if env[k] == os.environ.get(k, None):
            to_remove.append(k)
    for k in to_remove:
        del env[k]
        
    args = create_local_process_args(node, machine)
    # join environment vars are bash prefix args
    return ["%s=%s"%(k, v) for k, v in env.iteritems()] + args
    
## Setup environment for locally launched process. The local
## environment includes the default os environment, with any
## ROS-specific environment variables overriding this enviornment.
## @return dict : environment variables
def create_local_process_env(node, machine, master_uri, env=os.environ):

    # #1029: generate environment for the node. unset
    # #ROS-related environment vars before
    # update() so that extra environment variables don't end
    # up in the call.
    full_env = env.copy()

    for evar in [
        roslib.rosenv.ROS_MASTER_URI,
        roslib.rosenv.ROS_ROOT,
        roslib.rosenv.ROS_PACKAGE_PATH,
        roslib.rosenv.ROS_IP,
        'PYTHONPATH',
        roslib.rosenv.ROS_NAMESPACE]:
        if evar in full_env:
            del full_env[evar]

    proc_env = setup_env(node, machine, master_uri)
    full_env.update(proc_env)
    return full_env

def _launch_prefix_args(node):
    if node.launch_prefix:
        prefix = node.launch_prefix
        if type(prefix) == unicode:
            prefix = prefix.encode('UTF-8')
        return shlex.split(prefix)
    else:
        return []

## subroutine for creating node arguments
## @return list: arguments for node process
## @raise NodeParamsException: if args cannot be constructed for Node
## as specified (e.g. the node type does not exist)
def create_local_process_args(node, machine):
    # - Construct rosrun command
    remap_args = ["%s:=%s"%(src,dst) for src, dst in node.remap_args]
    resolve_dict = {}
    if node.name:
        # - for thie local process args, we *do* resolve the anon tag so that the user can execute
        (node_name) = roslib.substitution_args.resolve_args((node.name), context=resolve_dict, resolve_anon=True)
        remap_args.append('__name:=%s'%node_name)
        
    #resolve args evaluates substitution commands
    #shlex parses a command string into a list of args
    # - for thie local process args, we *do* resolve the anon tag so that the user can execute
    resolved = roslib.substitution_args.resolve_args(node.args, context=resolve_dict, resolve_anon=True)
    if type(resolved) == unicode:
        resolved = resolved.encode('UTF-8') #attempt to force to string for shlex/subprocess
    args = shlex.split(resolved) + remap_args

    start_t = time.time()
    try:
        cmd = roslib.packages.find_node(node.package, node.type,\
                                            machine.ros_root, machine.ros_package_path)
    except roslib.packages.ROSPkgException, e:
        # multiple nodes, invalid package
        raise NodeParamsException(str(e))
    end_t = time.time()
    logging.getLogger('roslaunch.node_args').info('find_node(%s, %s, %s, %s) took %ss'%(node.package, node.type, machine.ros_root, machine.ros_package_path, (end_t - start_t)))
    if not cmd:
        raise NodeParamsException("Cannot locate node of type [%s] in package [%s]"%(node.type, node.package))
    return _launch_prefix_args(node) + [cmd] + args        
    
