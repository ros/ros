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
L{WtfContext} object, which is commonly used throughout the roswtf
APIs to pass state.
"""

import os
import sys

import rospkg
import rospkg.environment

import rosgraph

import roslaunch.depends
import roslaunch.substitution_args

from roswtf.model import WtfWarning

class WtfException(Exception):
    """
    Base exception class of roswtf-related issues.
    """
    pass

class WtfContext(object):
    """
    WtfContext stores common state about the ROS filesystem and online
    environment. The primary use of this is for convenience (not
    having to load this state manually) and performance (not having to
    do the same calculation repeatedly).
    """
    __slots__ = ['pkg', 'pkg_dir', 'pkgs',
                 'stack', 'stack_dir', 'stacks',
                 'manifest_file', 'manifest',
                 'env', 'ros_root', 'ros_package_path', 'pythonpath',
                 'ros_master_uri',
                 'roslaunch_uris',
                 'launch_files',
                 'launch_file_deps',
                 'launch_file_missing_deps',
                 'system_state',
                 'service_providers',
                 'topics', 'services',
                 'nodes', 'uri_node_map',
                 'expected_edges',
                 'actual_edges',
                 'unconnected_subscriptions',
                 'use_sim_time',
                 'warnings', 'errors',
                 'rospack', 'rosstack']
    
    def __init__(self):
        # main package we are running 
        self.pkg = None
        self.pkg_dir = None
        # main stack we are running 
        self.stack = None
        self.stack_dir = None
        
        # - list of all packages involved in this check
        self.pkgs = []
        # - list of all stacks involved in this check        
        self.stacks = []        

        # manifest location of package that we are running 
        self.manifest_file = None
        # manifest of package that we are running 
        self.manifest = None

        # environment variables 
        self.env = {}

        # provide these for convenience
        self.ros_root = None
        self.ros_package_path = None
        self.pythonpath = None
        
        # launch file that is being run
        self.launch_files = None
        self.launch_file_deps = None
        self.launch_file_missing_deps = None        
        
        # online state
        self.roslaunch_uris = None 
        self.system_state = None #master.getSystemState
        self.topics = None
        self.services = None
        self.service_providers = None #names of nodes with services
        self.nodes = None
        self.uri_node_map = {}
        self.expected_edges = None
        self.actual_edges = None
        self.unconnected_subscriptions = None
        self.use_sim_time = None
        
        # caching rospack instance
        self.rospack = self.rosstack = None
        
        # warnings that we have collected so far
        self.warnings = []
        # errors that we have collected so far
        self.errors = []

    def as_dictionary(self):
        """
        @return: dictionary representation of context, which is
        useful for producing error messages
        @rtype: dict
        """
        return dict((s, getattr(self, s)) for s in self.__slots__)

    @staticmethod
    def from_roslaunch(roslaunch_files, env=None):
        """
        @param roslaunch_file: roslaunch_file to load from
        @type  roslaunch_file: str
        """
        if env is None:
            env = os.environ

        # can't go any further if launch file doesn't validate
        l, c = roslaunch.XmlLoader(), roslaunch.ROSLaunchConfig()
        for f in roslaunch_files:
            try:
                l.load(f, c, verbose=False) 
            except roslaunch.RLException, e:
                raise WtfException("Unable to load roslaunch file [%s]: %s"%(f, str(e)))

        ctx = WtfContext()
        ctx.rospack = rospkg.RosPack(rospkg.get_ros_paths(env))
        ctx.rosstack = rospkg.RosStack(rospkg.get_ros_paths(env))

        ctx.launch_files = roslaunch_files
        _load_roslaunch(ctx, roslaunch_files)
        # ctx.pkg and ctx.stack initialized by _load_roslaunch
        _load_pkg(ctx, ctx.pkg)
        if ctx.stack:
            _load_stack(ctx, ctx.stack)        
        _load_env(ctx, env)
        return ctx

    @staticmethod
    def from_stack(stack, env=None):
        """
        Initialize WtfContext from stack.
        @param stack: stack name
        @type  stack: str
        @raise WtfException: if context state cannot be initialized
        """
        if env is None:
            env = os.environ

        ctx = WtfContext()
        ctx.rospack = rospkg.RosPack(rospkg.get_ros_paths(env))
        ctx.rosstack = rospkg.RosStack(rospkg.get_ros_paths(env))

        _load_stack(ctx, stack)
        try:
            ctx.pkgs = ctx.rosstack.packages_of(stack)
        except rospkg.ResourceNotFound:
            # this should be handled elsewhere
            ctx.pkgs = []
        _load_env(ctx, env)
        return ctx
    
    @staticmethod
    def from_package(pkg, env=None):
        """
        Initialize WtfContext from package name.

        @param pkg: package name
        @type  pkg: str
        @raise WtfException: if context state cannot be initialized
        """
        if env is None:
            env = os.environ

        ctx = WtfContext()
        ctx.rospack = rospkg.RosPack(rospkg.get_ros_paths(env))
        ctx.rosstack = rospkg.RosStack(rospkg.get_ros_paths(env))
        
        _load_pkg(ctx, pkg)
        stack = ctx.rospack.stack_of(pkg)
        if stack:
            _load_stack(ctx, stack)
        _load_env(ctx, env)
        return ctx

    @staticmethod
    def from_env(env=None):
        """
        Initialize WtfContext from environment.
        
        @raise WtfException: if context state cannot be initialized
        """
        if env is None:
            env = os.environ

        ctx = WtfContext()
        ctx.rospack = rospkg.RosPack(rospkg.get_ros_paths(env))
        ctx.rosstack = rospkg.RosStack(rospkg.get_ros_paths(env))

        _load_env(ctx, env)
        return ctx
    
def _load_roslaunch(ctx, roslaunch_files):
    """
    Utility for initializing WtfContext state from roslaunch file
    """
    try:
        base_pkg, file_deps, missing = roslaunch.depends.roslaunch_deps(roslaunch_files)
        ctx.pkg = base_pkg
        ctx.launch_file_deps = file_deps
        ctx.launch_file_missing_deps = missing
    except roslaunch.substitution_args.SubstitutionException as se:
        raise WtfException("Cannot load roslaunch file(s): "+str(se))
    except roslaunch.depends.RoslaunchDepsException as e:
        raise WtfException(str(e))

def _load_pkg(ctx, pkg):
    """
    Utility for initializing WtfContext state
    @raise WtfException: if context state cannot be initialized
    """
    r = ctx.rospack
    ctx.pkg = pkg
    try:
        ctx.pkgs = [pkg] + r.get_depends(pkg)
    except rospkg.ResourceNotFound as e:
        raise WtfException("Cannot find dependencies for package [%s]: missing %s"%(pkg, e))
    try:
        ctx.pkg_dir = r.get_path(pkg)
        ctx.manifest_file = os.path.join(ctx.pkg_dir, 'manifest.xml')
        ctx.manifest = r.get_manifest(pkg)
    except rospkg.ResourceNotFound:
        raise WtfException("Cannot locate manifest file for package [%s]"%pkg)
    except rospkg.InvalidManifest as e:
        raise WtfException("Package [%s] has an invalid manifest: %s"%(pkg, e))

def _load_stack(ctx, stack):
    """
    Utility for initializing WtfContext state
    @raise WtfException: if context state cannot be initialized
    """
    r = ctx.rosstack
    ctx.stack = stack
    try:
        ctx.stacks = [stack] + r.get_depends(stack, implicit=True)
    except rospkg.ResourceNotFound as e:
        raise WtfException("Cannot load dependencies of stack [%s]: %s"%(stack, e))
    try:
        ctx.stack_dir = r.get_path(stack)
    except rospkg.ResourceNotFound:
        raise WtfException("[%s] appears to be a stack, but it's not on your ROS_PACKAGE_PATH"%stack)
    
    
def _load_env(ctx, env):
    """
    Utility for initializing WtfContext state

    @raise WtfException: if context state cannot be initialized
    """
    ctx.env = env
    try:
        ctx.ros_root = env[rospkg.environment.ROS_ROOT]
    except KeyError:
        raise WtfException("ROS_ROOT is not set")
    ctx.ros_package_path = env.get(rospkg.environment.ROS_PACKAGE_PATH, None)
    ctx.pythonpath = env.get('PYTHONPATH', None)
    ctx.ros_master_uri = env.get(rosgraph.ROS_MASTER_URI, None)

    
