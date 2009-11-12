#! /usr/bin/env python
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

"""
Python utilities for manipulating ROS Stacks.
See: http://ros.org/wiki/Stacks

Warning: this API is still fairly experimental and incomplete.
"""

import os
import sys

import roslib.exceptions
import roslib.packages
import roslib.rosenv

STACK_FILE = 'stack.xml'
ROS_STACK = 'ros'

class ROSStackException(roslib.exceptions.ROSLibException): pass
class InvalidROSStackException(ROSStackException): pass

def stack_of(pkg):
    """
    @return: name of stack that pkg is in, or None if pkg is not part of a stack
    @rtype: str
    @raise roslib.packages.InvalidROSPkgException: if pkg cannot be located
    """
    pkg_dir = roslib.packages.get_pkg_dir(pkg)
    dir = os.path.dirname(pkg_dir)
    while dir and os.path.dirname(dir) != dir:
        stack_file = os.path.join(dir, STACK_FILE)
        if os.path.exists(stack_file):
            #TODO: need to resolve issues regarding whether the
            #stack.xml or the directory defines the stack name
            return os.path.basename(dir)
        dir = os.path.dirname(dir)
        
def packages_of(stack, env=os.environ):
    """
    @return: name of packages that are part of stack
    @rtype: [str]
    @raise InvalidROSStackException: if stack cannot be located
    @raise ValueError: if stack name is invalid
    """
    # record settings for error messages
    ros_root = env[roslib.rosenv.ROS_ROOT]
    ros_package_path = env.get(roslib.rosenv.ROS_PACKAGE_PATH, '')
    
    if not stack:
        raise ValueError("stack name not specified")
    stack_dir = get_stack_dir(stack)
    if stack_dir is None:
        raise InvalidROSStackException("Cannot locate installation of stack %s. ROS_ROOT[%s] ROS_PACKAGE_PATH[%s]"%(stack, ros_root,ros_package_path))
    packages = []
    l = [os.path.join(stack_dir, d) for d in os.listdir(stack_dir)]
    while l:
        d = l.pop()
        if os.path.isdir(d):
            if roslib.packages.is_pkg_dir(d):
                p = os.path.basename(d)
                # this is sometimes true if we've descended into a build directory
                if not p in packages:
                    packages.append(p)
            elif os.path.basename(d) not in ['build', '.svn', '.git']: #recurse
                l.extend([os.path.join(d, e) for e in os.listdir(d)])
    return packages
    
from subprocess import Popen, PIPE
def get_stack_dir(stack):
    """
    @param stack: name of ROS stack to locate on disk
    @type  stack: str
    @return: directory of stack, or None
    @rtype: str
    @raise InvalidROSStackException: if stack cannot be located
    """
    list_stacks() #update cache
    return _dir_cache.get(stack, None)

# TODO: consolidate with list_pkgs
_dir_cache = {}
_cache_marker = None

def list_stacks(env=None):
    """
    Get list of all ROS stacks. This initializes an internal cache.

    @param env: override os.environ dictionary
    @type  env: dict
    @return: complete list of stacks names in ROS environment
    @rtype: [str]
    """
    if env is None:
        env = os.environ
    global _cache_marker
    # record settings for cache
    ros_root = env[roslib.rosenv.ROS_ROOT]
    ros_package_path = env.get(roslib.rosenv.ROS_PACKAGE_PATH, '')

    # validate cache
    if _cache_marker == (ros_root, ros_package_path):
        return _dir_cache.keys()
    else:
        _dir_cache.clear()
        _cache_marker = ros_root, ros_package_path
        
    pkg_dirs = roslib.packages.get_package_paths(env=env)
    stacks = []
    # ros is assumed to be at ROS_ROOT
    if os.path.exists(os.path.join(ros_root, 'stack.xml')):
        stacks.append('ros')
        _dir_cache['ros'] = ros_root
    
    for pkg_root in pkg_dirs:
        for dir, dirs, files in os.walk(pkg_root, topdown=True):
            if STACK_FILE in files:
                stack = os.path.basename(dir)
                if stack not in stacks:
                  stacks.append(stack)
                  _dir_cache[stack] = dir
                del dirs[:]
            elif 'rospack_nosubdirs' in files:
                del dirs[:]
            #small optimization
            elif '.svn' in dirs:
                dirs.remove('.svn')
            elif '.git' in dirs:
                dirs.remove('.git')
    return stacks

# #2022
def expand_to_packages(names):
    """
    Expand names into a list of packages. Names can either be of packages or stacks.

    @param names: names of stacks or packages
    @type  names: [str]
    @return: ([packages], [not_found]). expand_packages() returns two
    lists. The first is of packages names. The second is a list of
    names for which no matching stack or package was found. Lists may have duplicates.
    @rtype: ([str], [str])
    """

    # do full package list first. This forces an entire tree
    # crawl. This is less efficient for a small list of names, but
    # much more efficient for many names.
    package_list = roslib.packages.list_pkgs()
    valid = []
    invalid = []
    for n in names:
        if not n in package_list:
            try:
                valid.extend(roslib.stacks.packages_of(n))
            except roslib.stacks.InvalidROSStackException:
                invalid.append(n)
        else:
            valid.append(n)
    return valid, invalid
