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
    # kwc: this really is just a 1-directory reimplementation of
    # list_pkgs(). Should merge implementations, though have to deal
    # with issues of cache, etc...
    while l:
        d = l.pop()
        if os.path.isdir(d):
            if roslib.packages.is_pkg_dir(d):
                p = os.path.basename(d)
                # this is sometimes true if we've descended into a build directory
                if not p in packages:
                    packages.append(p)
            elif os.path.exists(os.path.join(d, 'rospack_nosubdirs')):
                # don't descend
                pass
            elif os.path.basename(d) not in ['build', '.svn', '.git']: #recurse
                l.extend([os.path.join(d, e) for e in os.listdir(d)])
    return packages
    
def get_stack_dir(stack):
    """
    Get the directory of a ROS stack. This will initialize an internal
    cache and return cached results if possible.
    
    This routine is not thread-safe to os.environ changes.
    
    @param stack: name of ROS stack to locate on disk
    @type  stack: str
    @return: directory of stack, or None if stack cannot be located
    @rtype: str
    """
    
    # this cache implementation is technically incorrect. it's
    # possible to get incorrect results by manipulating environment
    # overrides from multiple threads. for the sake of future
    # implementations, we provide an environment check, but within
    # this implementaiton it does not provide much guarantee

    env = os.environ
    if stack in _dir_cache:
        ros_root = env[roslib.rosenv.ROS_ROOT]
        ros_package_path = env.get(roslib.rosenv.ROS_PACKAGE_PATH, '')

        d, rr, rpp = _dir_cache[stack]
        if rr == ros_root and rpp == ros_package_path:
            return d
        
    list_stacks() #update cache
    
    if stack in _dir_cache:
        # don't revalidate on this side. there's no means to override
        # the environment, so this could only happen if the user in
        # modifying the environment mid-call, which has no correct
        # result anyways
        return _dir_cache[stack][0]
    else:
        return None

# TODO: consolidate with list_pkgs
_dir_cache = {}

def list_stacks():
    """
    Get list of all ROS stacks. This initializes an internal
    cache.

    This routine is not thread-safe to os.environ changes.

    @return: complete list of stacks names in ROS environment
    @rtype: [str]
    """
    env = os.environ
    ros_root = env[roslib.rosenv.ROS_ROOT]

    pkg_dirs = roslib.packages.get_package_paths(env=env)
    stacks = []
    # ros is assumed to be at ROS_ROOT
    if os.path.exists(os.path.join(ros_root, 'stack.xml')):
        stacks.append('ros')
        ros_package_path = env.get(roslib.rosenv.ROS_PACKAGE_PATH, '')
        _dir_cache['ros'] = ros_root, ros_root, ros_package_path
    
    for pkg_root in pkg_dirs:
        stacks.extend(list_stacks_by_path(pkg_root, cache=_dir_cache))
    return stacks

def list_stacks_by_path(path, cache=None):
    stacks = []

    # record settings for cache, not threadsafe
    env = os.environ
    ros_root = env[roslib.rosenv.ROS_ROOT]
    ros_package_path = env.get(roslib.rosenv.ROS_PACKAGE_PATH, '')
    
    for d, dirs, files in os.walk(path, topdown=True):
        if STACK_FILE in files:
            stack = os.path.basename(d)
            if stack not in stacks:
                stacks.append(stack)
                if cache is not None:
                    cache[stack] = d, ros_root, ros_package_path
            del dirs[:]
            continue #leaf
        elif roslib.packages.MANIFEST_FILE in files:
            del dirs[:]
            continue #leaf     
        elif 'rospack_nosubdirs' in files:
            del dirs[:]
            continue  #leaf
        #small optimization
        elif '.svn' in dirs:
            dirs.remove('.svn')
        elif '.git' in dirs:
            dirs.remove('.git')
        for sub_d in dirs:
            # followlinks=True only available in Python 2.6, so we
            # have to implement manually
            sub_p = os.path.join(d, sub_d)
            if os.path.islink(sub_p):
                stacks.extend(list_stacks_by_path(sub_p, cache=cache))
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
    if type(names) not in [tuple, list]:
        raise ValueError("names must be a list of strings")

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
            except roslib.stacks.InvalidROSStackException, e:
                invalid.append(n)
        else:
            valid.append(n)
    return valid, invalid
