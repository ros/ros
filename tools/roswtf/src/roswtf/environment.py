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

import os
import socket
import stat
import string
import sys

from os.path import isdir, isfile
from roswtf.rules import warning_rule, error_rule

#TODO: unit tests

## @return [str]: paths contained in \a path variable. \a path must
## conform to OS conventions for path separation (i.e. colon-separated
## on Unix)
def paths(path):
    if path:
        return path.split(os.pathsep)
    return []

## @return True if \a path has executable permissions
def is_executable(path):
    mode = os.stat(path)[stat.ST_MODE]
    return mode & (stat.S_IXUSR|stat.S_IXGRP|stat.S_IXOTH)

import urlparse
## @return str error message if \a url is not a valid url. \a url is
## allowed to be empty as that check is considered separately.
def invalid_url(url):
    if not url:
        return #caught by different rule
    p = urlparse.urlparse(url)
    if p[0] != 'http':
        return "protocol is not 'http'"
    if not p[1]:
        return "address is missing"
    if not ':' in p[1]:
        return "port number is missing"
    try:
        splits = p[1].split(':')
        if len(splits) != 2:
            return "invalid address string [%s]"%p[1]
        string.atoi(splits[1])
    except ValueError:
        return "port number [%s] is invalid"%(splits[1])
    
# Error-checking functions for more advanced checks

def ros_root_check(ctx):
    path = ctx.ros_root
    bindir = os.path.join(path, 'bin')
    if not isdir(bindir):
        return "ROS_ROOT [%s] does not have a valid bin directory"%path
    rospack = os.path.join(bindir, 'rospack')
    if not isfile(rospack):
        return "%(ros_root)s/bin is missing rospack. Perhaps that ROS has not been built yet"
    if not is_executable(rospack):
        return "%s is lacking executable permissions"%rospack

    
def _writable_dir_check(ctx, path, name):
    """
    If path is not None, validate that it is a writable directory
    """
    if path is None:
        return
    if isfile(path):
        return "%s [%s] must point to a directory, not a file"%(name, path)
    if not os.access(path, os.W_OK):
        return "%s [%s] is not writable"%(name, path)

def ros_home_check(ctx):
    return _writable_dir_check(ctx, ctx.env.get('ROS_HOME', None), 'ROS_HOME')
def ros_log_dir_check(ctx):
    return _writable_dir_check(ctx, ctx.env.get('ROS_LOG_DIR', None), 'ROS_LOG_DIR')
def ros_test_results_dir_check(ctx):
    return _writable_dir_check(ctx, ctx.env.get('ROS_TEST_RESULTS_DIR', None), 'ROS_TEST_RESULTS_DIR')

def pythonpath_check(ctx):
    path = ctx.pythonpath
    roslib_count = len(set([p for p in paths(path) if 'roslib' in p]))
    if not roslib_count:
        return "roslib directory is not in PYTHONPATH"
    if roslib_count > 1:
        return "Multiple roslib directories in PYTHONPATH (there should only be one)"
    rp = os.path.join('roslib', 'src')
    if not rp in path:
        return "roslib directory in PYTHONPATH should point to %s"%rp
    if 'rostools' in path:
        return "rostools should no longer be set in PYTHONPATH"

def rosconsole_config_file_check(ctx):
    if 'ROSCONSOLE_CONFIG_FILE' in ctx.env:
        return not isfile(ctx.env['ROSCONSOLE_CONFIG_FILE'])
    
def path_check(ctx):
    # rosdeb setup can clobber local ros stuff, so try and detect this
    path = ctx.env['PATH']
    idx = path.find('/usr/bin')
    if idx < 0:
        return
    if os.path.exists('/usr/lib/ros/'):
        rr_idx = path.find(ctx.ros_root)
        if rr_idx > -1 and rr_idx > idx:
            return True
        
def ros_master_uri_hostname(ctx):
    uri = ctx.ros_master_uri
    parsed = urlparse.urlparse(uri)
    p = urlparse.urlparse(uri)
    if not p[1]:
        return #caught by different rule
    if not ':' in p[1]:
        return #caught by different rule
    try:
        splits = p[1].split(':')
        if len(splits) != 2:
            return #caught by different rule
        socket.gethostbyname(splits[0])
    except socket.gaierror, e:
        return "Unknown host %s"%splits[0]
    
def boost_check(ctx):
    import subprocess
    retcode = subprocess.call(['rosboost-cfg', '--cflags'], stdout=subprocess.PIPE)
    if retcode != 0:
        return "Unable to locate boost" 

# Error/Warning Rules

environment_warnings = [
    (path_check,
     "PATH has /usr/bin set before ROS_ROOT/bin, which can cause problems as there is system install of ROS on this machine. You may wish to put ROS_ROOT/bin first"),
    (lambda ctx: ctx.ros_package_path is None, 
     "ROS_PACKAGE_PATH is not set. This is not required, but is unusual"),
    (lambda ctx: len(paths(ctx.ros_package_path)) == 0, 
     "ROS_PACKAGE_PATH is empty. This is not required, but is unusual"),
    (lambda ctx: not ctx.ros_master_uri,
     "ROS_MASTER_URI is empty. This is not required, but is unusual"),
    (lambda ctx: invalid_url(ctx.ros_master_uri),
     "ROS_MASTER_URI [%(ros_master_uri)s] is not a valid URL: "),
    (ros_master_uri_hostname,
     "Cannot resolve hostname in ROS_MASTER_URI [%(ros_master_uri)s]"),
    (rosconsole_config_file_check,
     "ROS_CONSOLE_CONFIG_FILE does not point to an existing file"),
    ]

environment_errors = [
    # ROS_ROOT
    (lambda ctx: not isdir(ctx.ros_root),
     "ROS_ROOT [%(ros_root)s] does not point to a directory"),
    (ros_root_check,
     "ROS_ROOT is invalid: "),
    
    # ROS_PACKAGE_PATH
    (lambda ctx: [d for d in paths(ctx.ros_package_path) if d and isfile(d)],
     "Path(s) in ROS_PACKAGE_PATH [%(ros_package_path)s] points to a file instead of a directory: "),
    (lambda ctx: [d for d in paths(ctx.ros_package_path) if d and not isdir(d) and d != 'test_roswtf_ignore' ],
     "Not all paths in ROS_PACKAGE_PATH [%(ros_package_path)s] point to an existing directory: "),
    
    # PYTHONPATH
    (lambda ctx: [d for d in paths(ctx.pythonpath) if d and not isdir(d)],
     "Not all paths in PYTHONPATH [%(pythonpath)s] point to a directory: "),
    (pythonpath_check,
     "PYTHONPATH [%(pythonpath)s] is invalid: "),

    # ROS_HOME, ROS_LOG_DIR, ROS_TEST_RESULTS_DIR
    (ros_home_check, "ROS_HOME is invalid: "),
    (ros_log_dir_check, "ROS_LOG_DIR is invalid: "),    
    (ros_test_results_dir_check, "ROS_TEST_RESULTS_DIR is invalid: "),    

    (lambda ctx: ctx.ros_bindeps_path and not isdir(ctx.ros_bindeps_path),
     "ROS_BINDEPS_PATH [%(ros_bindeps_path)s] does not point to a directory"),
    (boost_check, "Cannot locate boost. Please see installation instructions. "),
    ]

def wtf_check_environment(ctx):
    #TODO: check ROS_BOOST_ROOT
    for r in environment_warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in environment_errors:
        error_rule(r, r[0](ctx), ctx)
        
