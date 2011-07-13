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
ROS environment variables as well as routines for determining
configuration values that have environment overrides
(e.g. ROS_LOG_DIR, ROS_HOME, ROS_TEST_RESULTS_DIR).
"""

import os
import sys

import roslib.exceptions

# Global, usually set in setup
ROS_ROOT         = "ROS_ROOT"
ROS_MASTER_URI   = "ROS_MASTER_URI"
ROS_PACKAGE_PATH = "ROS_PACKAGE_PATH"
ROS_HOME         = "ROS_HOME"

# Build-related
ROS_BINDEPS_PATH = "ROS_BINDEPS_PATH"
ROS_BOOST_ROOT = "ROS_BOOST_ROOT"

# Per session
## hostname/address to bind XML-RPC services to. 
ROS_IP           ="ROS_IP"
ROS_HOSTNAME     ="ROS_HOSTNAME"
ROS_NAMESPACE    ="ROS_NAMESPACE"
## directory in which log files are written
ROS_LOG_DIR      ="ROS_LOG_DIR"
## directory in which test result files are written
ROS_TEST_RESULTS_DIR = "ROS_TEST_RESULTS_DIR"

class ROSEnvException(roslib.exceptions.ROSLibException):
    """Base class of roslib.rosenv errors."""
    pass

def get_ros_root(required=True, env=None):
    """
    @param required: (default True). If True, ROS_ROOT must be set and point to a valid directory.
    @type  required: bool
    @param env: override environment dictionary
    @type  env: dict
    @raise ROSEnvException: if required is True and ROS_ROOT is not
    set validly
    """
    if env is None:
        env = os.environ
    p = None
    try:
        if ROS_ROOT not in env:
            raise ROSEnvException("""
The %(ROS_ROOT)s environment variable has not been set.
Please set to the location of your ROS installation
before continuing.
"""%globals())

        p = env[ROS_ROOT]
        #Test:
        # 1. Is a path
        # 2. Is a directory
        if not os.path.exists(p):
            raise ROSEnvException("""
The %s environment variable has not been set properly:
%s does not exist.
Please update your ROS installation before continuing.
"""%(ROS_ROOT, p))
        if not os.path.isdir(p):
            raise ROSEnvException("""
The %s environment variable has not been set properly:
%s is not a directory.
Please update your ROS installation before continuing.
"""%(ROS_ROOT, p))
        return p
    except Exception as e:
        if required:
            raise
        return p

def get_ros_package_path(required=False, env=None):
    """
    @param required: (default False) if True, ROS_PACKAGE_PATH must be
    set and point to a valid directory.
    @type  required: bool
    @raise ROSEnvException: if ROS_PACKAGE_PATH is not set and \a
    required is True
    """
    if env is None:
        env = os.environ
    try:
        return env[ROS_PACKAGE_PATH]
    except KeyError as e:
        if required:
            raise ROSEnvException("%s has not been configured"%ROS_PACKAGE_PATH)

def get_master_uri(required=True, env=None, argv=None):
    """
    Get the ROS_MASTER_URI setting from the command-line args or
    environment, command-line args takes precedence.
    @param required: if True, enables exception raising
    @type  required: bool
    @param env: override environment dictionary
    @type  env: dict
    @param argv: override sys.argv
    @type  argv: [str]
    @raise ROSEnvException: if ROS_MASTER_URI value is invalidly
    specified or if required and ROS_MASTER_URI is not set
    """    
    if env is None:
        env = os.environ
    if argv is None:
        argv = sys.argv
    try:
        for arg in argv:
            if arg.startswith('__master:='):
                val = None
                try:
                    _, val = arg.split(':=')
                except:
                    pass
                
                # we ignore required here because there really is no
                # correct return value as the configuration is bad
                # rather than unspecified
                if not val:
                    raise ROSEnvException("__master remapping argument '%s' improperly specified"%arg)
                return val
        return env[ROS_MASTER_URI]
    except KeyError as e:
        if required:
            raise ROSEnvException("%s has not been configured"%ROS_MASTER_URI)
        
def resolve_path(p):
    """
    @param path: path string
    @type  path: str
    Catch-all utility routine for fixing ROS environment variables that
    are a single path (e.g. ROS_ROOT).  Currently this just expands
    tildes to home directories, but in the future it may encode other
    behaviors.
    """
    if p and p[0] == '~':
        return os.path.expanduser(p)
    return p
    
def resolve_paths(paths):
    """
    @param paths: path string with OS-defined separator (i.e. ':' for Linux)
    @type  paths: str
    Catch-all utility routine for fixing ROS environment variables that
    are paths (e.g. ROS_PACKAGE_PATH).  Currently this just expands
    tildes to home directories, but in the future it may encode other
    behaviors.
    """
    splits = [p for p in paths.split(os.pathsep) if p]
    return os.pathsep.join([resolve_path(p) for p in splits])


def get_ros_home(env=None):
    """
    Get directory location of '.ros' directory (aka ROS home).
    possible locations for this. The ROS_LOG_DIR environment variable
    has priority. If that is not set, then ROS_HOME/log is used. If
    ROS_HOME is not set, $HOME/.ros/log is used.

    @param env: override os.environ dictionary
    @type  env: dict
    @return: path to use use for log file directory
    @rtype: str
    """
    if env is None:
        env = os.environ
    if ROS_HOME in env:
        return env[ROS_HOME]
    else:
        #slightly more robust than $HOME
        return os.path.join(os.path.expanduser('~'), '.ros')
    
def get_log_dir(env=None):
    """
    Get directory to use for writing log files. There are multiple
    possible locations for this. The ROS_LOG_DIR environment variable
    has priority. If that is not set, then ROS_HOME/log is used. If
    ROS_HOME is not set, $HOME/.ros/log is used.

    @param env: override os.environ dictionary
    @type  env: dict
    @return: path to use use for log file directory
    @rtype: str
    """
    if env is None:
        env = os.environ
    if ROS_LOG_DIR in env:
        return env[ROS_LOG_DIR]
    else:
        return os.path.join(get_ros_home(env), 'log')

def get_test_results_dir(env=None):
    """
    Get directory to use for writing test result files. There are multiple
    possible locations for this. The ROS_TEST_RESULTS_DIR environment variable
    has priority. If that is set, ROS_TEST_RESULTS_DIR is returned.
    If ROS_TEST_RESULTS_DIR is not set, then ROS_HOME/test_results is used. If
    ROS_HOME is not set, $HOME/.ros/test_results is used.

    @param env: environment dictionary (defaults to os.environ)
    @type  env: dict
    @return: path to use use for log file directory
    @rtype: str
    """
    if env is None:
        env = os.environ
        
    if ROS_TEST_RESULTS_DIR in env:
        return env[ROS_TEST_RESULTS_DIR]
    else:
        return os.path.join(get_ros_home(env), 'test_results')

# this is a copy of the roslogging utility. it's been moved here as it is a common
# routine for programs using accessing ROS directories
def makedirs_with_parent_perms(p):
    """
    Create the directory using the permissions of the nearest
    (existing) parent directory. This is useful for logging, where a
    root process sometimes has to log in the user's space.
    @param p: directory to create
    @type  p: str
    """    
    p = os.path.abspath(p)
    parent = os.path.dirname(p)
    # recurse upwards, checking to make sure we haven't reached the
    # top
    if not os.path.exists(p) and p and parent != p:
        makedirs_with_parent_perms(parent)
        s = os.stat(parent)
        os.mkdir(p)

        # if perms of new dir don't match, set anew
        s2 = os.stat(p)
        if s.st_uid != s2.st_uid or s.st_gid != s2.st_gid:
            os.chown(p, s.st_uid, s.st_gid)
        if s.st_mode != s2.st_mode:
            os.chmod(p, s.st_mode)    

def on_ros_path(p):
    """
    Check to see if filesystem path is on paths specified in ROS
    environment (ROS_ROOT, ROS_PACKAGE_PATH).

    New in ROS 1.2.
    
    @param p: path
    @type  p: str
    @return: True if p is on the ROS path (ROS_ROOT, ROS_PACKAGE_PATH)
    """
    pkg = os.path.realpath(roslib.rosenv.resolve_path(p))
    paths = [p for p in roslib.packages.get_package_paths()]
    paths = [os.path.realpath(roslib.rosenv.resolve_path(x)) for x in paths]
    return bool([x for x in paths if pkg == x or pkg.startswith(x + os.sep)])
