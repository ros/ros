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
ROS environment variables.
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
## @deprecated Replaced by ROS_HOSTNAME, with equivalent functionality
ROS_IP           ="ROS_IP"
## hostname/address to bind XML-RPC services to. 
ROS_HOSTNAME     ="ROS_HOSTNAME"
ROS_NAMESPACE    ="ROS_NAMESPACE"
## directory in which log files are written
ROS_LOG_DIR      ="ROS_LOG_DIR"
## directory in which test result files are written
ROS_TEST_DIR     = "ROS_TEST_DIR"

class ROSEnvException(roslib.exceptions.ROSLibException):
    """Base class of roslib.rosenv errors."""
    pass

def get_ros_root(required=True, environ=os.environ):
    """
    @param required: if True, ROS_ROOT must be set and point to a valid directory.
    @type  required: bool
    @raise ROSEnvException: if required is True and ROS_ROOT is not
    set validly
    """
    p = None
    try:
        if not environ.has_key(ROS_ROOT):
            raise ROSEnvException, """
The %(ROS_ROOT)s environment variable has not been set.
Please set to the location of your ROS installation
before continuing.
"""%globals()

        p = environ[ROS_ROOT]
        #Test:
        # 1. Is a path
        # 2. Is a directory
        if not os.path.exists(p):
            raise ROSEnvException, """
The %s environment variable has not been set properly:
%s does not exist.
Please update your ROS installation before continuing.
"""%(ROS_ROOT, p)
        if not os.path.isdir(p):
            raise ROSEnvException, """
The %s environment variable has not been set properly:
%s is not a directory.
Please update your ROS installation before continuing.
"""%(ROS_ROOT, p)
        return p
    except Exception, e:
        if required:
            raise
        return p

def get_ros_package_path(required=True, environ=os.environ):
    """
    @raise ROSEnvException: if ROS_PACKAGE_PATH is not set and \a
    required is True
    """
    try:
        return environ[ROS_PACKAGE_PATH]
    except KeyError, e:
        if required:
            raise ROSEnvException("%s has not been configured"%ROS_PACKAGE_PATH)

def get_master_uri(required=True, environ=os.environ, argv=sys.argv):
    """
    Get the ROS_MASTER_URI setting from the command-line args or
    environment, command-line args takes precedence.
    @param required: if True, enables exception raising
    @type  required: bool
    @param environ: override environment dictionary
    @type  environ: dict
    @param argv: override sys.argv
    @type  argv: [str]
    @raise ROSEnvException: if ROS_MASTER_URI is not set
    """    
    try:
        for arg in argv:
            if arg.startswith('__master:='):
                try:
                    _, val = arg.split(':=') 
                    return val
                except:
                    # split unpack failed
                    if required:
                        raise ROSEnvException("__master remapping argument '%s' improperly specified"%arg)
                    else:
                        return None
        return environ[ROS_MASTER_URI]
    except KeyError, e:
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
    return os.pathsep.join([resolve_path(p) for p in paths.split(os.pathsep)])


def setup_default_environment():
  """
  Bootstrap common ROS environment variables. For now, only affects
  os.environ if the environment has a rosdeb-based installation. It
  does not check for remapping args that may also affect
  ROS_MASTER_URI as those have precedence regardless.
  """
  default_ros_root = "/usr/lib/ros"
  if os.path.isdir(default_ros_root):
    if 'ROS_ROOT' not in os.environ:
      os.environ['ROS_ROOT'] = default_ros_root
    if 'ROS_PACKAGE_PATH' not in os.environ:
      os.environ['ROS_PACKAGE_PATH'] = os.path.join(default_ros_root, "pkgs")
    if 'ROS_MASTER_URI' not in os.environ:
      os.environ['ROS_MASTER_URI'] = "http://localhost:%d" % (10000+os.geteuid(),)
    if 'ROS_LOG_DIR' not in os.environ:
      os.environ['ROS_LOG_DIR'] = os.path.join(os.environ.get("HOME"), ".ros", "logs")

    ros_root = os.environ.get("ROS_ROOT")

    if "PYTHONPATH" in os.environ:
      os.environ['PYTHONPATH'] = os.environ['PYTHONPATH'] + ":" + os.path.join(ros_root, "python") + ":" + os.path.join(ros_root, "lib")
    else:
      os.environ['PYTHONPATH'] = os.path.join(ros_root, "python") + ":" + os.path.join(ros_root, "lib")

    if "LD_LIBRARY_PATH" in os.environ:
      os.environ['LD_LIBRARY_PATH'] = os.path.join(ros_root, "lib") + ":" + os.environ["LD_LIBRARY_PATH"]
    else:
      os.environ['LD_LIBRARY_PATH'] = os.path.join(ros_root, "lib")
  
def get_log_dir(environ=None):
    """
    Get directory to use for writing log files. There are multiple
    possible locations for this. The ROS_LOG_DIR environment variable
    has priority. If that is not set, then ROS_HOME/log is used. If
    ROS_HOME is not set, $HOME/.ros/log is used.

    @param environ: environment dictionary (defaults to os.environ)
    @type  environ: dict
    @return: path to use use for log file directory
    @rtype: str
    """
    if environ is None:
        environ = os.environ
    if ROS_LOG_DIR in environ:
        log_dir = environ[ROS_LOG_DIR]
    elif ROS_HOME in environ:
        log_dir = os.path.join(environ[ROS_HOME], 'log')
    else:
        user_home_dir = os.path.expanduser('~') #slightly more robust than $HOME
        log_dir = os.path.join(user_home_dir, '.ros', 'log')
    return log_dir

def get_test_results_dir(environ=None):
    """
    Get directory to use for writing test result files. There are multiple
    possible locations for this. The ROS_TEST_DIR environment variable
    has priority. If that is set, ROS_TEST_DIR is returned.
    If ROS_TEST_DIR is not set, then ROS_HOME/test_results is used. If
    ROS_HOME is not set, $HOME/.ros/test_results is used.

    @param environ: environment dictionary (defaults to os.environ)
    @type  environ: dict
    @return: path to use use for log file directory
    @rtype: str
    """
    # temporary: XXX remove as soon as rosbuild.cmake migrated
    if environ is None:
        environ = os.environ
    if 1:
        return os.path.join(get_ros_root(environ=environ), 'test', 'test_results')
        
    if ROS_TEST_DIR in environ:
        return environ[ROS_TEST_DIR]
    elif ROS_HOME in environ:
        return os.path.join(environ[ROS_HOME], 'test_results')
    else:
        #slightly more robust than $HOME
        return os.path.join(os.path.expanduser('~'), '.ros', 'test_results')
