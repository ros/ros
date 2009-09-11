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
# Revision $Id: packspec.py 3357 2009-01-13 07:13:05Z jfaustwg $
# $Author: jfaustwg $

import os
import sys
import stat
import string

import roslib.exceptions
import roslib.names
import roslib.rosenv

MSG_DIR = 'msg'
SRV_DIR = 'srv'
SRC_DIR = 'src'

# aliases
ROS_PACKAGE_PATH = roslib.rosenv.ROS_PACKAGE_PATH
ROS_ROOT = roslib.rosenv.ROS_ROOT

class ROSPkgException(roslib.exceptions.ROSLibException): pass
class InvalidROSPkgException(ROSPkgException): pass
class MultipleNodesException(ROSPkgException): pass

# TODO: go through the code and eliminate unused methods -- there's far too many combos here

MANIFEST_FILE = 'manifest.xml'

#
# Map package/directory structure
#

## @return bool True: if \a d is the root directory of a ROS Package
def is_pkg_dir(d):
    return os.path.isfile(os.path.join(d, MANIFEST_FILE))

# TODO: really need to rethink this required logic. I think what we
# really want is a way to get the existing paths. The general behavior
# of PATH vars seems to be to not care if the path does not exist, so
# the 'required' notion seems incorrect. It's really just
# ros_root_required

## Get the paths to search for packages
## @param ros_root_required bool: if True, raise exception if
## environment is invalid (i.e. ROS_ROOT is not set properly)
## @param environ dict: override os.environ dictionary
## @throws roslib.rosenv.ROSEnvException if ros_root_required is True
## and ROS_ROOT is not set
def get_package_paths(ros_root_required=True, environ=os.environ):
    rpp = roslib.rosenv.get_ros_package_path(False, environ)
    if rpp:
        paths = [x for x in rpp.split(os.pathsep) if x]
    else:
        paths = []
    rr_path = roslib.rosenv.get_ros_root(ros_root_required, environ)
    if rr_path:
        return paths + [rr_path]
    else:
        return paths

## Get the package that the directory is contained within. This is
## determined by finding the nearest parent manifest.xml file. This
## isn't 100% reliable, but symlinks can full any heuristic that
## relies on ROS_ROOT.
## @param dir str: directory path
## @param environ dict: override os.environ dictionary
## @return (str, str): (packageDirectory, package) of the specified directory, or None,None if not in a package
def get_dir_pkg(dir, environ=os.environ):
    #TODO: the realpath is going to create issues with symlinks, most likely
    pkgDirs = [os.path.realpath(x) for x in get_package_paths(True, environ)]
    for pkgDir in pkgDirs:
        parent = os.path.dirname(os.path.realpath(dir))
        #walk up until we hit ros root or ros/pkg
        while not os.path.exists(os.path.join(dir, MANIFEST_FILE)) and parent != dir:
            dir = parent
            parent = os.path.dirname(dir)
        if os.path.exists(os.path.join(dir, MANIFEST_FILE)):
            pkg = os.path.basename(os.path.abspath(dir))
            return dir, pkg
    return None, None

## @internal
_pkg_dir_cache = {}

## Locate directory \a package is stored in
## @param package str: package name
## @param required bool: if True, an exception will be raised if the
## package directory cannot be located.
## @param ros_root str: if specified, override ROS_ROOT
## @param ros_package_path str: if specified, override ROS_PACKAGE_PATH
## @return str: directory containing package.
## @throws InvalidROSPkgException: if required is True and package cannot be located
def get_pkg_dir(package, required=True, ros_root=None, ros_package_path=None):
    #subprocess is a Python 2.4 module, lazy import here so that roslib.packages constants can be access from Python 2.3
    #UNIXONLY
    #TODO: replace with non-rospack-based solution (e.g. os.walk())
    from subprocess import Popen, PIPE
    try:
        penv = os.environ.copy()
        if ros_root:
            ros_root = roslib.rosenv.resolve_path(ros_root)
            rospack = os.path.join(ros_root, 'bin', 'rospack')
            penv[ROS_ROOT] = ros_root
        else:
            rospack = 'rospack'
            # record setting for _pkg_dir_cache
            if ROS_ROOT in os.environ:
                ros_root = os.environ[ROS_ROOT]
        if ros_package_path is not None:
            ros_package_path = roslib.rosenv.resolve_paths(ros_package_path)
            penv[ROS_PACKAGE_PATH] = ros_package_path
        elif ROS_PACKAGE_PATH in os.environ:
            # record setting for _pkg_dir_cache
            ros_package_path = os.environ[ROS_PACKAGE_PATH]

        # now that we've resolved the args, check the cache
        if package in _pkg_dir_cache:
            dir, rr, rpp = _pkg_dir_cache[package]
            if rr == ros_root and rpp == ros_package_path:
                return dir
            
        rpout, rperr = Popen([rospack, 'find', package], \
                                 stdout=PIPE, stderr=PIPE, env=penv).communicate()

        pkg_dir = (rpout or '').strip()
        if not pkg_dir:
            raise InvalidROSPkgException("Cannot locate installation of package %s: %s. ROS_ROOT[%s] ROS_PACKAGE_PATH[%s]"%(package, rperr.strip(), ros_root, ros_package_path))

        if not os.path.exists(pkg_dir):
            raise InvalidROSPkgException("Cannot locate installation of package %s: [%s] is not a valid path. ROS_ROOT[%s] ROS_PACKAGE_PATH[%s]"%(package, pkg_dir, ros_root, ros_package_path))
        elif not os.path.isdir(pkg_dir):
            raise InvalidROSPkgException("Package %s is invalid: file [%s] is in the way"%(package, pkg_dir))
        # we also save the ros_root/ros_package_path args as these affect the pkg_dir
        _pkg_dir_cache[package] = (pkg_dir, ros_root, ros_package_path)
        return pkg_dir
    except OSError, e:
        if required:
            raise InvalidROSPkgException("Environment configuration is invalid: cannot locate rospack (%s)"%e)
        return None
    except Exception, e:
        if required:
            raise
        return None

## @internal
## @param required:  if True, will attempt to  create the subdirectory
## if it does not exist. An exception will be raised  if this fails.
## @param package_dir str: directory of package
## @param subdir str: name of subdirectory to locate    
## @param environ dict: environment dictionary    
## @param required bool: if True, directory must exist    
## @return str: Package subdirectory if package exist, otherwise None.
## @throws InvalidROSPkgException: if required is True and directory does not exist
def _get_pkg_subdir_by_dir(package_dir, subdir, required=True, environ=os.environ):
    try:
        if not package_dir:
            raise Exception("Cannot create a '%(subdir)s' directory in %(package_dir)s: package %(package) cannot be located"%locals())
        dir = os.path.join(package_dir, subdir)
        if required and os.path.isfile(dir):
            raise Exception("""Package '%(package)s' is improperly configured: 
file %(dir)s is preventing the creation of a directory"""%locals())
        elif required and not os.path.isdir(dir):
            try:
                os.makedirs(dir) #lazy create
            except error:
                raise Exception("""Package '%(package)s' is improperly configured: 
Cannot create a '%(subdir)s' directory in %(package_dir)s.
Please check permissions and try again.
"""%locals())
        return dir
    except Exception, e:
        if required:
            raise
        return None
    
## @param required:  if True, will attempt to  create the subdirectory
## if it does not exist. An exception will be raised  if this fails.
## @param package str: name of package
## @param environ dict: environment dictionary    
## @param required bool: if True, directory must exist    
## @return str: Package subdirectory if package exist, otherwise None.
## @throws InvalidROSPkgException: if required is True and directory does not exist
def get_pkg_subdir(package, subdir, required=True, environ=os.environ):
    pkg_dir = get_pkg_dir(package, required, ros_root=environ[ROS_ROOT]) 
    return _get_pkg_subdir_by_dir(pkg_dir, subdir, required, environ)

#
# Map ROS resources to files
#

## @param subdir str: name of subdir -- these should be one of the
## string constants, e.g. MSG_DIR
## @return str: path to resource in the specified subdirectory of the
## package, or None if the package does not exists
## @throws roslib.packages.InvalidROSPkgException If \a package does not exist 
def resource_file(package, subdir, resource_name):
    dir = get_pkg_subdir(package, subdir, False)
    if dir is None:
        raise InvalidROSPkgException(package)
    return os.path.join(dir, resource_name)


# TODO: try and read in .rospack_cache
# TODO: use this to replace get_pkg_dir shelling to rospack
## @param pkg_dirs [str]: (optional) list of paths to search for packages
## @return [str]: complete list of package names in ROS environment
def list_pkgs(pkg_dirs=None):
    if pkg_dirs is None:
        pkg_dirs = get_package_paths(True)
    packages = []
    # record settings for cache
    ros_root = os.environ[ROS_ROOT]
    ros_package_path = os.environ.get(ROS_PACKAGE_PATH, '')
    for pkgRoot in pkg_dirs:
        for dir, dirs, files in os.walk(pkgRoot, topdown=True):
            if MANIFEST_FILE in files:
                package = os.path.basename(dir)
                if package not in packages:
                  packages.append(package)
                  _pkg_dir_cache[package] = dir, ros_root, ros_package_path
                del dirs[:]
            elif 'rospack_nosubdirs' in files:
                del dirs[:]
            #small optimization
            elif '.svn' in dirs:
                dirs.remove('.svn')
            elif '.git' in dirs:
                dirs.remove('.git')
    return packages

# TODO: reimplement using find_resource

## Locate the executable that implements the node
## @param node_type str: type of node
## @param ros_root str: if specified, override ROS_ROOT
## @param ros_package_path str: if specified, override ROS_PACKAGE_PATH
## @return str: path to node or None if node is not in the package
## @throws roslib.packages.InvalidROSPkgException If package does not exist 
def find_node(pkg, node_type, ros_root=None, ros_package_path=None):
    dir = get_pkg_dir(pkg, required=True, \
                      ros_root=ros_root, ros_package_path=ros_package_path)
    #UNIXONLY
    node_exe = None
    for p, dirs, files in os.walk(dir):
        if node_type in files:
            test_path = os.path.join(p, node_type)
            s = os.stat(test_path)
            if (s.st_mode & stat.S_IRWXU == stat.S_IRWXU):
                return test_path
        if '.svn' in dirs:
            dirs.remove('.svn')
        elif '.git' in dirs:
            dirs.remove('.git')

## Locate the file named \a resource_name in package, optionally
## matching specified \a filter
## @param filter fn(str): function that takes in a path argument and
## returns True if the it matches the desired resource
## @param ros_root str: if specified, override ROS_ROOT
## @param ros_package_path str: if specified, override ROS_PACKAGE_PATH
## @return [str]: lists of matching paths for resource
## @throws roslib.packages.InvalidROSPkgException If package does not exist 
def find_resource(pkg, resource_name, filter_fn=None, ros_root=None, ros_package_path=None):
    dir = get_pkg_dir(pkg, required=True, \
                      ros_root=ros_root, ros_package_path=ros_package_path)
    #UNIXONLY
    matches = []
    node_exe = None
    for p, dirs, files in os.walk(dir):
        if resource_name in files:
            test_path = os.path.join(p, resource_name)
            if filter_fn is not None:
                if filter_fn(test_path):
                    matches.append(test_path)
            else:
                matches.append(test_path)
        if '.svn' in dirs:
            dirs.remove('.svn')
        elif '.git' in dirs:
            dirs.remove('.git')
    return matches
