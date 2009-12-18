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
# $Author$

from __future__ import with_statement

import os
import sys
import stat
import string

from subprocess import Popen, PIPE

import roslib.exceptions
import roslib.names
import roslib.rosenv

MSG_DIR = 'msg'
SRV_DIR = 'srv'
SRC_DIR = 'src'

# aliases
ROS_PACKAGE_PATH = roslib.rosenv.ROS_PACKAGE_PATH
ROS_ROOT = roslib.rosenv.ROS_ROOT

class ROSPkgException(roslib.exceptions.ROSLibException):
    """
    Base class of package-related errors.
    """
    pass
class InvalidROSPkgException(ROSPkgException):
    """
    Exception that indicates that a ROS package does not exist
    """
    pass
class MultipleNodesException(ROSPkgException):
    """
    Exception that indicates that multiple ROS nodes by the same name are in the same package.
    """
    pass

# TODO: go through the code and eliminate unused methods -- there's far too many combos here

MANIFEST_FILE = 'manifest.xml'

#
# Map package/directory structure
#

def is_pkg_dir(d):
    """
    @param d: directory location
    @type  d: str
    @return: True if d is the root directory of a ROS Package
    @rtype: bool
    """
    return os.path.isfile(os.path.join(d, MANIFEST_FILE))

def get_package_paths(ros_root_required=True, env=None):
    """
    Get the paths to search for packages
    
    @param ros_root_required: if True, raise exception if
    environment is invalid (i.e. ROS_ROOT is not set properly)
    @type  ros_root_required: bool
    @param env: override os.environ dictionary
    @type  env: dict
    @raise roslib.rosenv.ROSEnvException: if ros_root_required is True
    and ROS_ROOT is not set
    """
    if env is None:
        env = os.environ
    rpp = roslib.rosenv.get_ros_package_path(required=False, env=env)
    if rpp:
        paths = [x for x in rpp.split(os.pathsep) if x]
    else:
        paths = []
    rr_path = roslib.rosenv.get_ros_root(required=ros_root_required, env=env)
    if rr_path:
        return paths + [rr_path]
    else:
        return paths

def get_dir_pkg(d):
    """
    Get the package that the directory is contained within. This is
    determined by finding the nearest parent manifest.xml file. This
    isn't 100% reliable, but symlinks can full any heuristic that
    relies on ROS_ROOT.
    @param d: directory path
    @type  d: str
    @return: (package_directory, package) of the specified directory, or None,None if not in a package
    @rtype: (str, str)
    """
    #TODO: the realpath is going to create issues with symlinks, most likely

    parent = os.path.dirname(os.path.realpath(d))
    #walk up until we hit ros root or ros/pkg
    while not os.path.exists(os.path.join(d, MANIFEST_FILE)) and parent != d:
        d = parent
        parent = os.path.dirname(d)
    if os.path.exists(os.path.join(d, MANIFEST_FILE)):
        pkg = os.path.basename(os.path.abspath(d))
        return d, pkg
    return None, None

_pkg_dir_cache = {}

def get_pkg_dir(package, required=True, ros_root=None, ros_package_path=None):
    """
    Locate directory package is stored in
    
    @param package: package name
    @type  package: str
    @param required: if True, an exception will be raised if the
    package directory cannot be located.
    @type  required: bool
    @param ros_root: if specified, override ROS_ROOT
    @type  ros_root: str
    @param ros_package_path: if specified, override ROS_PACKAGE_PATH
    @type  ros_package_path: str
    @return: directory containing package or None if package cannot be found and required is False.
    @rtype: str
    @raise InvalidROSPkgException: if required is True and package cannot be located
    """    

    #UNIXONLY
    #TODO: replace with non-rospack-based solution (e.g. os.walk())
    try:
        penv = os.environ.copy()
        if ros_root:
            ros_root = roslib.rosenv.resolve_path(ros_root)
            penv[ROS_ROOT] = ros_root
        elif ROS_ROOT in os.environ:
            # record setting for _pkg_dir_cache
            ros_root = os.environ[ROS_ROOT]
        if ros_root:
            rospack = os.path.join(ros_root, 'bin', 'rospack')
        else:
            rospack = 'rospack'
        if ros_package_path is not None:
            ros_package_path = roslib.rosenv.resolve_paths(ros_package_path)
            penv[ROS_PACKAGE_PATH] = ros_package_path
        elif ROS_PACKAGE_PATH in os.environ:
            # record setting for _pkg_dir_cache
            ros_package_path = os.environ[ROS_PACKAGE_PATH]

        # update cache if we haven't. NOTE: we only get one cache
        if not _pkg_dir_cache:
            _read_rospack_cache(_pkg_dir_cache, ros_root, ros_package_path)
            
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
        # don't update cache: this should only be updated from
        # rospack_cache as it will corrupt list_pkgs() otherwise.
        #_pkg_dir_cache[package] = (pkg_dir, ros_root, ros_package_path)
        return pkg_dir
    except OSError, e:
        if required:
            raise InvalidROSPkgException("Environment configuration is invalid: cannot locate rospack (%s)"%e)
        return None
    except Exception, e:
        if required:
            raise
        return None

def _get_pkg_subdir_by_dir(package_dir, subdir, required=True, env=None):
    """
    @param required: if True, will attempt to  create the subdirectory
        if it does not exist. An exception will be raised  if this fails.
    @type  required: bool
    @param package_dir: directory of package
    @type  package_dir: str
    @param subdir: name of subdirectory to locate
    @type  subdir: str
    @param env: override os.environ dictionary    
    @type  env: dict
    @param required: if True, directory must exist    
    @type  required: bool
    @return: Package subdirectory if package exist, otherwise None.
    @rtype: str
    @raise InvalidROSPkgException: if required is True and directory does not exist
    """
    if env is None:
        env = os.environ
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
    
def get_pkg_subdir(package, subdir, required=True, env=None):
    """
    @param required: if True, will attempt to create the subdirectory
        if it does not exist. An exception will be raised  if this fails.
    @type  required: bool
    @param package: name of package
    @type  package: str
    @param env: override os.environ dictionary
    @type  env: dict
    @param required: if True, directory must exist    
    @type  required: bool
    @return: Package subdirectory if package exist, otherwise None.
    @rtype: str
    @raise InvalidROSPkgException: if required is True and directory does not exist
    """
    if env is None:
        env = os.environ
    pkg_dir = get_pkg_dir(package, required, ros_root=env[ROS_ROOT]) 
    return _get_pkg_subdir_by_dir(pkg_dir, subdir, required, env)

#
# Map ROS resources to files
#

def resource_file(package, subdir, resource_name):
    """
    @param subdir: name of subdir -- these should be one of the
        string constants, e.g. MSG_DIR
    @type  subdir: str
    @return: path to resource in the specified subdirectory of the
        package, or None if the package does not exists
    @rtype: str
    @raise roslib.packages.InvalidROSPkgException: If package does not exist 
    """
    d = get_pkg_subdir(package, subdir, False)
    if d is None:
        raise InvalidROSPkgException(package)
    return os.path.join(d, resource_name)

def _read_rospack_cache(cache, ros_root, ros_package_path):
    """
    Read in rospack_cache data into cache
    @param cache: empty dictionary to store package list in. 
        If no cache argument provided, list_pkgs() will use internal _pkg_dir_cache
        and will return cached answers if available.
        The format of the cache is {package_name: dir_path, ros_root, ros_package_path}.
    @type  cache: {str: str, str, str}
    """
    try:
        with open(os.path.join(roslib.rosenv.get_ros_home(), 'rospack_cache')) as f:
            for l in f.readlines():
                l = l[:-1]
                if not len(l):
                    continue
                if l[0] == '#':
                    # check that the cache matches our env
                    if l.startswith('#ROS_ROOT='):
                        if not l[len('#ROS_ROOT='):] == ros_root:
                            return False
                    elif l.startswith('#ROS_PACKAGE_PATH='):
                        if not l[len('#ROS_PACKAGE_PATH='):]:
                            return False
                else:
                    cache[os.path.basename(l)] = l, ros_root, ros_package_path
        return True
    except:
        pass
    
# TODO: use this to replace get_pkg_dir shelling to rospack
def list_pkgs(pkg_dirs=None, cache=None):
    """
    List packages in ROS_ROOT and ROS_PACKAGE_PATH. 
    
    @param pkg_dirs: (optional) list of paths to search for packages
    @type  pkg_dirs: [str]
    @param cache: Empty dictionary to store package list in. 
        If no cache argument provided, list_pkgs() will use internal _pkg_dir_cache
        and will return cached answers if available.
        The format of the cache is {package_name: dir_path, ros_root, ros_package_path}.
    @type  cache: {str: str, str, str}
    @return: complete list of package names in ROS environment
    @rtype: [str]
    """
    if pkg_dirs is None:
        pkg_dirs = get_package_paths(True)
    if cache is None:
        cache = _pkg_dir_cache
        # if _pkg_dir_cache is already loaded, return its keys
        if cache:
            return cache.keys()
        ros_root = os.environ[ROS_ROOT]
        ros_package_path = os.environ.get(ROS_PACKAGE_PATH, '')
        if _read_rospack_cache(cache, ros_root, ros_package_path):
            return cache.keys()
    
    packages = []
    for pkg_root in pkg_dirs:
        list_pkgs_by_path(pkg_root, packages, cache=cache)
    return packages

def list_pkgs_by_path(path, packages=[], cache=None):
    """
    @param path: path to list packages in
    @type  path: str
    @param packages: list of packages to append to. If package is
      already present in packages, it will be ignored.
    @type  packages: [str]
    @param cache: (optional) package path cache to update. Maps package name to directory path.
    @type  cache: {str: str}
    @return: complete list of package names in ROS environment. Same as packages parameter.
    @rtype: [str]
    """
    # record settings for cache
    ros_root = os.environ[ROS_ROOT]
    ros_package_path = os.environ.get(ROS_PACKAGE_PATH, '')

    for d, dirs, files in os.walk(path, topdown=True):
        if MANIFEST_FILE in files:
            package = os.path.basename(d)
            if package not in packages:
                packages.append(package)
                if cache is not None:
                    cache[package] = d, ros_root, ros_package_path
            del dirs[:]
            continue #leaf
        elif 'rospack_nosubdirs' in files:
            del dirs[:]
            continue #leaf
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
                packages.extend(list_pkgs_by_path(sub_p, cache=cache))
            
    return packages

# TODO: reimplement using find_resource

def find_node(pkg, node_type, ros_root=None, ros_package_path=None):
    """
    Locate the executable that implements the node
    
    @param node_type: type of node
    @type  node_type: str
    @param ros_root: if specified, override ROS_ROOT
    @type  ros_root: str
    @param ros_package_path: if specified, override ROS_PACKAGE_PATH
    @type  ros_package_path: str
    @return: path to node or None if node is not in the package
    @rtype: str
    @raise roslib.packages.InvalidROSPkgException: If package does not exist 
    """
    dir = get_pkg_dir(pkg, required=True, \
                      ros_root=ros_root, ros_package_path=ros_package_path)
    #UNIXONLY
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

def find_resource(pkg, resource_name, filter_fn=None, ros_root=None, ros_package_path=None):
    """
    Locate the file named resource_name in package, optionally
    matching specified filter
    @param filter: function that takes in a path argument and
        returns True if the it matches the desired resource
    @type  filter: fn(str)
    @param ros_root: if specified, override ROS_ROOT
    @type  ros_root: str
    @param ros_package_path: if specified, override ROS_PACKAGE_PATH
    @type  ros_package_path: str
    @return: lists of matching paths for resource
    @rtype: [str]
    @raise roslib.packages.InvalidROSPkgException: If package does not exist 
    """
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

