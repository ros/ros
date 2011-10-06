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

"""
Python utilities for manipulating ROS packages.
See: U{http://ros.org/wiki/Packages}

Warning: while most of this API is stable, some parts are still fairly
experimental and incomplete. In particular, the L{ROSPackages} class
in very experimental.
"""

import os
import sys
import stat
import string

from subprocess import Popen, PIPE

import rospkg

import roslib.manifest
import roslib.os_detect

SRC_DIR = 'src'

# aliases
ROS_PACKAGE_PATH = rospkg.environment.ROS_PACKAGE_PATH
ROS_ROOT = rospkg.environment.ROS_ROOT

class ROSPkgException(Exception):
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
    Locate directory package is stored in. This routine uses an
    internal cache.

    NOTE: cache does *not* rebuild if packages are relocated after
    this process is initiated.
    
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
            ros_root = rospkg.environment._resolve_path(ros_root)
            penv[ROS_ROOT] = ros_root
        elif ROS_ROOT in os.environ:
            # record setting for _pkg_dir_cache
            ros_root = os.environ[ROS_ROOT]
        if ros_root:
            rospack = os.path.join(ros_root, 'bin', 'rospack')
        else:
            rospack = 'rospack'

        if 'ROS_BUILD' in os.environ:
            rospack = os.path.join(os.environ['ROS_BUILD'], 'bin', 'rospack')

        if ros_package_path is not None:
            ros_package_path = rospkg.environment._resolve_paths(ros_package_path)
            penv[ROS_PACKAGE_PATH] = ros_package_path
        elif ROS_PACKAGE_PATH in os.environ:
            # record setting for _pkg_dir_cache
            ros_package_path = os.environ[ROS_PACKAGE_PATH]

        # update cache if we haven't. NOTE: we only get one cache
        if not _pkg_dir_cache:
            _read_rospack_cache(_pkg_dir_cache, ros_root, ros_package_path)
            
        # now that we've resolved the args, check the cache
        if package in _pkg_dir_cache:
            dir_, rr, rpp = _pkg_dir_cache[package]
            if rr == ros_root and rpp == ros_package_path:
                if os.path.isfile(os.path.join(dir_, MANIFEST_FILE)):
                    return dir_
                else:
                    # invalidate cache
                    _invalidate_cache(_pkg_dir_cache)
            
        rpout, rperr = Popen([rospack, 'find', package], \
                                 stdout=PIPE, stderr=PIPE, env=penv).communicate()

        pkg_dir = (rpout or '').strip()
        #python3.1 popen returns as bytes
        if (isinstance(pkg_dir, bytes)):
            pkg_dir = pkg_dir.decode()
        if not pkg_dir:
            raise InvalidROSPkgException("Cannot locate installation of package %s: %s. ROS_ROOT[%s] ROS_PACKAGE_PATH[%s]"%(package, rperr.strip(), ros_root, ros_package_path))

        if not os.path.exists(pkg_dir):
            raise InvalidROSPkgException("Cannot locate installation of package %s: [%s] is not a valid path. ROS_ROOT[%s] ROS_PACKAGE_PATH[%s]"%(package, pkg_dir, ros_root, ros_package_path))
        elif not os.path.isdir(pkg_dir):
            raise InvalidROSPkgException("Package %s is invalid: file [%s] is in the way"%(package, pkg_dir))
        # don't update cache: this should only be updated from
        # rospack_cache as it will corrupt package list otherwise.
        #_pkg_dir_cache[package] = (pkg_dir, ros_root, ros_package_path)
        return pkg_dir
    except OSError as e:
        if required:
            raise InvalidROSPkgException("Environment configuration is invalid: cannot locate rospack (%s)"%e)
        return None
    except Exception as e:
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
        d = os.path.join(package_dir, subdir)
        if required and os.path.isfile(d):
            raise Exception("""Package '%(package)s' is improperly configured: 
file %(d)s is preventing the creation of a directory"""%locals())
        elif required and not os.path.isdir(d):
            try:
                os.makedirs(d) #lazy create
            except error:
                raise Exception("""Package '%(package)s' is improperly configured: 
Cannot create a '%(subdir)s' directory in %(package_dir)s.
Please check permissions and try again.
"""%locals())
        return d
    except Exception as e:
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

def _update_rospack_cache(env=None):
    """
    Internal routine to update global package directory cache
    
    @return: True if cache is valid
    @rtype: bool
    """
    if env is None:
        env = os.environ
    cache = _pkg_dir_cache
    if cache:
        return True
    ros_root = env[ROS_ROOT]
    ros_package_path = env.get(ROS_PACKAGE_PATH, '')
    return _read_rospack_cache(cache, ros_root, ros_package_path)

def _invalidate_cache(cache):
    # I've only made this a separate routine because roslib.packages should really be using
    # the roslib.stacks cache implementation instead with the separate cache marker
    cache.clear()

def _read_rospack_cache(cache, ros_root, ros_package_path):
    """
    Read in rospack_cache data into cache. On-disk cache specifies a
    ROS_ROOT and ROS_PACKAGE_PATH, which must match the requested
    environment.
    
    @param cache: empty dictionary to store package list in. 
        If no cache argument provided, will use internal _pkg_dir_cache
        and will return cached answers if available.
        The format of the cache is {package_name: dir_path, ros_root, ros_package_path}.
    @type  cache: {str: str, str, str}
    @param ros_package_path: ROS_ROOT value
    @type  ros_root: str
    @param ros_package_path: ROS_PACKAGE_PATH value or '' if not specified
    @type  ros_package_path: str
    @return: True if on-disk cache matches and was loaded, false otherwise
    @rtype: bool
    """
    try:
        with open(os.path.join(rospkg.get_ros_home(), 'rospack_cache')) as f:
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
                        if not l[len('#ROS_PACKAGE_PATH='):] == ros_package_path:
                            return False
                else:
                    cache[os.path.basename(l)] = l, ros_root, ros_package_path
        return True
    except:
        pass
    
def list_pkgs_by_path(path, packages=None, cache=None, env=None):
    """
    List ROS packages within the specified path.

    Optionally, a cache dictionary can be provided, which will be
    updated with the package->path mappings. list_pkgs_by_path() does
    NOT returned cached results -- it only updates the cache.
    
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
    if packages is None:
        packages = []
    if env is None:
        env = os.environ
    # record settings for cache
    ros_root = env[ROS_ROOT]
    ros_package_path = env.get(ROS_PACKAGE_PATH, '')

    path = os.path.abspath(path)
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

    if 'ROS_BUILD' in os.environ:
        tst = os.path.join(os.environ['ROS_BUILD'], 'bin', node_type)
        if os.path.isfile(tst):
            return tst

    d = get_pkg_dir(pkg, required=True, \
                    ros_root=ros_root, ros_package_path=ros_package_path)
    
    #UNIXONLY: (partial) slowly supporting Windows here
    if sys.platform in ['win32', 'cygwin']:
        # Windows logic requires more file patterns to resolve and is
        # not case-sensitive, so leave it separate

        # in the near-term, just hack in support for .exe/.bat. In the long
        # term this needs to:
        #
        #  * parse PATHEXT to generate matches
        #  * perform case-insensitive compares against potential
        #    matches, in path-ext order

        # - We still have to look for bare node_type as user may have
        #   specified extension manually
        node_type = node_type.lower()
        matches = [node_type, node_type+'.exe', node_type+'.bat']
        for p, dirs, files in os.walk(d):
            # case insensitive
            files = [f.lower() for f in files]
            for m in matches:
                if m in files:
                    test_path = os.path.join(p, m)
                    s = os.stat(test_path)
                    if (s.st_mode & (stat.S_IRUSR | stat.S_IXUSR) ==
                        (stat.S_IRUSR | stat.S_IXUSR)):
                        return test_path
            if '.svn' in dirs:
                dirs.remove('.svn')
            elif '.git' in dirs:
                dirs.remove('.git')
    else:
        #TODO: this could just execute find_resource with a filter_fn
        for p, dirs, files in os.walk(d):
            if node_type in files:
                test_path = os.path.join(p, node_type)
                s = os.stat(test_path)
                if (s.st_mode & (stat.S_IRUSR | stat.S_IXUSR) ==
                    (stat.S_IRUSR | stat.S_IXUSR)):
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
    d = get_pkg_dir(pkg, required=True, \
                    ros_root=ros_root, ros_package_path=ros_package_path)
    #UNIXONLY
    matches = []
    node_exe = None
    for p, dirs, files in os.walk(d):
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

def rosdeps_of(packages):
    """
    Collect all rosdeps of specified packages into a dictionary.
    @param packages: package names
    @type  packages: [str]
    @return: dictionary mapping package names to list of rosdep names.
    @rtype: {str: [str]}
    """
    if not type(packages) in [list, tuple]:
        raise TypeError("packages must be list or tuple")
    _update_rospack_cache()
    from roslib.manifest import load_manifest
    manifests = [load_manifest(p) for p in packages]
    map = {}
    for pkg, m in zip(packages, manifests): #py3k
        map[pkg] = [d.name for d in m.rosdeps]
    return map

def _safe_load_manifest(p):
    """
    Calls roslib.manifest.load_manifest and returns None if the calls raises an Exception (i.e. invalid package)
    """
    try:
        return roslib.manifest.load_manifest(p)
    except:
        return roslib.manifest.Manifest()

class ROSPackages(object):
    """
    UNSTABLE/EXPERIMENTAL
    
    Utility class for querying properties about ROS packages. This
    should be used when querying properties about multiple
    packages. ROSPackages caches information about packages, which
    enables it to have higher performance than alternatives like
    shelling out to rospack.

    Example::
      rp = ROSPackages()
      d = rp.depends1(['roscpp', 'rospy'])
      print d['roscpp']
      d = rp.rosdeps(['roscpp', 'rospy'])
      print d['rospy']
    """
    
    def __init__(self):
        self.manifests = {}
        self._depends_cache = {}
        self._rosdeps_cache = {}

    def load_manifests(self, packages):
        """
        Load manifests for specified packages into 'manifests' attribute.
        
        
        @param packages: package names
        @type  packages: [str]
        """

        if not type(packages) in [list, tuple]:
            raise TypeError("packages must be list or tuple")

        # load any manifests that we haven't already
        to_load = [p for p in packages if not p in self.manifests]
        if to_load:
            _update_rospack_cache()
            self.manifests.update(dict([(p, _safe_load_manifest(p)) for p in to_load]))
        
    def depends1(self, packages):
        """
        Collect all direct dependencies of specified packages into a
        dictionary.
        
        @param packages: package names
        @type  packages: [str]
        @return: dictionary mapping package names to list of dependent package names.
        @rtype: {str: [str]}
        """
        self.load_manifests(packages)
        map = {}
        manifests = self.manifests
        for pkg in packages:
            map[pkg] = [d.package for d in manifests[pkg].depends]
        return map

    def depends(self, packages):
        """
        Collect all dependencies of specified packages into a
        dictionary.
        
        @param packages: package names
        @type  packages: [str]
        @return: dictionary mapping package names to list of dependent package names.
        @rtype: {str: [str]}
        """

        self.load_manifests(packages)
        map = {}
        for pkg in packages:
            if pkg in self._depends_cache:
                map[pkg] = self._depends_cache[pkg]
            else:
                # this will cache for future reference
                map[pkg] = self._depends(pkg)
        return map

    def _depends(self, package):
        """
        Compute recursive dependencies of a single package and cache
        the result in self._depends_cache.

        This is an internal routine. It assumes that
        load_manifests() has already been invoked for package.
        
        @param package: package name
        @type  package: str
        @return: list of rosdeps
        @rtype: [str]
        """

        if package in self._depends_cache:
            return self._depends_cache[package]

        # assign key before recursive call to prevent infinite case
        self._depends_cache[package] = s = set()
        
        manifests = self.manifests
        # take the union of all dependencies
        pkgs = [p.package for p in manifests[package].depends]
        self.load_manifests(pkgs)
        for p in pkgs:
            s.update(self._depends(p))
        # add in our own deps
        s.update(pkgs)
        # cache the return value as a list
        s = list(s)
        self._depends_cache[package] = s
        return s
    
    def rosdeps0(self, packages):
        """
        Collect rosdeps of specified packages into a dictionary.
        @param packages: package names
        @type  packages: [str]
        @return: dictionary mapping package names to list of rosdep names.
        @rtype: {str: [str]}
        """

        self.load_manifests(packages)
        map = {}
        manifests = self.manifests
        for pkg in packages:
            map[pkg] = [d.name for d in manifests[pkg].rosdeps]
        return map
        
    def rosdeps(self, packages):
        """
        Collect all (recursive) dependencies of specified packages
        into a dictionary.
        
        @param packages: package names
        @type  packages: [str]
        @return: dictionary mapping package names to list of dependent package names.
        @rtype: {str: [str]}
        """

        self.load_manifests(packages)
        map = {}
        for pkg in packages:
            if pkg in self._rosdeps_cache:
                map[pkg] = self._rosdeps_cache[pkg]
            else:
                # this will cache for future reference
                map[pkg] = self._rosdeps(pkg)
        return map

    def _rosdeps(self, package):
        """
        Compute recursive rosdeps of a single package and cache the
        result in self._rosdeps_cache.

        This is an internal routine. It assumes that
        load_manifests() has already been invoked for package.
        
        @param package: package name
        @type  package: str
        @return: list of rosdeps
        @rtype: [str]
        """

        if package in self._rosdeps_cache:
            return self._rosdeps_cache[package]
        # set the key before recursive call to prevent infinite case
        self._rosdeps_cache[package] = s = set()

        manifests = self.manifests
        # take the union of all dependencies
        pkgs = [p.package for p in manifests[package].depends]
        self.load_manifests(pkgs)
        for p in pkgs:
            s.update(self._rosdeps(p))
        # add in our own deps
        s.update([d.name for d in manifests[package].rosdeps])
        # cache the return value as a list
        s = list(s)
        self._rosdeps_cache[package] = s
        return s
        
def _platform_supported(file, os, version):
    m = roslib.manifest.parse_file(file)
    for p in m.platforms:
        if os == p.os and version == p.version:
            return True
    return False

def platform_supported(pkg, os, version):
    """
    Return whether the platform defined by os and version is marked as supported in the package
    @param pkg The package to test for support
    @param os The os name to test for support
    @param version The os version to test for support
    """
    return _platform_supported(roslib.manifest.manifest_file(pkg), os, version)

def current_platform_supported(pkg):
    """
    Return whether the current running platform is marked as supported in the package
    @param pkg The package to test for support
    """
    os_detector = roslib.os_detect.OSDetect()
    return platform_supported(pkg, os_detector.get_name(), os_detector.get_version())

