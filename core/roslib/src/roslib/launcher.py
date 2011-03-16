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
# $Author$

"""
Python path loader for python scripts and applications. Paths are
derived from dependency structure declared in ROS manifest files.
"""

import sys
import os

import roslib.manifest
import roslib.packages

def get_manifest_file(package_name):
    """
    @return: name of package to get manifest for
    @rtype: str
    @raise InvalidROSPkgException: if required is True and package cannot be located
    """
    return roslib.manifest.manifest_file(package_name, required=True)
        
# bootstrapped keeps track of which packages we've loaded so we don't update the path multiple times
_bootstrapped = []

def load_manifest(package_name, bootstrap_version="0.7"):
    """
    Update the Python sys.path with package's dependencies
    @param package_name: name of the package that load_manifest() is being called from.
    @type  package_name: str
    @param bootstrap_version: (keyword argument) do not use. Soon to be deprecated
    @type  bootstrap_version: str
    """
    if package_name in _bootstrapped:
        return
    prefix = []
    if 'ROS_BUILD' in os.environ:
        if os.environ['ROS_BUILD'] == os.environ['ROS_ROOT']:
            return
        prefix = [os.path.join(os.environ['ROS_BUILD'], 'gen', 'py'),
                  os.path.join(os.environ['ROS_BUILD'], '..', 'rosidl', 'src')]
    sys.path = prefix + _generate_python_path(package_name, [], os.environ) + sys.path
    
def _append_package_paths(manifest_, paths, pkg_dir):
    """
    Added paths for package to paths
    @param manifest_: package manifest
    @type  manifest_: Manifest
    @param pkg_dir: package's filesystem directory path
    @type  pkg_dir: str
    @param paths: list of paths
    @type  paths: [str]
    """
    exports = manifest_.get_export('python','path')
    if exports:
        for export in exports:
            if ':' in export:
                export = export.split(':')
            else:
                export = [export]
            for e in export:
                paths.append(e.replace('${prefix}', pkg_dir))
    else:
        dirs = [os.path.join(pkg_dir, d) for d in ['src', 'lib']]
        paths.extend(filter(os.path.isdir, dirs))
    
def _generate_python_path(pkg, depends, env=os.environ):
    """
    Recursive subroutine for building dependency list and python path
    @param manifest_file: manifest to parse for additional dependencies
    @param depends: current dependency set. Will be modified
    @return: list of directory paths to add to python path in order to include
      package and dependencies described in manifest file.
    @raise InvalidROSPkgException: if an error occurs while attempting to load package or dependencies
    """
    if pkg in _bootstrapped:
        return []
    manifest_file = roslib.manifest.manifest_file(pkg, True, env)
    if not manifest_file:
        raise roslib.packages.InvalidROSPkgException("cannot locate package [%s]"%pkg)
    _bootstrapped.append(pkg)
    
    pkg_dir = os.path.dirname(os.path.abspath(manifest_file))
    depends.append(pkg)
    m = roslib.manifest.parse_file(manifest_file)
    
    paths = []
    _append_package_paths(m, paths, pkg_dir)

    try:
        for d in m.depends:
            if d.package in depends:
                continue 
            try: #add sub-dependencies to paths and depends
                paths.extend(_generate_python_path(d.package, depends, env))
            except roslib.packages.InvalidROSPkgException, e:
                # translate error message to give more context
                raise roslib.packages.InvalidROSPkgException("While loading package '%s': %s"%(d.package, str(e)))
            except:
                import traceback
                raise roslib.packages.InvalidROSPkgException("While loading package '%s': cannot load dependency '%s'\nLower level error was %s"%(pkg, d.package, traceback.format_exc()))
    except:
        if pkg in _bootstrapped:
            _bootstrapped.remove(pkg)
        raise
    return paths
