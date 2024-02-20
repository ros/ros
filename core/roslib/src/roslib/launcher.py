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

#! /usr/bin/env python
"""
Python path loader for python scripts and applications. Paths are
derived from dependency structure declared in ROS manifest files.
"""

import os
import sys
import rospkg

# Global variables to track bootstrapped packages and ROS package data
_bootstrapped = []
_rospack = rospkg.RosPack()


def get_depends(package, rospack):
    """
    Retrieves the dependencies of a given package.

    Args:
        package (str): The name of the package.
        rospack (rospkg.RosPack): Instance of RosPack for accessing package information.

    Returns:
        list[str]: A list of dependencies for the given package.
    """
    vals = rospack.get_depends(package, implicit=True)
    return [v for v in vals if not rospack.get_manifest(v).is_catkin]


def load_manifest(package_name):
    """
    Updates the Python sys.path with the specified package's dependencies.

    Args:
        package_name (str): Name of the package from which load_manifest() is called.

    Returns:
        None
    """
    if package_name not in _bootstrapped:
        sys.path[0:0] = _generate_python_path(package_name, _rospack)


def _append_package_paths(manifest_, paths, pkg_dir):
    """
    Adds paths for a package to the given paths list.

    Args:
        manifest_ (rospkg.Manifest): Package manifest.
        paths (list[str]): List of paths to be updated.
        pkg_dir (str): Filesystem directory path of the package.

    Returns:
        None
    """
    exports = manifest_.get_export('python', 'path')
    for export in exports:
        expanded_exports = export.split(':') if ':' in export else [export]
        paths.extend(e.replace('${prefix}', pkg_dir) for e in expanded_exports)

    if not exports:
        dirs = [os.path.join(pkg_dir, d) for d in ['src', 'lib']]
        paths.extend(d for d in dirs if os.path.isdir(d))


def _generate_python_path(pkg, rospack):
    """
    Recursive subroutine for building dependency list and python path.

    Args:
        pkg (str): Name of the package.
        rospack (rospkg.RosPack): Instance of RosPack for accessing package information.

    Returns:
        list[str]: A list of paths for the specified package and its dependencies.

    Raises:
        rospkg.ResourceNotFound: If an error occurs while attempting to load package or dependencies.
    """
    if pkg in _bootstrapped:
        return []

    m = rospack.get_manifest(pkg)
    if m.is_catkin:
        _bootstrapped.append(pkg)
        return []

    packages = get_depends(pkg, rospack) + [pkg]

    paths = []
    for p in packages:
        m = rospack.get_manifest(p)
        d = rospack.get_path(p)
        _append_package_paths(m, paths, d)
        _bootstrapped.append(p)
    return paths
