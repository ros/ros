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

import os
import itertools

import roslib.manifest
import roslib.names
import roslib.packages

################################################################################
# List ROS package resources


## Helper routine for loading Manifest instances
## @param package_dir str: package directory location
## @return Manifest: manifest for package
def _get_manifest_by_dir(package_dir):
    f = os.path.join(package_dir, roslib.manifest.MANIFEST_FILE)
    if f:
        return roslib.manifest.parse_file(f)
    else:
        return None

## List resources in a package directory within a particular
## subdirectory. This is useful for listing messages, services, etc...
## @param package_dir str: package directory location
## @param subdir str: name of subdirectory
## @param include_depends bool: if True, include resources in dependencies as well    
## @param rfilter fn: resource filter function that returns true if  filenames is the desired resource type
def list_package_resources_by_dir(package_dir, include_depends, subdir, rfilter=os.path.isfile):
    package = os.path.basename(package_dir)
    resources = []
    dir = roslib.packages._get_pkg_subdir_by_dir(package_dir, subdir, False)
    if os.path.isdir(dir):
        resources = [roslib.names.resource_name(package, f, my_pkg=package) \
                     for f in os.listdir(dir) if rfilter(os.path.join(dir, f))]
    else:
        resources = []
    if include_depends:
        depends = _get_manifest_by_dir(package_dir).depends
        dirs = [roslib.packages.get_pkg_subdir(d.package, subdir, False) for d in depends]
        dirs = filter(lambda dir: dir and os.path.isdir(dir), dirs)
        for (dep, dir) in itertools.izip(depends, dirs):
            resources.extend(\
                [roslib.names.resource_name(dep.package, f, my_pkg=package) \
                 for f in os.listdir(dir) if rfilter(os.path.join(dir, f))])
    return resources

## List resources in a package within a particular subdirectory. This is useful for listing
## messages, services, etc...    
## @param package str: package name
## @param subdir str: name of subdirectory
## @param include_depends bool: if True, include resources in dependencies as well    
## @param rfilter fn: resource filter function that returns true if  filenames is the desired resource type
def list_package_resources(package, include_depends, subdir, rfilter=os.path.isfile):
    package_dir = roslib.packages.get_pkg_dir(package)
    return list_package_resources_by_dir(package_dir, include_depends, subdir, rfilter)

