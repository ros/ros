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
# Revision $Id: manifest.py 3357 2009-01-13 07:13:05Z jfaustwg $
# $Author: jfaustwg $
"""Python parser for rospack manifest.xml files"""
## Python parser for rospack manifest.xml files
## See: http://pr.willowgarage.com/wiki/Packages

import sys
import os
import getopt

import roslib.exceptions
import roslib.packages
import roslib.rosenv

MANIFEST_FILE = 'manifest.xml'

import roslib.manifestlib
# re-export symbols for backwards compatibility
from roslib.manifestlib import ManifestException, Depend, Export, ROSDep, VersionControl

## object representation of a ROS manifest file
class Manifest(roslib.manifestlib._Manifest):
    __slots__ = []
    def __init__(self):
        super(Manifest, self).__init__('package')
        
    ## Get exports that match the specified tag and attribute, e.g. 'python', 'path'
    def get_export(self, tag, attr):
        return [e.get(attr) for e in self.exports if e.tag == tag if e.get(attr) is not None]

## @param package_dir str: path to package directory
## @param environ dict: environment dictionary
## @param required bool: require that the directory exist
## @return str: path to manifest file of package
## @throws InvalidROSPkgException if required is True and manifest file cannot be located
def _manifest_file_by_dir(package_dir, required=True, environ=os.environ):
    try:
        p = os.path.join(package_dir, MANIFEST_FILE)
        if not required and not os.path.exists(p):
            return p
        if not os.path.isfile(p):
            raise roslib.packages.InvalidROSPkgException("""
Package '%(package_dir)s' is improperly configured: no manifest file is present.
"""%locals())
        return p
    except roslib.packages.InvalidROSPkgException, e:
        if required:
            raise

## @param package str: package name
## @param environ dict: environment dictionary
## @param required bool: require that the directory exist
## @return str: path to manifest file of package
## @throws InvalidROSPkgException if required is True and manifest file cannot be located
def manifest_file(package, required=True, environ=os.environ):
    # ros_root needs to be determined from the environment or else
    # everything breaks when trying to launch nodes via ssh where the
    # path isn't setup correctly.
    d = roslib.packages.get_pkg_dir(package, required, ros_root=environ[roslib.rosenv.ROS_ROOT]) 
    return _manifest_file_by_dir(d, required, environ)
        
## Parse manifest.xml file
## @param file str: manifest.xml file path
## @return Manifest
def parse_file(file):
    return roslib.manifestlib.parse_file(Manifest(), file)

## Parse manifest.xml string contents
## @param string str: manifest.xml contents
## @return Manifest
def parse(string, filename='string'):
    return roslib.manifestlib.parse(Manifest(), string, filename)
