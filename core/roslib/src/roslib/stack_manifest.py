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
"""Python parser for rospack stack.xml files"""
## Python parser for rospack stack.xml files
## See: http://pr.willowgarage.com/wiki/Packages

import sys
import os
import getopt

import roslib.exceptions
import roslib.packages
import roslib.rosenv

STACK_FILE = 'stack.xml'

import roslib.manifestlib
# re-export symbols so that external code does not have to import manifestlib as well
from roslib.manifestlib import ManifestException, StackDepend

## object representation of a ROS manifest file
class StackManifest(roslib.manifestlib._Manifest):
    __slots__ = []
    def __init__(self):
        super(StackManifest, self).__init__('stack')
        
## @param stack_dir str: path to stack directory
## @param required bool: require that the directory exist
## @return str: path to manifest file of stack
## @throws InvalidROSPkgException if required is True and manifest file cannot be located
def _stack_file_by_dir(stack_dir, required=True):
    try:
        p = os.path.join(stack_dir, STACK_FILE)
        if not required and not os.path.exists(p):
            return p
        if not os.path.isfile(p):
            raise roslib.stacks.InvalidROSStackException("""
Stack '%(stack_dir)s' is improperly configured: no manifest file is present.
"""%locals())
        return p
    except roslib.stacks.InvalidROSStackException, e:
        if required:
            raise

## @param stack str: stack name
## @param required bool: require that the directory exist
## @return str: path to manifest file of stack
## @throws InvalidROSPkgException if required is True and manifest file cannot be located
def stack_file(stack, required=True):
    d = roslib.stacks.get_stack_dir(stack)
    return _stack_file_by_dir(d, required)
        
## @internal
## Parse stack.xml file
## @param file str: stack.xml file path
## @return StackManifest
def parse_file(file):
    return roslib.manifestlib.parse_file(StackManifest(), file)

## Parse stack.xml string contents
## @param string str: stack.xml contents
## @return StackManifest
def parse(string, filename='string'):
    s = roslib.manifestlib.parse(StackManifest(), string, filename)
    #TODO: validate
    return s
