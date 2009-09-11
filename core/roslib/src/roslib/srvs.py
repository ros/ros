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
# Revision $Id: srvspec.py 3357 2009-01-13 07:13:05Z jfaustwg $
# $Author: jfaustwg $
"""ROS Service Description Language Spec"""
## ROS Service Description Language Spec
#  Implements http://pr.willowgarage.com/wiki/ROS/Message_Description_Language

import os
import re
import cStringIO

import roslib.exceptions
import roslib.msgs
import roslib.names
import roslib.packages
import roslib.resources

# don't directly use code from this, though we do depend on the
# manifest.Depend data type
import roslib.manifest

## file extension
EXT = roslib.names.SRV_EXT #alias
SEP = roslib.names.PRN_SEPARATOR #e.g. std_msgs/String
## input/output deliminator
IODELIM   = '---'
COMMENTCHAR = roslib.msgs.COMMENTCHAR

VERBOSE = False
## @return: True if msg-related scripts should print verbose output
def is_verbose():
    return VERBOSE

## set whether msg-related scripts should print verbose output
def set_verbose(v):
    global VERBOSE
    VERBOSE = v

class SrvSpecException(roslib.exceptions.ROSLibException): pass

# msg spec representation ##########################################

class SrvSpec(object):
    
    def __init__(self, request, response, text):
        self.request = request
        self.response = response
        self.text = text
        
    def __eq__(self, other):
        if not other or not isinstance(other, SrvSpec):
            return False
        return self.msgIn == other.msgIn and self.msgOut == other.msgOut
    def __ne__(self, other):
        if not other or not isinstance(other, SrvSpec):
            return True
        return not self.__eq__(other)

    def __repr__(self):
        return "SrvSpec[%s, %s]"%(repr(self.msgIn), repr(self.msgOut))
    
# srv spec loading utilities ##########################################

## @internal
## predicate for filtering directory list. matches message files
def _srv_filter(f):
    return os.path.isfile(f) and f.endswith(EXT)

# also used by doxymaker
## list all services in the specified package
## @param package str: name of package to search
## @param include_depends bool: if True, will also list services in package dependencies
## @return [str]: service type names
def list_srv_types(package, include_depends):
    types = roslib.resources.list_package_resources(package, include_depends, roslib.packages.SRV_DIR, _srv_filter)
    return [x[:-len(EXT)] for x in types]

## @param package str: name of package .srv file is in
## @param type str: type name of service
## @return str: file path of .srv file in specified package
def srv_file(package, type):
    return roslib.packages.resource_file(package, roslib.packages.SRV_DIR, type+EXT)

## List all messages that a package contains
#  @param depend Depend: roslib.manifest.Depend object representing package
#  to load messages from
#  @param nameContext str: package prefix for message type names
#  @return [(str,roslib.MsgSpec), [str]]: list of message type names and specs for package, as well as a list
#      of message names that could not be processed. 
def get_pkg_srv_specs(depend, nameContext):
    #almost identical to roslib.msgs.get_pkg_msg_specs
    types = list_srv_types(depend.package, False)
    specs = [] #no fancy list comprehension as we want to show errors
    failures = []
    for t in types:
        try: 
            spec = load_from_file(srv_file(depend.package, t), nameContext)
            specs.append(spec)
        except Exception, e:
            failures.append(t)
            print "ERROR: unable to load %s"%t
    return specs, failures

##
#  @param text str: .msg text 
#  @param package_context: context to use for msgTypeName, i.e. the package name,
#      or '' to use local naming convention.
#  @return Roslib.MsgSpec: Message type name and message specification
#  @throws Roslib.MsgSpecException: if syntax errors or other problems are detected in file
def load_from_string(text, package_context=''):
    textIn  = cStringIO.StringIO()
    textOut = cStringIO.StringIO()
    accum = textIn
    for l in text.split('\n'):
        l = l.split(COMMENTCHAR)[0].strip() #strip comments        
        if l.startswith(IODELIM): #lenient, by request
            accum = textOut
        else:
            accum.write(l+'\n')
    # create separate roslib.msgs objects for each half of file
    msgIn, msgOut = [roslib.msgs.load_from_string(b.getvalue()) for b in [textIn, textOut]]
    return SrvSpec(msgIn, msgOut, text)

## Convert the .srv representation in the file to a SrvSpec instance.
#  @param package_context: context to use for type name, i.e. the package name,
#      or '' to use local naming convention.
#  @param file str: name of file to load from
#  @return (str, L{SrvSpec}): Message type name and message specification
#  @throws SrvSpecException: if syntax errors or other problems are detected in file
def load_from_file(file, package_context=''):
    if VERBOSE:
        if package_context:
            print "Load spec from", file, "into namespace [%s]"%package_context
        else:
            print "Load spec from", file            
    fileName = os.path.basename(file)
    type = fileName[:-len(EXT)]
    # determine the type name
    if package_context:
        while package_context.endswith(SEP):
            package_context = package_context[:-1] #strip message separators
        type = "%s%s%s"%(package_context, SEP, type)
    if not roslib.names.is_valid_local_name(type):
        raise SrvSpecException("%s: %s is not a legal service type name"%(file, type))
    
    f = open(file, 'r')
    try:
        text = f.read()
        return (type, load_from_string(text, package_context))
    finally:
        f.close()




