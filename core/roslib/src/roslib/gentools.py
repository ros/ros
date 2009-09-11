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
# Revision $Id: gentools.py 2843 2008-11-17 20:23:24Z sfkwc $
# $Author: sfkwc $

## Library for supporting message and service generation for all
## client libraries. This is mainly responsible for calculating the
## md5sums and message definitions of classes.

# NOTE: this should not contain any rospy-specific code. The rospy
# generator library is rospy.genpy.

import sys
import cStringIO

import roslib.msgs 
import roslib.names 
import roslib.packages 
import roslib.srvs 

## @internal
## Add the list of message types that \a spec depends on to \a
## depends.
## @param spec Msgs: message spec to compute dependencies for
## @param deps [str]: list of dependencies. This list will be updated
## with the dependencies of \a spec when the method completes
def _add_msgs_depends(spec, deps, package_context):
    for t in spec.types:
        t = roslib.msgs.base_msg_type(t)
        if not roslib.msgs.is_builtin(t):
            # special mapping for header
            if t == roslib.msgs.HEADER:
                # have to re-names Header
                deps.append('roslib/Header')
            if roslib.msgs.is_registered(t):
                depspec = roslib.msgs.get_registered(t)
                if t != roslib.msgs.HEADER:
                    if '/' in t:
                        deps.append(t)
                    else:
                        deps.append(package_context+'/'+t)
            else:
                #lazy-load
                key, depspec = roslib.msgs.load_by_type(t, package_context)
                if t != roslib.msgs.HEADER:
                  deps.append(key)
                roslib.msgs.register(key, depspec)
            _add_msgs_depends(depspec, deps, package_context)

## @internal
## subroutine of compute_md5()/compute_sha1()
## @param get_deps_dict dict: dictionary returned by get_dependencies call
## @param hash hash instance            
def _compute_hash(get_deps_dict, hash):
    uniquedeps = get_deps_dict['uniquedeps']
    spec = get_deps_dict['spec']    
    # accumulate the hash
    # - root file
    hash.update(spec.text)
    # - dependencies
    for d in uniquedeps:
        hash.update(roslib.msgs.get_registered(d).text)
    return hash.hexdigest()
    
## Compute md5 hash for message/service
## @param get_deps_dict dict: dictionary returned by get_dependencies call
## @return str md5 hash
def compute_md5(get_deps_dict):
    try:
        # md5 is deprecated in Python 2.6 in favor of hashlib, but hashlib is
        # unavailable in Python 2.4
        import hashlib
        return _compute_hash(get_deps_dict, hashlib.md5())
    except ImportError:
        import md5
        return _compute_hash(get_deps_dict, md5.new())

## Compute sha1 hash for message/service
## @param get_deps_dict dict: dictionary returned by get_dependencies call
## @return str sha1 hash
def compute_sha1(get_deps_dict):
    import sha
    return _compute_hash(get_deps_dict, sha.new())

## Compute full text of message/service, including text of embedded
## types.  The text of the main msg/srv is listed first. Embedded
## msg/srv files are denoted first by an 80-character '=' separator,
## followed by a type declaration line,'MSG: pkg/type', followed by
## the text of the embedded type.
## @param get_deps_dict dict: dictionary returned by get_dependencies call
## @return str concatenated text for msg/srv file and embedded msg/srv types.
def compute_full_text(get_deps_dict):
    buff = cStringIO.StringIO()
    sep = '='*80+'\n'

    # write the text of the top-level type
    buff.write(get_deps_dict['spec'].text)
    buff.write('\n')    
    # append the text of the dependencies (embedded types)
    for d in get_deps_dict['uniquedeps']:
        buff.write(sep)
        buff.write("MSG: %s\n"%d)
        buff.write(roslib.msgs.get_registered(d).text)
        buff.write('\n')
    return buff.getvalue()

## Compute dependencies of the specified message/service file
## @param f str: message or service file to get dependencies for
## @param stdout pipe: stdout pipe
## @param stderr pipe: stderr pipe
## @return dict: 'files': list of files that \a file depends on,
## 'deps': list of dependencies by type, 'spec': Msgs/Srvs
## instance.
def get_file_dependencies(f, stdout=sys.stdout, stderr=sys.stderr):
    _, package = roslib.packages.get_dir_pkg(f)
    spec = None
    if f.endswith(roslib.msgs.EXT):
        _, spec = roslib.msgs.load_from_file(f)
    elif f.endswith(roslib.srvs.EXT):
        _, spec = roslib.srvs.load_from_file(f)
    else:
        raise Exception("[%s] does not appear to be a message or service"%spec)
    return get_dependencies(spec, package, stdout, stderr)

## Compute dependencies of the specified Msgs/Srvs
## @param spec Msgs/Srvs: message or service spec instance
## @param package str: package name
## @param stdout pipe: (optional) stdout pipe
## @param stderr pipe: (optional) stderr pipe
## @param compute_files bool: (optional, default=True) compute file
## dependencies of message ('files' key in return value)
## @return dict: 'files': list of files that \a file depends on,
## 'deps': list of dependencies by type, 'spec': Msgs/Srvs
## instance. 'uniquedeps': list of dependencies with duplicates removed.
def get_dependencies(spec, package, compute_files=True, stdout=sys.stdout, stderr=sys.stderr):

    # #518: as a performance optimization, we're going to manually control the loading
    # of msgs instead of doing package-wide loads.
    
    #we're going to manipulate internal apis of msgs, so have to
    #manually init
    roslib.msgs._init()

    deps = []
    if isinstance(spec, roslib.msgs.MsgSpec):
        _add_msgs_depends(spec, deps, package)
    elif isinstance(spec, roslib.srvs.SrvSpec):
        _add_msgs_depends(spec.request, deps, package)
        _add_msgs_depends(spec.response, deps, package)                
    else:
        raise Exception("[%s] does not appear to be a message or service"%spec)

    # convert from type names to file names
    
    if compute_files:
        files = {}
        for d in set(deps):
            d_pkg, t = roslib.names.package_resource_name(d)
            d_pkg = d_pkg or package # convert '' -> local package 
            files[d] = roslib.msgs.msg_file(d_pkg, t)
    else:
        files = None

    # create unique dependency list
    uniquedeps = []
    for d in deps:
        if not d in uniquedeps:
            uniquedeps.append(d)

    if compute_files:
        return { 'files': files, 'deps': deps, 'spec': spec, 'uniquedeps': uniquedeps }
    else:
        return { 'deps': deps, 'spec': spec, 'uniquedeps': uniquedeps }        



