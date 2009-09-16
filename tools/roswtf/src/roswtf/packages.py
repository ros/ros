# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

import os
import time

from roswtf.environment import paths, is_executable
from roswtf.rules import warning_rule, error_rule

import roslib.msgs
import roslib.packages
import roslib.srvs

## look for unbuilt .msg files
def msgs_built(ctx):
    unbuilt = set([])
    for pkg in ctx.pkgs:
        pkg_dir = roslib.packages.get_pkg_dir(pkg)
        mtypes = roslib.msgs.list_msg_types(pkg, False)
        for t in mtypes:
            expected = [os.path.join('msg', 'cpp', pkg, '%s.h'%t),
                        os.path.join('msg', 'lisp', pkg, '%s.lisp'%t),
                        os.path.join('src', pkg, 'msg', '_%s.py'%t)]
            for e in expected:
                if not os.path.isfile(os.path.join(pkg_dir, e)):
                    unbuilt.add(pkg)
    return list(unbuilt)

## look for unbuilt .srv files
def srvs_built(ctx):
    unbuilt = set([])
    for pkg in ctx.pkgs:
        pkg_dir = roslib.packages.get_pkg_dir(pkg)
        mtypes = roslib.srvs.list_srv_types(pkg, False)
        for t in mtypes:
            expected = [os.path.join('srv', 'cpp', pkg, '%s.h'%t),
                        os.path.join('srv', 'lisp', pkg, '%s.lisp'%t),
                        os.path.join('src', pkg, 'srv', '_%s.py'%t)]
            for e in expected:
                if not os.path.isfile(os.path.join(pkg_dir, e)):
                    unbuilt.add(pkg)
    return list(unbuilt)

def _manifest_msg_srv_export(ctx, type_):
    missing = []
    for pkg in ctx.pkgs:
        pkg_dir = roslib.packages.get_pkg_dir(pkg)
        d = os.path.join(pkg_dir, type_)
        if os.path.isdir(d):
            files = os.listdir(d)
            if filter(lambda x: x.endswith('.'+type_), files):
                m_file = roslib.manifest.manifest_file(pkg, True)
                m = roslib.manifest.parse_file(m_file)
                cflags = m.get_export('cpp', 'cflags')
                include = '-I${prefix}/%s/cpp'%type_
                if not filter(lambda x: include in x, cflags):
                    missing.append(pkg)
    return missing
    
def manifest_msg_export(ctx):
    return _manifest_msg_srv_export(ctx, 'msg')
def manifest_srv_export(ctx):
    return _manifest_msg_srv_export(ctx, 'srv')
    
#CMake missing genmsg/gensrv
def _cmake_genmsg_gensrv(ctx, type_):
    missing = []
    cmds = ['rosbuild_gen%s()'%type_, 'gen%s()'%type_]
    for pkg in ctx.pkgs:
        pkg_dir = roslib.packages.get_pkg_dir(pkg)
        d = os.path.join(pkg_dir, type_)
        if os.path.isdir(d):
            files = os.listdir(d)
            if filter(lambda x: x.endswith('.'+type_), files):
                c_file = os.path.join(pkg_dir, 'CMakeLists.txt')
                f = open(c_file, 'r')
                try:
                    for l in f:
                        # ignore all whitespace
                        l = l.strip().replace(' ', '')
                        found_cmd = False
                        for cmd in cmds:
                            if l.startswith(cmd):
                                found_cmd = True
                        if found_cmd:
                            break
                    else:
                        missing.append(pkg)
                finally:
                    f.close()
    return missing

def cmake_genmsg(ctx):
    return _cmake_genmsg_gensrv(ctx, 'msg')
def cmake_gensrv(ctx):
    return _cmake_genmsg_gensrv(ctx, 'srv')    

#TODO: not sure if we should have this rule or not as it _does_ fail on ros-pkg
def makefile_exists(ctx):
    missing = []
    for pkg in ctx.pkgs:
        pkg_dir = roslib.packages.get_pkg_dir(pkg)
        p = os.path.join(pkg_dir, 'Makefile')
        if not os.path.isfile(p):
            missing.append(pkg)
    return missing

def rospack_time(ctx):
    start = time.time()
    roslib.scriptutil.rospackexec(['deps', 'roslib'])
    # arbitrarily tuned 
    return (time.time() - start) > 0.5

def cmakelists_package_valid(ctx):
    missing = []
    for pkg in ctx.pkgs:
        found = False
        pkg_dir = roslib.packages.get_pkg_dir(pkg)
        p = os.path.join(pkg_dir, 'CMakeLists.txt')
        if not os.path.isfile(p):
            continue #covered by cmakelists_exists
        f = open(p)
        try:
            for l in f:
                # ignore all whitespace
                l = l.strip().replace(' ', '')
                
                if l.startswith('rospack('):
                    found = True
                    if not l.startswith('rospack(%s)'%pkg):
                        missing.append(pkg)
                        break
                    # there may be more than 1 rospack() declaration, so scan through entire
                    # CMakeLists
                elif l.startswith("rosbuild_init()"):
                    found = True
            if not found:
                missing.append(pkg)
        finally:
            f.close()
    # rospack exists outside our build system
    if 'rospack' in missing:
        missing.remove('rospack')
    return missing

packages_warnings = [
    # disabling as it is too common and regular
    #(makefile_exists,
    # "The following packages have no Makefile:"),
    (cmakelists_package_valid,
     "The following packages have incorrect rospack() declarations in CMakeLists.txt:"),
    (rospack_time,
     "rospack is running very slowly. Consider running 'rospack profile' to find slow areas of your code tree."),
    (manifest_msg_export,
     'The following packages are missing\n<cpp cflags="-I${prefix}/msg/cpp"/> in manifest.xml:'),
    (manifest_srv_export,
     'The following packages are missing\n<cpp cflags="-I${prefix}/srv/cpp"/> in manifest.xml:'),
    (cmake_genmsg,
     'The following packages need genmsg() in CMakeLists.txt:'),
    (cmake_gensrv,     
     'The following packages need gensrv() in CMakeLists.txt:'),
    ]
packages_errors = [
    (msgs_built, "Messages have not been built in the following package(s).\nYou can fix this by typing 'rosmake %(pkg)s':"),
    (srvs_built, "Services have not been built in the following package(s).\nYou can fix this by typing 'rosmake %(pkg)s':"),
    ]

def wtf_check_packages(ctx):
    # no package in context to verify
    if not ctx.pkg:
        return
    
    for r in packages_warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in packages_errors:
        error_rule(r, r[0](ctx), ctx)

