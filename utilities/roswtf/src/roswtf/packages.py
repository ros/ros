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
import roslib.srvs

## look for unknown tags in manifest
def manifest_valid(ctx):
    errors = []
    if ctx.manifest is not None:
        errors = ["<%s>"%t.tagName for t in ctx.manifest.unknown_tags]
    return errors
    
def _manifest_msg_srv_export(ctx, type_):
    exist = []
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
                if filter(lambda x: include in x, cflags):
                    exist.append(pkg)
    return exist
    
def manifest_msg_srv_export(ctx):
    msgs = set(_manifest_msg_srv_export(ctx, 'msg'))
    srvs = set(_manifest_msg_srv_export(ctx, 'srv'))
    errors = []

    for pkg in msgs & srvs:
        errors.append('%s: -I${prefix}/msg/cpp -I${prefix}/srv/cpp'%pkg)
    for pkg in msgs - srvs:
        errors.append('%s: -I${prefix}/msg/cpp'%pkg)
    for pkg in srvs - msgs:
        errors.append('%s: -I${prefix}/srv/cpp'%pkg)
    return errors
        

def _check_for_rpath_flags(pkg, lflags):
    if not lflags:
        return
    L_arg = '-L'
    Wl_arg = '-Wl'
    rpath_arg = '-rpath'
    lflags_args = lflags.split()
    # Collect the args we care about
    L_args = []
    rpath_args = []
    i = 0
    while i < len(lflags_args):
        f = lflags_args[i]
        if f.startswith(L_arg) and len(f) > len(L_arg):
            # normpath avoids problems with trailing slash vs. no trailing
            # slash, #2284
            L_args.append(os.path.normpath(f[len(L_arg):]))
        elif f == L_arg and (i+1) < len(lflags_args):
            i += 1
            # normpath avoids problems with trailing slash vs. no trailing
            # slash, #2284
            L_args.append(os.path.normpath(lflags_args[i]))
        elif f.startswith(Wl_arg) and len(f) > len(Wl_arg):
            # -Wl can be followed by multiple, comma-separated arguments,
            # #2284.
            args = f.split(',')
            j = 1
            while j < (len(args) - 1):
                if args[j] == rpath_arg:
                   # normpath avoids problems with trailing slash vs. no trailing
                   # slash, #2284
                    rpath_args.append(os.path.normpath(args[j+1]))
                    j += 2
                else:
                    j += 1
        i += 1
    # Check for parallelism; not efficient, but these strings are short
    for f in L_args:
        if f not in rpath_args:
            return '%s: found flag "-L%s", but no matching "-Wl,-rpath,%s"'%(pkg, f,f)
    for f in rpath_args:
        if f not in L_args:
            return '%s: found flag "-Wl,-rpath,%s", but no matching "-L%s"'%(pkg, f,f)

def manifest_rpath_flags(ctx):
    warn = []
    for pkg in ctx.pkgs:
        # Use rospack to get lflags, so that they can be bash-expanded
        # first, #2286.
        import subprocess
        lflags = subprocess.Popen(['rospack', 'export', '--lang=cpp', '--attrib=lflags', pkg], stdout=subprocess.PIPE).communicate()[0]
        err_msg = _check_for_rpath_flags(pkg, lflags)
        if err_msg:
            warn.append(err_msg)
    return warn

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
        finally:
            f.close()
    # rospack exists outside our build system
    if 'rospack' in missing:
        missing.remove('rospack')
    return missing

warnings = [
    # disabling as it is too common and regular
    (cmakelists_package_valid,
     "The following packages have incorrect rospack() declarations in CMakeLists.txt.\nPlease switch to using rosbuild_init():"),
    
    (manifest_msg_srv_export,
     'The following packages have msg/srv-related cflags exports that are no longer necessary\n\t<export>\n\t\t<cpp cflags="..."\n\t</export>:'),
    (manifest_valid, "%(pkg)s/manifest.xml has unrecognized tags:"),

    ]
errors = [
    (manifest_rpath_flags, "The following packages have rpath issues in manifest.xml:"),
    ]

def wtf_check(ctx):
    # no package in context to verify
    if not ctx.pkgs:
        return
    
    for r in warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in errors:
        error_rule(r, r[0](ctx), ctx)

