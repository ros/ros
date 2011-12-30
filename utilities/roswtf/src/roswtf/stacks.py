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
# Revision $Id: packages.py 6258 2009-09-22 22:08:20Z kwc $

import os
import time

from roswtf.environment import paths, is_executable
from roswtf.rules import warning_rule, error_rule

import rospkg

_packages_of_cache = {}
def _packages_of(rosstack, d):
    if d in _packages_of_cache:
        return _packages_of_cache[d]
    else:
        _packages_of_cache[d] = pkgs = rosstack.packages_of(d)
        return pkgs
                
def manifest_depends(ctx):
    # This rule should probably be cache optimized
    errors = []
    rospack = rospkg.RosPack()
    rosstack = rospkg.RosStack()

    stack_list = rosstack.list()
    #print stack_list
    for s in ctx.stacks:
        try:
            s_deps = []
            s_pkgs = _packages_of(rosstack, s)
            for p in s_pkgs:
                s_deps.extend(rospack.get_depends(p, implicit=False))
            m = rosstack.get_manifest(s)
            m_file = os.path.join(rosstack.get_path(s), 'stack.xml')
            for d in m.depends:
                if not d.name in stack_list:
                    errors.append("%s (%s does not exist)"%(m_file, d))
                elif d.name in ['ros', 'ros_comm']:
                    # ros dependency always exists. ros_comm
                    # dependency has implicit connections (msggen), so
                    # we ignore.
                    continue
                else:
                    pkgs = _packages_of(rosstack, d.name)
                    # check no longer works due to rosdeps chains
                    #if not [p for p in pkgs if p in s_deps]:
                    #    errors.append("%s (%s appears to be an unnecessary depend)"%(m_file, d))
        except rospkg.ResourceNotFound:
            # report with a different rule
            pass
    return errors

warnings = [
    (manifest_depends, "The following stack.xml file list invalid dependencies:"),
    
    ]
errors = [
    ]

def wtf_check(ctx):
    # no package in context to verify
    if not ctx.stacks:
        return
    
    for r in warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in errors:
        error_rule(r, r[0](ctx), ctx)

