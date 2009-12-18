#! /usr/bin/env python

# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# Author Tully Foote/tfoote@willowgarage.com

from __future__ import with_statement

import os
import sys, string
import subprocess
import roslib.rospack
import roslib.rosenv
import roslib.stacks

def can_build(pkg, use_whitelist = False, use_blacklist = False, os_name = None, os_version = None):
    """
    Return (buildable, "reason why not")
    """
    output_str = ""
    output_state = True

    if use_whitelist:
        pass

    if use_blacklist:
        blacklist_packages = roslib.rospack.rospack_depends_on(pkg)
        blacklist_packages.append(pkg)
        for p in blacklist_packages:
            blacklist_path = os.path.join(get_pkg_dir(p), "ROS_BUILD_BLACKLIST")
            if os.path.exists(blacklist_path):
                output_state = False
                output_str += "ROS_BUILD_BLACKLIST found in %s"%p
                with open(blacklist_path) as f:
                    output_str += f.read() + "\n"

    if os.path.exists(os.path.join(get_pkg_dir(pkg), "ROS_NOBUILD")):
        output_state = False
        output_str += "ROS_NOBUILD in package\n"


    if not os.path.exists(os.path.join(get_pkg_dir(pkg), "Makefile")):
        output_state = False
        output_str += " No Makefile in package\n"

    

    return (output_state, output_str)
