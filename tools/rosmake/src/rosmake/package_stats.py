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

def can_build(pkg, use_whitelist = False, use_whitelist_recursive = False, use_blacklist = False, failed_packages = [], os_name = None, os_version = None):
    """
    Return (buildable, "reason why not")
    """


        
    output_str = ""
    output_state = True
    buildable = True
    if not os_name and not os_version:
        osd = roslib.os_detect.OSDetect()
        os_name = osd.get_name()
        os_version = osd.get_version()
        

    for p in failed_packages:
        if p in roslib.rospack.rospack_depends(pkg):
            buildable = False
            output_state = False
            output_str += " Package %s cannot be built for dependent package %s failed. \n"%(pkg, p)


    if use_whitelist:

        if not roslib.packages.platform_supported(pkg, os_name, os_version):
            buildable = False
            output_state = False
            output_str += " Package %s is not supported on this OS\n"%pkg
        if use_whitelist_recursive:
            for p in roslib.rospack.rospack_depends(pkg):
                if not roslib.packages.platform_supported(pkg, os_name, os_version):
                    output_state = False
                    output_str += " Package %s is not supported on this OS\n"%p
                
    if use_blacklist:
        blacklist_packages = roslib.rospack.rospack_depends(pkg)
        blacklist_packages.append(pkg)
        for p in blacklist_packages:
            blacklist_path = os.path.join(roslib.packages.get_pkg_dir(p), "ROS_BUILD_BLACKLIST")
            if os.path.exists(blacklist_path):
                buildable = False
                output_str += "ROS_BUILD_BLACKLIST found in %s contents are:\n[[[\n"%p
                with open(blacklist_path) as f:
                    output_str += f.read() + "\n]]]\n"

    if os.path.exists(os.path.join(roslib.packages.get_pkg_dir(pkg), "ROS_NOBUILD")):
        buildable = False
        output_state = True # dependents are ok, it should already be built
        output_str += "ROS_NOBUILD in package %s\n"%pkg


    if not os.path.exists(os.path.join(roslib.packages.get_pkg_dir(pkg), "Makefile")):
        output_state = True # dependents are ok no need to build
        buildable = False
        output_str += " No Makefile in package %s\n"%pkg

    

    return (buildable, output_state, output_str)
