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

# Author Tully Foote tfoote@willowgarage.com

import time

from roswtf.rules import warning_rule, error_rule
import roslib.manifest
import rospy
import subprocess

################################################################################
# RULES


def detect_zombies(ctx):
    (std_out, std_err) = subprocess.Popen(["rospack", "profile", "--zombie-only"], stdout=subprocess.PIPE).communicate()
    return std_out.split()

def stack_dependencies_present(ctx):
    print "Checking stack dependencies"
    stack_list = roslib.stacks.list_stacks()
    missing = [s for s in stack_list if set(stack_list).issubset(roslib.rospack.rosstack_depends_1(s))]
    print "Done checking stack dependencies"
    return missing

def package_dependencies_present(ctx):
    print "Checking package dependencies"
    package_list = roslib.packages.list_pkgs()
    missing = [s for s in package_list if set(package_list).issubset(roslib.rospack.rospack_depends_1(s))]
    print "Done checking package dependencies"
    return missing

def ros_root_invalid(ctx):
    rr = roslib.rosenv.get_ros_root()
    last_element = rr.split('/')[-1]
    if last_element != "ros":
        return ["ROS_ROOT must end in 'ros' it currently ends in %s"%last_element]
    else:
        return None


################################################################################
# roswtf PLUGIN

# rospack_warnings and rospack_errors declare the rules that we actually check
                
rospack_warnings = [
  (detect_zombies,"Zombied directories detected"),
]
rospack_errors = [
  (stack_dependencies_present,"Not all stack dependencies are present for stack"),
  (package_dependencies_present,"Not all package dependencies are present for package"),
  (ros_root_invalid,"ROS_ROOT invalid"),
]


# roswtf entry point for online checks
def roswtf_plugin_online(ctx):
    return # no dynamic checks
    # don't run plugin if tf isn't active as these checks take awhile
    #if not is_tf_active() and not is_tf_message_active():
    #    return
    

    for r in rospack_warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in rospack_errors:
        error_rule(r, r[0](ctx), ctx)

    
# currently no static checks for tf
def roswtf_plugin_static(ctx):
    for r in rospack_warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in rospack_errors:
        error_rule(r, r[0](ctx), ctx)
        
