#!/usr/bin/env python
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
#     * Neither the name of the Willow Garage, Inc. nor the names of its
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

from __future__ import with_statement
import os.path
import roslib.os_detect
import subprocess

import rosdep.base_rosdep

# Determine whether package p needs to be installed
def equery_detect(p):
    cmd = ['equery', '-q', 'l', p]
    pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (std_out, std_err) = pop.communicate()
    
    return (std_out.count("") == 1)

# Check equery for existence and compatibility (gentoolkit 0.3)
def equery_available():
    if not os.path.exists("/usr/bin/equery"):
        return False

    cmd = ['equery', '-V']
    pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (stdout, stderr) = pop.communicate()

    return "0.3." == stdout[8:12]


###### Gentoo SPECIALIZATION #########################
class Gentoo(roslib.os_detect.Gentoo, rosdep.base_rosdep.RosdepBaseOS):
    def strip_detected_packages(self, packages):
        if equery_available():
            return [p for p in packages if equery_detect(p)]
        else:
            return packages

    def generate_package_install_command(self, packages, default_yes):
        if len(packages) == 0:
            return "# Package prerequisites satisfied - nothing to do"
        elif equery_available():
            return "#Packages\nsudo emerge " + ' '.join(packages)
        else:
	    return "#Packages\nsudo emerge -u " + ' '.join(packages)

###### END Gentoo SPECIALIZATION ########################
