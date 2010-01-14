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

# Author Tully Foote/tfoote@willowgarage.com

from __future__ import with_statement
import os.path
import roslib.os_detect
import subprocess

def rpm_detect(p):
    return subprocess.call(['rpm', '-q', p], stdout=subprocess.PIPE, stderr=subprocess.PIPE)    

class YumInstall:
    def strip_detected_packages(self, packages):
        return [p for p in packages if rpm_detect(p)]

    def generate_package_install_command(self, packages, default_yes):
        if not packages:
            return "#No Packages to install"

        if default_yes:
            return "#Packages\nsudo yum -y install " + ' '.join(packages)
        else:
            return "#Packages\nsudo yum install " + ' '.join(packages)


###### Fedora SPECIALIZATION #########################
class Fedora(roslib.os_detect.Fedora, YumInstall): pass
                 
###### END Fedora SPECIALIZATION ########################

###### Rhel SPECIALIZATION #########################
class Rhel(roslib.os_detect.Rhel, YumInstall): pass
###### END Rhel SPECIALIZATION ########################

