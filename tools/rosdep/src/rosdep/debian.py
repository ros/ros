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
from linux_helpers import *
import os.path 

###### DEBIAN SPECIALIZATION #########################
def dpkg_detect(p):
    cmd = ['dpkg-query', '-W', '-f=\'${Status}\'', p]
    pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (std_out, std_err) = pop.communicate()
    std_out = std_out.strip('\'')
    std_out_list = std_out.split()
    if len(std_out_list) == 3:
        return (std_out_list[2] =='installed')
    else:
        return False

###### Debian SPECIALIZATION #########################
class Debian:
    def check_presence(self):
        if "Debian" == lsb_get_os():
            return True
        return False

    def get_version(self):
        return lsb_get_release_codename()
    def get_name(self):
        return "debian"

    def strip_detected_packages(self, packages):
        return [p for p in packages if not dpkg_detect(p)]

    def generate_package_install_command(self, packages, default_yes):
        if not packages:
            return "#No Packages to build"
        if default_yes:
            return "#Packages\nsudo apt-get install -y " + ' '.join(packages)        
        else:
            return "#Packages\nsudo apt-get install " + ' '.join(packages)

###### END Debian SPECIALIZATION ########################


###### UBUNTU SPECIALIZATION #########################
class Ubuntu(Debian):
    """ This is an implementation of a standard interface for
    interacting with rosdep.  This defines all Ubuntu sepecific
    methods, including detecting the OS/Version number.  As well as
    how to check for and install packages."""
    def check_presence(self):
        if "Ubuntu" == lsb_get_os():
            return True
        return False

    def get_version(self):
        return lsb_get_release_version()
    def get_name(self):
        return "ubuntu"

###### END UBUNTU SPECIALIZATION ########################

###### Mint SPECIALIZATION #########################
class Mint:
    def check_presence(self):
        try:
            filename = "/etc/issue"
            if os.path.exists(filename):
                with open(filename, 'r') as fh:                
                    os_list = fh.read().split()
                if os_list and os_list[0] == "Linux" and os_list[1] == "Mint":
                    return True
        except:
            print "Mint failed to detect OS"
        return False

    def get_version(self):
        try:
            filename = "/etc/issue"
            if os.path.exists(filename):
                with open(filename, 'r') as fh:
                    os_list = fh.read().split()
                if os_list[0] == "Linux" and os_list[1] == "Mint":
                    return os_list[2]
        except:
            print "Mint failed to get version"
            return False

        return False

    def get_name(self):
        return "mint"

    def strip_detected_packages(self, packages):
        return [p for p in packages if not dpkg_detect(p)]

    def generate_package_install_command(self, packages, default_yes):        
        return "#Packages\nsudo apt-get install " + ' '.join(packages)

###### END Mint SPECIALIZATION ########################
