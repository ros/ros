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

import subprocess
import os
import roslib.os_detect

import rosdep.base_rosdep

###### DEBIAN SPECIALIZATION #########################

###### Rosdep Test OS #########################
class RosdepTestOS(rosdep.base_rosdep.RosdepBaseOS):
    def __init__(self):
        self.name = "uninitialized"
    def check_presence(self):
        if "ROSDEP_TEST_OS" in os.environ:
            return True
        return False

    def get_name(self):
        return os.environ.get("ROSDEP_TEST_OS", "rosdep_test_os")

    def get_version(self):
        return os.environ.get("ROSDEP_TEST_VERSION", "rosdep_test_version")
    
    def strip_detected_packages(self, packages):
        return packages

    def generate_package_install_command(self, packages, default_yes):
        if default_yes:
            return "#yes"
        else:
            return "#no"


class AptGetInstall():

    def dpkg_detect(self, pkgs):
        ret_list = []
        cmd = ['dpkg-query', '-W', '-f=\'${Package} ${Status}\n\'']
        cmd.extend(pkgs)
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        std_out = std_out.replace('\'','')
        pkg_list = std_out.split('\n')
        for pkg in pkg_list:
            pkg_row = pkg.split()
            if len(pkg_row) == 4 and (pkg_row[3] =='installed'):
                ret_list.append( pkg_row[0])
        return ret_list

    def strip_detected_packages(self, packages):
        return list(set(packages) - set(self.dpkg_detect(packages)))

    def generate_package_install_command(self, packages, default_yes):
        if not packages:
            return "#No Packages to install"
        if default_yes:
            return "#Packages\nsudo apt-get install -y " + ' '.join(packages)        
        else:
            return "#Packages\nsudo apt-get install " + ' '.join(packages)

###### Debian SPECIALIZATION #########################
class Debian(roslib.os_detect.Debian, AptGetInstall, rosdep.base_rosdep.RosdepBaseOS):
    """ This is an implementation of a standard interface for
    interacting with rosdep.  This defines all Ubuntu sepecific
    methods, including detecting the OS/Version number.  As well as
    how to check for and install packages."""
    pass
###### END Debian SPECIALIZATION ########################

###### UBUNTU SPECIALIZATION #########################
class Ubuntu(roslib.os_detect.Ubuntu, AptGetInstall, rosdep.base_rosdep.RosdepBaseOS):
    """ This is an implementation of a standard interface for
    interacting with rosdep.  This defines all Ubuntu sepecific
    methods, including detecting the OS/Version number.  As well as
    how to check for and install packages."""
    pass

###### END UBUNTU SPECIALIZATION ########################

###### Mint SPECIALIZATION #########################
class Mint(AptGetInstall, rosdep.base_rosdep.RosdepBaseOS):
    """ This is an implementation of a standard interface for
    interacting with rosdep.  Mint is closely coupled to Ubuntu, it
    will masquerade as ubuntu for the purposes of rosdep. """
    
    def __init__(self):
        self.mint_detector = roslib.os_detect.Mint()
        self.version_map = {'9':'10.04',
                            '8':'9.10', 
                            '7':'9.04',
                            '6':'8.10',
                            '5':'8.04'}
    def get_version(self):
        return self.version_map[self.mint_detector.get_version()]

    def get_name(self):
        return 'ubuntu'

    def check_presence(self):
        return self.mint_detector.check_presence()
        

    pass

###### END Mint SPECIALIZATION ########################
