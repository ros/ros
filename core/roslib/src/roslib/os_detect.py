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

"""
Library for detecting the current OS, including detecting specific
Linux distributions. 

The APIs of this library are still very coupled with the rosdep 
command-line tool.
"""

from __future__ import with_statement

import roslib.exceptions
import roslib.rospack
import roslib.stacks
import os
import sys
import subprocess
import types
import tempfile
import yaml

####### Linux Helper Functions #####
def lsb_get_os():
    """
    Linux: wrapper around lsb_release to get the current OS
    """
    try:
        cmd = ['lsb_release', '-si']
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        return std_out.strip()
    except:
        return None
    
def lsb_get_codename():
    """
    Linux: wrapper around lsb_release to get the current OS codename
    """
    try:
        cmd = ['lsb_release', '-sc']
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        return std_out.strip()
    except:
        return None
    
def lsb_get_version():
    """
    Linux: wrapper around lsb_release to get the current OS version
    """
    try:
        cmd = ['lsb_release', '-sr']
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        return std_out.strip()
    except:
        return None

class OSDetectException(roslib.exceptions.ROSLibException): pass

class OSBase:
    def check_presence(self):
        raise OSDetectException("check_presence unimplemented")

    def get_name(self):
        raise OSDetectException("check_presence unimplemented")

    def get_version(self):
        raise OSDetectException("check_presence unimplemented")

###### Debian SPECIALIZATION #########################
class Debian(OSBase):
    """
    Detect Debian OS.
    """
    def check_presence(self):
        if "Debian" == lsb_get_os():
            return True
        return False

    def get_version(self):
        return lsb_get_codename()
    def get_name(self):
        return "debian"

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
        return lsb_get_version()
    def get_name(self):
        return "ubuntu"

###### END UBUNTU SPECIALIZATION ########################

###### Mint SPECIALIZATION #########################
class Mint(Debian):
    """
    Detect Mint variants of Debian.
    """
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

    #get_version inherited from debian

    def get_name(self):
        return "mint"
###### END Mint SPECIALIZATION ########################

###### Fedora SPECIALIZATION #########################
class Fedora(OSBase):
    """
    Detect Fedora OS.
    """
    def check_presence(self):
        try:
            filename = "/etc/redhat-release"
            if os.path.exists(filename):
                with open(filename, 'r') as fh:                
                    os_list = fh.read().split()
                if os_list and os_list[0] == "Fedora" and os_list[1] == "release":
                    return True
        except:
            pass
        return False

    def get_version(self):
        try:
            filename = "/etc/issue"
            if os.path.exists(filename):
                with open(filename, 'r') as fh:
                    os_list = fh.read().split()
                if os_list[0] == "Fedora" and os_list[1] == "release":
                    return os_list[2]
        except:
            print "Fedora failed to get version"
            return False

        return False

    def get_name(self):
        return "fedora"

###### END Fedora SPECIALIZATION ########################

###### Rhel SPECIALIZATION #########################
class Rhel(Fedora):
    """
    Detect Redhat OS.
    """
    def check_presence(self):
        try:
            filename = "/etc/redhat-release"
            if os.path.exists(filename):
                with open(filename, 'r') as fh:                
                    os_list = fh.read().split()
                if os_list and os_list[2] == "Enterprise":
                    return True
        except:
            pass
        return False

    def get_version(self):
        try:
            filename = "/etc/issue"
            if os.path.exists(filename):
                with open(filename, 'r') as fh:
                    os_list = fh.read().split()
                if os_list and os_list[2] == "Enterprise":
                    return os_list[6]
        except:
            print "Rhel failed to get version"
            return False

        return False

    def get_name(self):
        return "rhel"

###### END Rhel SPECIALIZATION ########################

###### Macports SPECIALIZATION #########################
def port_detect(p):
    """
    Detect presence of Macports by running "port installed" command.
    """
    cmd = ['port', 'installed', p]
    pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (std_out, std_err) = pop.communicate()
    
    return (std_out.count("(active)") > 0)

class Macports(OSBase):
    """
    Detect OS X and Macports.
    """
    def check_presence(self):
        filename = "/usr/bin/sw_vers"
        if os.path.exists(filename):
            return True
        return False
    
    def get_version(self):
        return "macports" # macports is a rolling release and isn't versionsed

    def get_name(self):
        return "macports"

###### END Macports SPECIALIZATION ########################

###### Arch SPECIALIZATION #########################
class Arch(OSBase):
    """
    Detect Arch Linux.
    """

    def check_presence(self):
        filename = "/etc/arch-release"
        if os.path.exists(filename):
            return True
        return False

    def get_version(self):
        return ""
        # arch didn't have a version parsing in cpp version
        try:
            filename = "/etc/issue"
            if os.path.exists(filename):
                with open(filename, 'r') as fh:
                    os_list = fh.read().split()
                if os_list[0] == "Linux" and os_list[1] == "Arch":
                    return os_list[2]
        except:
            print "Arch failed to get version"
            return False

        return False

    def get_name(self):
        return "arch"

###### END Arch SPECIALIZATION ########################


###### Cygwin SPECIALIZATION #########################
class Cygwin(OSBase):
    """
    Detect Cygwin presence on Windows OS.
    """
    def check_presence(self):
        filename = "/usr/bin/cygwin1.dll"
        if os.path.exists(filename):
            return True
        return False
    
    def get_version(self):
        cmd = ['uname','-r'];
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        return std_out.strip()

    def get_name(self):
        return "cygwin"

###### END Cygwin SPECIALIZATION ########################

###### Gentoo Sepcialization ###############################
class Gentoo(OSBase):
    """
    Detect Gentoo OS.
    """
    def check_presence(self):
        try:
            filename = "/etc/gentoo-release"
            if os.path.exists(filename):
                with open(filename, 'r') as fh:                
                    os_list = fh.read().split()
                if os_list and os_list[0] == "Gentoo" and os_list[1] == "Base":
                    return True
        except:
            pass#print >> sys.stderr, "Gentoo failed to detect OS"
        return False

    def get_version(self):
        try:
            filename = "/etc/gentoo-release"
            if os.path.exists(filename):
                with open(filename, 'r') as fh:
                    os_list = fh.read().split()
                if os_list[0] == "Gentoo" and os_list[1] == "Base":
                    return os_list[4]
        except:
            print >> sys.stderr, "Gentoo failed to get version"
            return False

        return False

    def get_name(self):
        return "gentoo"

###### END Gentoo Sepcialization ###############################


#### Override class for debugging and unsupported OSs ###########
class Override(OSBase):
    def __init__(self):
        self._os_name = "uninitialized from ROS_OS_OVERRIDE=name:version"
        self._os_version = "uninitialized from ROS_OS_OVERRIDE=name:version"
        
    def check_presence(self):
        try:
            (self._os_name, self._os_version) = os.environ["ROS_OS_OVERRIDE"].split(':')
            print >> sys.stderr, "Using environment variable ROS_OS_OVERRIDE=name:version"
            return True
        except:
            return False
    
    def get_version(self):
        return self._os_version

    def get_name(self):
        return self._os_name
    




class OSDetect:
    """ This class will iterate over registered classes to lookup the
    active OS and version"""
    def __init__(self, os_list = [Debian(), Ubuntu(), Mint(), Macports(), Arch(), Fedora(), Rhel(), Gentoo(), Cygwin()]):
        self._os_list = [ Override()]
        self._os_list.extend(os_list)

        self._os_class = None
        self._os_name = None
        self._os_version = None

        self.detect_os()

    def add_os(self, class_ref):
        self._os_list.append(class_ref)

        # \TODO look at throwing here
    def detect_os(self):
        for os_class in self._os_list:
            if os_class.check_presence():
                self._os_name = os_class.get_name()
                self._os_version = os_class.get_version()
                self._os_class = os_class
                return True
        attempted_oss = [o.get_name() for o in self._os_list]
        raise OSDetectException("Could not detect OS, tried %s"%attempted_oss)
        return False

    def get_os(self):
        if not self._os_class:
            self.detect_os()
        return self._os_class

    def get_name(self):
        if not self._os_name:
            self.detect_os()
        return self._os_name

    def get_version(self):
        if not self._os_version:
            not self.detect_os()
        return self._os_version

