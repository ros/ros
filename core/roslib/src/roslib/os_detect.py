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

import roslib.exceptions
import os
import sys
import subprocess
import types
import tempfile
import distutils.version # To parse version numbers

if sys.hexversion > 0x03000000: #Python3
    python3 = True
else:
    python3 = False

####### Linux Helper Functions #####
def _read_stdout(cmd):
    pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (std_out, std_err) = pop.communicate()
    if python3:
        return std_out.decode()
    else:
        return std_out
    
def lsb_get_os():
    """
    Linux: wrapper around lsb_release to get the current OS
    """
    try:
        return _read_stdout(['lsb_release', '-si']).strip()
    except:
        return None
    
def lsb_get_codename():
    """
    Linux: wrapper around lsb_release to get the current OS codename
    """
    try:
        return _read_stdout(['lsb_release', '-sc']).strip()
    except:
        return None
    
def lsb_get_version():
    """
    Linux: wrapper around lsb_release to get the current OS version
    """
    try:
        return _read_stdout(['lsb_release', '-sr']).strip()
    except:
        return None

def uname_get_machine():
    """
    Linux: wrapper around uname to determine if OS is 64-bit
    """
    try:
        return _read_stdout(['uname', '-m']).strip()
    except:
        return None



#### Override class for debugging and unsupported OSs ###########
class OSOverride:
    def __init__(self):
        self._os_name = "uninitialized from ROS_OS_OVERRIDE=name:version"
        self._os_version = "uninitialized from ROS_OS_OVERRIDE=name:version"
        
    def check_presence(self):
        try:
            (self._os_name, self._os_version) = os.environ["ROS_OS_OVERRIDE"].split(':')
            sys.stderr.write("Using environment variable ROS_OS_OVERRIDE name = %s version = %s\n"%(self._os_name, self._os_version))
            return True
        except:
            return False
    
    def get_version(self):
        return self._os_version

    def get_name(self):
        return self._os_name


class OSDetectException(roslib.exceptions.ROSLibException): pass

class OSBase:
    """
    This defines the API used for OS detection within the os_detect
    module for roslib.  All OS specific instantiantions must inherit
    and override these methods.
    """
    def check_presence(self):
        """
        Return if the specific OS which this class is designed to
        detect is present.  Only one version of this class should return for
        any version.  
        """
        raise OSDetectException("check_presence unimplemented")

    def get_name(self):
        """
        Return the standardized name for this OS.  (ala Ubuntu Hardy Heron = "ubuntu")
        """
        raise OSDetectException("get_name unimplemented")

    def get_version(self):
        """
        Return the standardized version for this OS. (ala Ubuntu Hardy Heron = "8.04")
        """
        raise OSDetectException("get_version unimplemented")


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

###### Mandriva SPECIALIZATION #########################
class Mandriva(OSBase):
    """
    Detect Mandriva OS. The returned version will be the year release (e.g.
    2010.0) concatenated with the machine architecture (e.g. x86_64), resulting
    in something like 2010.0x86_64.
    """
    def check_presence(self):
        if "MandrivaLinux" == lsb_get_os():
            return True
        return False
    
    def get_version(self):
        return lsb_get_version()+uname_get_machine()
    def get_name(self):
        return "mandriva"

###### END Mandriva SPECIALIZATION ########################



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
        return lsb_get_codename()
    def get_name(self):
        return "ubuntu"

###### END UBUNTU SPECIALIZATION ########################

###### Mint SPECIALIZATION #########################
class Mint(OSBase):
    """
    Detect Mint variants of Debian.
    """
    def check_presence(self):
        if "LinuxMint" == lsb_get_os():
            return True
        return False

    def get_version(self):
        return lsb_get_version()

    def get_name(self):
        return "mint"
###### END Mint SPECIALIZATION ########################

###### OpenSuse SPECIALIZATION #########################
class OpenSuse(OSBase):
    """
    Detect OpenSuse OS.
    """
    def check_presence(self):
        try:
            filename = "/etc/SuSE-brand"
            if os.path.exists(filename):
                with open(filename, 'r') as fh:
                    os_list = fh.read().split()
                    if len(os_list) > 0 and os_list[0] == "openSUSE":
                        return True
        except:
            pass
        return False

    def get_version(self):
        try:
            filename = "/etc/SuSE-brand"
            if os.path.exists(filename):
                with open(filename, 'r') as fh:
                    os_list = fh.read().strip().split('\n')
                    if len(os_list) == 2:
                        os_list = os_list[1].split(' = ')
                        if os_list[0] == "VERSION":
                            return os_list[1]
        except:
            return False
        
        return False

    def get_name(self):
        return "opensuse"

###### END OpenSuse SPECIALIZATION ########################


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
            sys.stderr.write("Fedora failed to get version\n")
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
            sys.stderr.write("Rhel failed to get version\n")
            return False

        return False

    def get_name(self):
        return "rhel"

###### END Rhel SPECIALIZATION ########################

###### OSX SPECIALIZATION #########################
def port_detect(p):
    """
    Detect presence of Macports by running "port installed" command.
    """
    std_out = _read_stdout(['port', 'installed', p])
    return (std_out.count("(active)") > 0)

class Osx(OSBase):
    """
    Detect OS X 
    """
    def check_presence(self):
        filename = "/usr/bin/sw_vers"
        if os.path.exists(filename):
            return True
        return False
    
    def get_version(self):
        # REP 111 this should be the code name (e.g., lion, snow, tiger) #3570
        std_out = _read_stdout(['/usr/bin/sw_vers','-productVersion'])
        ver = distutils.version.StrictVersion(std_out).version
        if len(ver) < 2:
            raise OSDetectException("invalid version string: %s"%(std_out))
        major, minor = ver[0:2]
        # Source: http://en.wikipedia.org/wiki/Mac_OS_X#Versions
        if major == 10 and minor == 4:
            return 'tiger'
        elif major == 10 and minor == 5:
            return 'leopard'
        elif major == 10 and minor == 6:
            return 'snow'
        elif major == 10 and minor == 7:
            return 'lion'
        else:
            raise OSDetectException("unrecognized version: %s"%(std_out))

    def get_name(self):
        return "osx"

###### END OSX SPECIALIZATION ########################

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
        return _read_stdout(['uname','-r']).strip()

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
            pass
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
            sys.stderr.write("Gentoo failed to get version\n")
            return False

        return False

    def get_name(self):
        return "gentoo"

###### END Gentoo Sepcialization ###############################

###### FreeBSD SPECIALIZATION #########################
class FreeBSD(OSBase):
    """
    Detect FreeBSD OS.
    """
    def check_presence(self):
        try:
            filename = "/usr/bin/uname"
            if os.path.exists(filename):
                std_out = _read_stdout([filename])
                if std_out.strip() == "FreeBSD":
                    return True
            else:
                return False
        except:
            pass#print >> sys.stderr, "FreeBSD failed to detect OS"
        return False

    def get_version(self):
        try:
            filename = "/usr/bin/uname"
            if os.path.exists(filename):
                return _read_stdout([filename, "-r"]).strip()
            else:
                return False
        except:
            sys.stderr.write("FreeBSD failed to get version\n")
            return False

        return False

    def get_name(self):
        return "freebsd"

###### FreeBSD SPECIALIZATION #########################

    




class OSDetect:
    """ This class will iterate over registered classes to lookup the
    active OS and version"""
    def __init__(self, os_list = [Debian(), Mandriva(), Ubuntu(), Mint(), Osx(), Arch(), OpenSuse(), Fedora(), Rhel(), Gentoo(), Cygwin(), FreeBSD()]):
        self._os_list = os_list
        for o in self._os_list:
            if not isinstance(o, OSBase):
                raise OSDetectException("Class [%s] not derived from OSBase"%o.__class__.__name__)

        self._os_class = None
        self._os_name = None
        self._os_version = None

        self.detect_os()

    def add_os(self, class_ref):
        self._os_list.append(class_ref)

        # \TODO look at throwing here
    def detect_os(self):
        override = OSOverride()
        if override.check_presence():
            for os_class in self._os_list:
                if os_class.get_name() == override.get_name():
                    self._os_name = override.get_name()
                    self._os_version = override.get_version()
                    self._os_class = os_class
                    return True

        for os_class in self._os_list:
            if os_class.check_presence():
                self._os_name = os_class.get_name()
                self._os_version = os_class.get_version()
                self._os_class = os_class
                return True

        # No solution found
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

