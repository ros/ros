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

import subprocess
import os.path 
import roslib.os_detect
import os 
import shutil
import urllib
import urllib2
import tarfile
import tempfile
import yaml

import rosdep.base_rosdep
import rosdep.core


class InstallerAPI():
    def __init__(self, arg_dict):
        """
        Set all required fields here 
        """
        raise NotImplementedError, "Base class __init__"
    
    def check_presence(self):
        """
        This script will return true if the rosdep is found on the
        system, otherwise false.
        """
        raise NotImplementedError, "Base class check_presence"

    def generate_package_install_command(self, default_yes, execute = True, display = True):
        """
        If execute is True, install the rosdep, else if display = True
        print the script it would have run to install.
        @param default_yes  Pass through -y or equivilant to package manager
        """
        raise NotImplementedError, "Base class generate_package_install_command"

    def get_depends(self): 
        """ 
        Return the dependencies, only necessary if the package manager
        doesn't handle the dependencies.
        """
        return [] # Default return empty list

class SourceInstaller(InstallerAPI):
    def __init__(self, arg_dict):
        self.url = arg_dict.get("uri")
        if not self.url:
            raise rosdep.core.RosdepException("uri required for source rosdeps") 


        #TODO add md5sum verification
        if "ROSDEP_DEBUG" in os.environ:
            print "Downloading manifest %s"%self.url
        try:
            self.manifest = yaml.load(urllib2.urlopen(self.url))
        except urllib2.URLError, ex:
            raise rosdep.core.RosdepException("Failed to load url %s with error: %s"%(self.url, ex))
        except yaml.scanner.ScannerError, ex:
            raise rosdep.core.RosdepException("Failed to parse yaml in %s:  Error: %s"%(self.url, ex))
            
        if "ROSDEP_DEBUG" in os.environ:
            print "Downloaded manifest:\n{{{%s\n}}}\n"%self.manifest
        
        self.install_command = self.manifest.get("install-script", "#!/bin/bash\n#no install-script specificd")
        self.check_presence_command = self.manifest.get("check-presence-script", "#!/bin/bash\n#no check-presence-script\nfalse")

        self.exec_path = self.manifest.get("exec-path", ".")

        self.depends = self.manifest.get("depends", [])

        self.tarball = self.manifest.get("uri")
        if not self.tarball:
            raise rosdep.core.RosdepException("uri required for source rosdeps") 


    def check_presence(self):

        return rosdep.core.create_tempfile_from_string_and_execute(self.check_presence_command)

    def generate_package_install_command(self, default_yes = False, execute = True):
        tempdir = tempfile.mkdtemp()
        success = False

        if "ROSDEP_DEBUG" in os.environ:
            print "Fetching %s"%self.tarball
        f = urllib.urlretrieve(self.tarball)

        try:
            tarf = tarfile.open(f[0])
            tarf.extractall(tempdir)

            if execute:
                if "ROSDEP_DEBUG" in os.environ:
                    print "Running installation script"
                success = rosdep.core.create_tempfile_from_string_and_execute(self.install_command, os.path.join(tempdir, self.exec_path))
            else:
                print "Would have executed\n{{{%s\n}}}"%self.install_command
            
        finally:
            shutil.rmtree(tempdir)
            os.remove(f[0])

        if success:
            if "ROSDEP_DEBUG" in os.environ:
                print "successfully executed script"
            return True
        return False

    def get_depends(self): 
        #todo verify type before returning
        return self.depends
        

        

class AptInstaller(InstallerAPI):
    """ 
    An implementation of the InstallerAPI for use on debian style
    systems.
    """
    def __init__(self, arg_dict):

        
        packages = arg_dict.get("packages", "")
        if type(packages) == type("string"):
            packages = packages.split()


        self.packages_to_install = list(set(packages) - set(self.dpkg_detect(packages)))


    def check_presence(self):
        return len(self.packages_to_install) == 0


    def generate_package_install_command(self, default_yes = False, execute = True, display = True):
        script = '!#/bin/bash\n#no script'
        if not self.packages_to_install:
            script =  "#!/bin/bash\n#No Packages to install"
        if default_yes:
            script = "#!/bin/bash\n#Packages %s\nsudo apt-get install -y "%self.packages_to_install + ' '.join(self.packages_to_install)        
        else:
            script =  "#!/bin/bash\n#Packages %s\nsudo apt-get install "%self.packages_to_install + ' '.join(self.packages_to_install)

        if execute:
            return rosdep.core.create_tempfile_from_string_and_execute(script)
        elif display:
            print "To install packages: %s would have executed script\n{{{\n%s\n}}}"%(self.packages_to_install, script)
        return False



    def dpkg_detect(self, pkgs):
        """ 
        Given a list of package, return the list of installed packages.
        """
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


        

class PipInstaller(InstallerAPI):
    """ 
    An implementation of the InstallerAPI for use on debian style
    systems.
    """
    def __init__(self, arg_dict):

        packages = arg_dict.get("packages", "")
        if type(packages) == type("string"):
            packages = packages.split()


        self.packages_to_install = list(set(packages) - set(self.pip_detect(packages)))
        self.depends = arg_dict.get("depends", [])


    def check_presence(self):
        return len(self.packages_to_install) == 0

    def get_depends(self):
        #todo verify type before returning
        return self.depends

    def generate_package_install_command(self, default_yes = False, execute = True, display = True):
        script = '!#/bin/bash\n#no script'
        if not self.packages_to_install:
            script =  "#!/bin/bash\n#No PIP Packages to install"
        #if default_yes:
        #    script = "#!/bin/bash\n#Packages %s\nsudo apt-get install -U "%self.packages_to_install + ' '.join(self.packages_to_install)        
        #else:
        script =  "#!/bin/bash\n#Packages %s\nsudo pip install -U "%self.packages_to_install + ' '.join(self.packages_to_install)

        if execute:
            return rosdep.core.create_tempfile_from_string_and_execute(script)
        elif display:
            print "To install packages: %s would have executed script\n{{{\n%s\n}}}"%(self.packages_to_install, script)
        return False



    def pip_detect(self, pkgs):
        """ 
        Given a list of package, return the list of installed packages.
        """
        ret_list = []
        cmd = ['pip', 'freeze']
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        pkg_list = std_out.split('\n')
        for pkg in pkg_list:
            pkg_row = pkg.split("==")
            print pkg_row
            if pkg_row[0] in pkgs:
                ret_list.append( pkg_row[0])
        return ret_list

