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

import roslib.scriptutil
import os
import sys
import subprocess
import yaml

########## Class for interacting with rosdep.yaml files
class RosdepLookup:
    """This is a class for interacting with rosdep.yaml files.  It
    will load all rosdep.yaml files in the current configuration at
    startup.  It has accessors to allow lookups into the rosdep.yaml
    from rosdep name and returns the string from the yaml file for the
    appropriate OS/version.

    It uses the OSIndex class for OS detection.
"""
    def __init__(self, osindex):
        """ Read all rosdep.yaml files found at the root of stacks in
        the current environment and build them into a map."""
        self.os_index = osindex
        self.rosdep_map = {}
        self.rosdep_source = {}
        ## Find all rosdep.yamls here and load them into a map
        stacks = roslib.scriptutil.rosstackexec(["list-names"]).split()
        #print stacks
        for s in stacks:
            path = os.path.join(roslib.scriptutil.rosstackexec(["find", s]), "rosdep.yaml")
            if os.path.exists(path):
                try:
                    f = open(path)
                    yaml_text = f.read()
                    f.close()

                    yaml_dict = yaml.load(yaml_text)
                    for key in yaml_dict:
                        if key in self.rosdep_source.keys():
                            print "rosdep already loaded %s from %s.  But it is also defined in %s.  This will not be overwritten"%(key, self.rosdep_source[key], path)
                            #exit(-1)
                        else:
                            self.rosdep_source[key] = path
                            self.rosdep_map[key] = yaml_dict[key]

                except yaml.YAMLError, exc:
                    print "Failed parsing yaml while processing %s\n"%path, exc
                    sys.exit(1)
            
        #print "built map", self.rosdep_map

    def lookup_rosdep(self, rosdep):
        """ Lookup the OS specific packages or script from the
        prebuilt maps."""
        os_name = self.os_index.get_os_name()
        os_version = self.os_index.get_os_version()

        if rosdep in self.rosdep_map:
            individual_rosdep_map = self.rosdep_map[rosdep]
            # See if the version for this OS exists
            if os_name in individual_rosdep_map:
                os_specific = individual_rosdep_map[os_name]
                # See if there are different versions called out
                if type(os_specific) == type("String"):
                    return os_specific
                else:# it must be a map of versions
                    if os_version in os_specific.keys():
                        return os_specific[versions]
                    else:
                        ## Hack to match rounding errors in pyyaml load 9.04  != 9.03999999999999996 in string space
                        for key in os_specific.keys():
                            if float(key) == float(os_version):
                                #print "Matched %s"%(os_version)
                                return os_specific[key]

                        print "failed to find specific version %s of %s within"%(os_version, rosdep), os_specific
                        return False
                    
            else:
                print "failed to find OS(%s) version of %s "%(os_name, rosdep)
                return False

        else:
            return False
        
        
        


########## Class for interacting with customized OS detectors ############
class OSIndex:
    """ This class will iterate over registered classes to lookup the
    active OS and Version of that OS for lookup in rosdep.yaml"""
    def __init__(self):
        self._os_map = {}
        self._os_detected = False
        self._os_version = False

    def add_os(self, name, class_ref):
        self._os_map[name] = class_ref

        # \TODO look at throwing here
    def get_os_name(self):
        if not self._os_detected:
            for name in self._os_map.keys():
                if self._os_map[name].check_presence:
                    self._os_detected = name
                    return name
        if not self._os_detected:
            print "Failed to detect OS"
            sys.exit(1)
        return self._os_detected

    def get_os_version(self):
        return self._os_map[self.get_os_name()].get_version()

    def detect_packages(self, packages):
        return self._os_map[self.get_os_name()].detect_packages(packages)

    def build_script(self, packages):
        return self._os_map[self.get_os_name()].build_script(packages)


###### UBUNTU SPECIALIZATION #########################
class Ubuntu:
    """ This is an implementation of a standard interface for
    interacting with rosdep.  This defines all Ubuntu sepecific
    methods, including detecting the OS/Version number.  As well as
    how to check for and install packages."""
    def __init__(self, index):
        index.add_os("ubuntu", self)

    def check_presence(self):
        try:
            filename = "/etc/issue"
            if os.path.exists(filename):
                fh = open(filename, 'r')
                os_list = fh.read().split()
                if os_list[0] == "Ubuntu":
                    return True
        except:
            print "Ubuntu failed to detect OS"
            return False

        return False

    def get_version(self):
        try:
            filename = "/etc/issue"
            if os.path.exists(filename):
                fh = open(filename, 'r')
                os_list = fh.read().split()
                if os_list[0] == "Ubuntu":
                    return os_list[1]
        except:
            print "Ubuntu failed to get version"
            return False

        return False

    def detect_packages(self, packages):
        packages_not_detected = []
        for p in packages:
            fnull = open(os.devnull, 'w')
            if not subprocess.call(['dpkg', '-s', p], stdout= fnull, stderr = fnull):
                pass#print "Detected %s"%p
            else:
                packages_not_detected.append(p)
                #print "Didn't detect %s"%p
        return packages_not_detected

    def build_script(self, packages):        
        apt = "#Packages\nsudo apt-get install "
        for p in packages:
            apt += " " + p
        return apt + other



###### END UBUNTU SPECIALIZATION ########################

######################## REPLACE WITH COMMANDLINE #################
pkgs = ["rxdeps", "roscpp", "tf"]


### Find all dependencies

rosdeps = set()
for p in pkgs:
  args = ["rosdep", p]
  #print "\n\n\nmy args are", args
  deps_list = roslib.scriptutil.rospackexec(args).split('\n')
  for dep_str in deps_list:
      dep = dep_str.split()
      if len(dep) == 2 and dep[0] == "name:":
          rosdeps.add(dep[1])
      else:
          print len(dep)
          print "rospack returned wrong number of values \n%s"%dep
  

#print rosdeps
rosdeps = list(rosdeps)



### Detect OS name and version
osi = OSIndex()
rdl = RosdepLookup(osi)

################ Add All specializations here ##############################
ubuntu = Ubuntu(osi)

############## Testing ################################
print "Detected OS: " + osi.get_os_name()
print "Detected Version: " + osi.get_os_version()

native_packages = []
scripts = []
for r in rosdeps:
    specific = rdl.lookup_rosdep(r)
    if specific:
        if len(specific.split('\n')) == 1:
            for pk in specific.split():
                native_packages.append(pk)
        else:
            scripts.append(specific)
            

undetected = osi.detect_packages(native_packages)

print "Undetected packages", undetected
if len(undetected) > 0:
    print osi.build_script(undetected)
for sc in scripts:
    print sc

