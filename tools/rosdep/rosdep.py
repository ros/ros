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

#"""
#Library and command-line tool for calculating rosdeps.
#"""

from __future__ import with_statement

import roslib.rospack
import roslib.stacks
import os
import sys
import subprocess
import types
import tempfile
import yaml

class RosdepException(Exception):
    pass

class RosdepLookupPackage:
    """
    This is a class for interacting with rosdep.yaml files.  It will
    load all rosdep.yaml files in the current configuration at
    startup.  It has accessors to allow lookups into the rosdep.yaml
    from rosdep name and returns the string from the yaml file for the
    appropriate OS/version.

    It uses the OSIndex class for OS detection.
    """
    
    def __init__(self, os_name, os_version, package):
        """ Read all rosdep.yaml files found at the root of stacks in
        the current environment and build them into a map."""
        self.os_name = os_name
        self.os_version = os_version
        self.rosdep_map = {}
        self.rosdep_source = {}
        ## Find all rosdep.yamls here and load them into a map

        if package:
            self.load_for_package(package)
        
        

    def load_for_package(self, package):
        rosdep_dependent_packages = roslib.rospack.rospack_depends(package)
        rosdep_dependent_packages.append(package)

        paths = set()
        for p in rosdep_dependent_packages:
            stack = roslib.stacks.stack_of(p)
            if stack:
                paths.add( os.path.join(roslib.stacks.get_stack_dir(stack), "rosdep.yaml"))
            else:
                paths.add( os.path.join(roslib.packages.get_pkg_dir(p), "rosdep.yaml"))


        for path in paths:
            self.insert_map(self.parse_yaml(path), path)
        #print "built map", self.rosdep_map

        # Override with ros_home/rosdep.yaml if present
        ros_home = roslib.rosenv.get_ros_home()
        path = os.path.join(ros_home, "rosdep.yaml")
        self.insert_map(self.parse_yaml(path), path, override=True)

    def insert_map(self, yaml_dict, source_path, override=False):
        for key in yaml_dict:
            if key in self.rosdep_source:
                if override:
                    print >>sys.stderr, "ROSDEP_OVERRIDE: %s being overridden with %s from %s"%(key, yaml_dict[key], source_path)
                    self.rosdep_source[key].append("Overriding with "+source_path)
                    self.rosdep_map[key] = self.get_os_from_yaml(yaml_dict[key])
                else:
                    if self.rosdep_map[key] == self.get_os_from_yaml(yaml_dict[key]):
                        #print >> sys.stderr, "DEBUG: Same key found for %s: %s"%(key, self.rosdep_map[key])
                        pass
                    else:
                        print >>sys.stderr, "CONFLICT: Rules for %s do not match.  These two rules do not match: \n{{{"%key, self.rosdep_map[key],"}}}, from %s, \n{{{"%self.rosdep_source[key], self.get_os_from_yaml(yaml_dict[key]), "}}} from %s"%source_path
                        print >>sys.stderr, "QUITTING: due to conflicting rosdep definitions, please resolve."
                        exit(-1)
            else:
                self.rosdep_source[key] = [source_path]
                self.rosdep_map[key] = self.get_os_from_yaml(yaml_dict[key])
                        #print "rosdep_map[%s] = %s"%(key, self.rosdep_map[key])


    def parse_yaml(self, path):
        #print "parsing path", path
        if os.path.exists(path):
            try:
                f = open(path)
                yaml_text = f.read()
                f.close()

                return yaml.load(yaml_text)

            except yaml.YAMLError, exc:
                print >> sys.stderr, "Failed parsing yaml while processing %s\n"%path, exc
                sys.exit(1)        
                
        return {}
        
    def get_os_from_yaml(self, yaml_map):
        # See if the version for this OS exists
        if self.os_name in yaml_map:
            return self.get_version_from_yaml(yaml_map[self.os_name])
        else:
            print >> sys.stderr, "failed to find OS(%s) version of %s "%(os_name, rosdep)
            return False

    def get_version_from_yaml(self, os_specific):
        if type(os_specific) == type("String"):
            return os_specific
        else:# it must be a map of versions
            if self.os_version in os_specific.keys():
                return os_specific[os_version]
            else:
                ## Hack to match rounding errors in pyyaml load 9.04  != 9.03999999999999996 in string space
                for key in os_specific.keys():
                    # NOTE: this hack fails if os_version is not major.minor
                    if self.os_name == "ubuntu" and float(key) == float(self.os_version):
                        #print "Matched %s"%(os_version)
                        return os_specific[key]

                print >> sys.stderr, "failed to find specific version %s of %s within"%(os_version, rosdep), os_specific
                return False                    




    def lookup_rosdep(self, rosdep):
        """ Lookup the OS specific packages or script from the
        prebuilt maps."""

        if rosdep in self.rosdep_map:
            return self.rosdep_map[rosdep]
        else:
            return False
        
    def get_map(self):
        return self.rosdep_map
        

    def get_sources(self, rosdep):
        if rosdep in self.rosdep_source:
            return self.rosdep_source[rosdep]
        else:
            return []


########## Class for interacting with customized OS detectors ############
class OSIndex:
    """ This class will iterate over registered classes to lookup the
    active OS and Version of that OS for lookup in rosdep.yaml"""
    def __init__(self):
        self._os_list =[]
        try:
            self._os_detected = os.environ["ROSDEP_OS_NAME"]
        except:
            self._os_detected = False
        try:
            self._os_version = os.environ["ROSDEP_OS_VERSION"]
        except:
            self._os_version = False

    def add_os(self, class_ref):
        self._os_list.append(class_ref)

        # \TODO look at throwing here
    def get_os(self):
        if not self._os_detected:
            for os_class in self._os_list:
                if os_class.check_presence():
                    self._os_detected = os_class
                    return os_class
        if not self._os_detected:
            print "Failed to detect OS"
            sys.exit(-1) # TODO do this more elegantly

        return self._os_detected
    def get_os_name(self):
        os_class = self.get_os()
        if os_class:
            return os_class.get_name()
        else:
            return False

    def get_os_version(self):
        if not self._os_version:
            self._os_version = self.get_os().get_version()
        return self._os_version

    def strip_detected_packages(self, packages):
        return self.get_os().strip_detected_packages(packages)

    def generate_package_install_command(self, packages, default_yes):
        if len(packages) > 0:
            bash_script = ""
            try:
                bash_script = self.get_os().generate_package_install_command(packages, default_yes)
            except KeyError:
                return "# os name '%s' not registered as a valid os"%self.get_os_name()
            return bash_script
        else:
            return "#No packages to install: skipping package install command.\n"

####### Linux Helper Functions #####
def lsb_get_os():
    try:
        cmd = ['lsb_release', '-si']
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        return std_out.strip()
    except:
        return None
    
def lsb_get_codename():
    try:
        cmd = ['lsb_release', '-sc']
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        return std_out.strip()
    except:
        return None
    
def lsb_get_release_version():
    try:
        cmd = ['lsb_release', '-sr']
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        return std_out.strip()
    except:
        return None


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

###### Arch SPECIALIZATION #########################

def pacman_detect(p):
    return subprocess.call(['pacman', '-Q', p], stdout=subprocess.PIPE, stderr=subprocess.PIPE)    

class Arch:

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


    def strip_detected_packages(self, packages):
        return [p for p in packages if pacman_detect(p)]

    def generate_package_install_command(self, packages, default_yes):        
        return "#Packages\nsudo pacman -Sy --needed " + ' '.join(packages)

###### END Arch SPECIALIZATION ########################


###### Macports SPECIALIZATION #########################
def port_detect(p):
    cmd = ['port', 'installed', p]
    pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (std_out, std_err) = pop.communicate()
    
    return (std_out.count("(active)") > 0)

class Macports:
    def check_presence(self):
        filename = "/usr/bin/sw_vers"
        if os.path.exists(filename):
            return True
        return False
    
    def get_version(self):
        return "macports" # macports is a rolling release and isn't versionsed

    def get_name(self):
        return "macports"

    def strip_detected_packages(self, packages):
        return [p for p in packages if not port_detect(p)] 

    def generate_package_install_command(self, packages, default_yes):        
        return "#Packages\nsudo port install " + ' '.join(packages)

###### END Macports SPECIALIZATION ########################


def yum_detect(p):
    return subprocess.call(['yum', 'list', p], stdout=subprocess.PIPE, stderr=subprocess.PIPE)    

###### Fedora SPECIALIZATION #########################
class Fedora:
    def check_presence(self):
        try:
            filename = "/etc/redhat_release"
            if os.path.exists(filename):
                with open(filename, 'r') as fh:                
                    os_list = fh.read().split()
                if os_list and os_list[0] == "Fedora" and os_list[1] == "release":
                    return True
        except:
            print "Fedora failed to detect OS"
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

    def strip_detected_packages(self, packages):
        return [p for p in packages if yum_detect(p)]

    def generate_package_install_command(self, packages, default_yes):
        if default_yes:
            return "#Packages\nyum -y install " + ' '.join(packages)
        else:
            return "#Packages\nyum install " + ' '.join(packages)

###### END Fedora SPECIALIZATION ########################

###### Rhel SPECIALIZATION #########################
class Rhel(Fedora):
    def check_presence(self):
        try:
            filename = "/etc/redhat_release"
            if os.path.exists(filename):
                with open(filename, 'r') as fh:                
                    os_list = fh.read().split()
                if os_list and os_list[2] == "Enterprise":
                    return True
        except:
            print "Rhel failed to detect OS"
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


class Rosdep:
    def __init__(self, packages, command = "rosdep", robust = False):
        self.osi = OSIndex()
        self.rosdeps = self.gather_rosdeps(packages, command)
        self.robust = robust

    def gather_rosdeps(self, packages, command):
        rosdeps = {}
        for p in packages:
          args = [command, p]
          #print "\n\n\nmy args are", args
          deps_list = [x for x in roslib.rospack.rospackexec(args).split('\n') if x]
          rosdeps[p] = []
          for dep_str in deps_list:
              dep = dep_str.split()
              if len(dep) == 2 and dep[0] == "name:":
                  rosdeps[p].append(dep[1])
              else:
                  print len(dep)
                  print "rospack returned wrong number of values \n\"%s\""%dep_str

                  
        # todo deduplicate
        return rosdeps

    def get_packages_and_scripts(self):
        native_packages = []
        scripts = []
        failed_rosdeps = []
        for p in self.rosdeps:
            rdlp = RosdepLookupPackage(self.osi.get_os_name(), self.osi.get_os_version(), p)
            for r in self.rosdeps[p]:
                specific = rdlp.lookup_rosdep(r)
                if specific:
                    if len(specific.split('\n')) == 1:
                        for pk in specific.split():
                            native_packages.append(pk)
                    else:
                        scripts.append(specific)
                else:
                    failed_rosdeps.append(r)

        if len(failed_rosdeps) > 0:
            if not self.robust:
                raise RosdepException("Rosdeps %s could not be resolved"%failed_rosdeps)
            else:
                print >> sys.stderr, "WARNING: Rosdeps %s could not be resolved"%failed_rosdeps
        return (list(set(native_packages)), list(set(scripts)))

    def get_native_packages(self):
        return get_packages_and_scripts()[0]

    def generate_script(self, include_duplicates=False, default_yes = False):
        native_packages, scripts = self.get_packages_and_scripts()
        undetected = native_packages if include_duplicates else \
            self.osi.strip_detected_packages(native_packages)
        return self.osi.generate_package_install_command(undetected, default_yes) + \
            "\n".join(["\n%s"%sc for sc in scripts])
        
    def check(self):
        try:
            native_packages, scripts = self.get_packages_and_scripts()
        except RosdepException:
            pass
        undetected = self.osi.strip_detected_packages(native_packages)
        return_str = ""
        if len(undetected) > 0:
            return_str += "Did not detect packages: %s\n"%undetected
        if len(scripts) > 0:
            return_str += "The following scripts were not tested:\n"
        for s in scripts:
            return_str += s + '\n'
        return return_str

    def what_needs(self, rosdep_args):
        packages = []
        for p in roslib.packages.list_pkgs():
            rosdeps_needed = self.gather_rosdeps([p], "rosdep")[p]
            matches = [r for r in rosdep_args if r in rosdeps_needed]
            for r in matches:
                packages.append(p)
                
        return packages

    def install(self, include_duplicates, default_yes):
        with tempfile.NamedTemporaryFile() as fh:
            script = self.generate_script(include_duplicates, default_yes)
            fh.write(script)
            fh.flush()
            
            print "executing this script:\n %s"%script
            p= subprocess.Popen(['bash', fh.name])
            p.communicate()
                    
    def depdb(self, packages):
        output = "Rosdep dependencies for operating system %s version %s "%(self.osi.get_os_name(), self.osi.get_os_version())
        for p in packages:
            output += "\nPACKAGE: %s\n"%p
            rdlp = RosdepLookupPackage(self.osi.get_os_name(), self.osi.get_os_version(), p)
            map = rdlp.get_map()
            for k in map:
                output = output + "<<<< %s -> %s >>>>\n"%(k, map[k])
        return output

    def where_defined(self, rosdeps):
        output = ""
        locations = {}
        rdlp = RosdepLookupPackage(self.osi.get_os_name(), self.osi.get_os_version(), None)
        
        for r in rosdeps:
            locations[r] = set()

        path = os.path.join(roslib.rosenv.get_ros_home(), "rosdep.yaml")
        rosdep_dict = rdlp.parse_yaml(path)
        for r in rosdeps:
            if r in rosdep_dict:
                locations[r].add("Override:"+path)
            

        for p in roslib.packages.list_pkgs():
            path = os.path.join(roslib.packages.get_pkg_dir(p), "rosdep.yaml")
            rosdep_dict = rdlp.parse_yaml(path)
            for r in rosdeps:
                if r in rosdep_dict:
                    addendum = ""
                    if roslib.stacks.stack_of(p):
                        addendum = "<<Unused due to package '%s' being in a stack.]]"%p
                    locations[r].add(">>" + path + addendum)
            

        for s in roslib.stacks.list_stacks():
            path = os.path.join(roslib.stacks.get_stack_dir(s), "rosdep.yaml")
            rosdep_dict = rdlp.parse_yaml(path)
            for r in rosdeps:
                if r in rosdep_dict:
                    locations[r].add(path)
            
        for rd in locations:
            output += "%s defined in %s"%(rd, locations[rd])
        return output

################################################################################
# COMMAND LINE PROCESSING
    
_usage = """usage: rosdep [options] <command> <args>

Commands:

rosdep generate_bash  <packages>...
rosdep satisfy <packages>...
  will try to generate a bash script which will satisfy the 
  dependencies of package(s) on your operating system.

rosdep install <packages>...
  will generate a bash script and then execute it.

rosdep depdb <packages>...
  will generate the dependency database for package(s) and print
  it to the console (note that the database will change depending
  on which package(s) you query.

rosdep what_needs <rosdeps>...
  will print a list of packages that declare a rosdep on (at least
  one of) ROSDEP_NAME[S]

rosdep where_defined <rosdeps>...
  will print a list of yaml files that declare a rosdep on (at least
  one of) ROSDEP_NAME[S]

rosdep check <packages>...
  will check if the dependencies of package(s) have been met.
"""

_commands = ['generate_bash', 'satisfy', 'install', 'depdb', 'what_needs', 'check', 'where_defined']

def main():
    from optparse import OptionParser
    parser = OptionParser(usage=_usage, prog='rosdep')
    parser.add_option("--verbose", "-v", dest="verbose", default=False, 
                      action="store_true", help="verbose display")
    parser.add_option("--include_duplicates", "-i", dest="include_duplicates", default=False, 
                      action="store_true", help="do not deduplicate")
    parser.add_option("--default-yes", "-y", dest="default_yes", default=False, 
                      action="store_true", help="Tell the package manager to default to y or fail when installing")
    parser.add_option("-r", "-k", dest="robust", default=False, 
                      action="store_true", help="Continue installing despite errors.")
    parser.add_option("-a", "--all", dest="rosdep_all", default=False, 
                      action="store_true", help="select all packages")

    options, args = parser.parse_args()


    if len(args) == 0:
        parser.error("Please enter a command")
    command = args[0]
    if not command in _commands:
        parser.error("Unsupported command %s."%command)
    if len(args) < 2:
        parser.error("Please enter arguments for '%s'"%command)
    rdargs = args[1:]

    verified_packages = []


    if not (command == "what_needs" or command == "where_defined" ): # package mode
        if options.rosdep_all:
            rdargs = roslib.packages.list_pkgs()
        
        (verified_packages, rejected_packages) = roslib.stacks.expand_to_packages(rdargs)
    
        if len(rejected_packages) > 0:
            print "Warning: could not identify %s as a package"%rejected_packages
        if len(verified_packages) == 0:
            parser.error("No Valid Packages listed")
                
    else: # rosdep as argumets 
        if options.rosdep_all:
            parser.error("-a, --all is not a valid option for this command")

    ### Find all dependencies
    r = Rosdep(verified_packages, robust=options.robust)

    ### Detect OS name and version

    ################ Add All specializations here ##############################
    r.osi.add_os(Ubuntu())
    r.osi.add_os(Debian())
    r.osi.add_os(Fedora())
    r.osi.add_os(Rhel())
    r.osi.add_os(Arch())
    r.osi.add_os(Macports())
    ################ End Add specializations here ##############################
    
    if options.verbose:
        print "Detected OS: " + r.osi.get_os_name()
        print "Detected Version: " + r.osi.get_os_version()

    try:
        if command == "generate_bash" or command == "satisfy":
            print r.generate_script(include_duplicates=options.include_duplicates, default_yes=options.default_yes)
            return True
        elif command == "install":
            r.install(options.include_duplicates, options.default_yes);
            return True
    except RosdepException, e:
        print "ERROR: %s"%e
        return False
        
    if command == "depdb":
        print r.depdb(verified_packages)
        return True

    elif command == "what_needs":
        print '\n'.join(r.what_needs(rdargs))
        return True

    elif command == "where_defined":
        print r.where_defined(rdargs)
        return True

    elif command == "check":
        output = r.check()
        if len(rejected_packages) > 0:
            print "Arguments %s are not packages"%rejected_packages
            return False
        if len(output) == 0:
            return True
        else:
            print "check failed", output
            return False

if __name__ == '__main__':
    sys.exit(not main() or 0)
