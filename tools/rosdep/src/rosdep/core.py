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
import roslib.os_detect
import os
import sys
import subprocess
import types
import tempfile
import yaml

import debian
import redhat
import macports
import arch
import core


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
        self.package = package
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
                for s in roslib.rospack.rosstack_depends(stack):
                    paths.add( os.path.join(roslib.stacks.get_stack_dir(s), "rosdep.yaml"))
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
            print >> sys.stderr, "failed to resolve a rule for OS(%s)"%(self.os_name)
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
                    if self.os_name == "ubuntu" and float(key) == 9.1 and self.os_version == "karmic":
                        print >> sys.stderr, "Warning matched key '9.10' for %s please convert to using 'karmic'"%os_specific[key]
                        return os_specific[key]
                    if self.os_name == "ubuntu" and float(key) == 9.04 and self.os_version == "jaunty":
                        print >> sys.stderr, "Warning matched key '9.04' for %s please convert rosdep.yaml to using 'jaunty'"%os_specific[key]
                        return os_specific[key]
                    if self.os_name == "ubuntu" and float(key) == 8.04 and self.os_version == "hardy":
                        print >> sys.stderr, "Warning matched key '8.04' for %s please convert to using 'hardy'"%os_specific[key]
                        return os_specific[key]

                print >> sys.stderr, "failed to find specific version %s of %s within"%(os_version, rosdep), os_specific
                return False                    




    def lookup_rosdep(self, rosdep):
        """ Lookup the OS specific packages or script from the
        prebuilt maps."""

        if rosdep in self.rosdep_map:
            return self.rosdep_map[rosdep]
        else:
            print >> sys.stderr, "Failed to find rosdep %s for package %s"%(rosdep, self.package)
            return False
        
    def get_map(self):
        return self.rosdep_map
        

    def get_sources(self, rosdep):
        if rosdep in self.rosdep_source:
            return self.rosdep_source[rosdep]
        else:
            return []




 

class Rosdep:
    def __init__(self, packages, command = "rosdep", robust = False):
        os_list = [debian.Ubuntu(), debian.Debian(), debian.Mint(), redhat.Fedora(), redhat.Rhel(), arch.Arch(), macports.Macports()]
        self.osi = roslib.os_detect.OSAbstraction(os_list)
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
            rdlp = RosdepLookupPackage(self.osi.get_name(), self.osi.get_version(), p)
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
            self.osi.get_os_specific_class().strip_detected_packages(native_packages)
        return "set -o errexit\n" + self.osi.get_os_specific_class().generate_package_install_command(undetected, default_yes) + \
            "\n".join(["\n%s"%sc for sc in scripts])
        
    def check(self):
        try:
            native_packages, scripts = self.get_packages_and_scripts()
        except RosdepException:
            pass
        undetected = self.osi.get_os_specific_class().strip_detected_packages(native_packages)
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
            rosdeps_needed = self.gather_rosdeps([p], "rosdep0")[p]
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
        output = "Rosdep dependencies for operating system %s version %s "%(self.osi.get_name(), self.osi.get_version())
        for p in packages:
            output += "\nPACKAGE: %s\n"%p
            rdlp = RosdepLookupPackage(self.osi.get_name(), self.osi.get_version(), p)
            map = rdlp.get_map()
            for k in map:
                output = output + "<<<< %s -> %s >>>>\n"%(k, map[k])
        return output

    def where_defined(self, rosdeps):
        output = ""
        locations = {}
        rdlp = RosdepLookupPackage(self.osi.get_name(), self.osi.get_version(), None)
        
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

