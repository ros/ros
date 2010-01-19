#! /usr/bin/env python

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
#     * Neither the name of Willow Garage, Inc. nor the names of its
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

import os
import sys, string
import subprocess
import roslib.rospack
import roslib.rosenv
import roslib.stacks



class PackageFlagTracker:
  """ This will use the dependency tracker to test if packages are
  blacklisted and all their dependents. """
  def __init__(self, dependency_tracker, os_name = None, os_version = None):
    if not os_name and not os_version:
        try:
            osd = roslib.os_detect.OSDetect()
            self.os_name = osd.get_name()
            self.os_version = osd.get_version()
        except roslib.os_detect.OSDetectException, ex:
            print >> sys.stderr, "Could not detect OS. platform detection will not work"
    else:
        self.os_name = os_name
        self.os_version = os_version

    self.blacklisted = {}
    self.blacklisted_osx = {}
    self.nobuild = set()
    self.nomakefile = set()
    self.packages_tested = set()
    self.dependency_tracker = dependency_tracker
    self.paths = {}
    self.build_failed = set()

  def get_path(self, package):
    if not package in self.paths:
      self.paths[package] = roslib.packages.get_pkg_dir(package)
    return self.paths[package]

  def register_blacklisted(self, blacklisted_package, dependent_package):
    if dependent_package in self.blacklisted.keys():
      self.blacklisted[dependent_package].append(blacklisted_package)
    else:
      self.blacklisted[dependent_package] = [blacklisted_package] 
      
  def register_blacklisted_osx(self, blacklisted_package, dependent_package):
    if dependent_package in self.blacklisted_osx:
      self.blacklisted_osx[dependent_package].append(blacklisted_package)
    else:
      self.blacklisted_osx[dependent_package] =  [blacklisted_package] 

  def _check_package_flags(self, package):
    if package in self.packages_tested:
      return 
    if os.path.exists(os.path.join(self.get_path(package), "ROS_BUILD_BLACKLIST")):
      self.register_blacklisted(package, package)
      for p in roslib.rospack.rospack_depends_on(package):
        self.register_blacklisted(package, p)
        
    if os.path.exists(os.path.join(self.get_path(package), "ROS_BUILD_BLACKLIST_OSX")):
      self.register_blacklisted_osx(package, package)
      for p in roslib.rospack.rospack_depends_on(package):
        self.register_blacklisted_osx(package, p)

    if os.path.exists(os.path.join(self.get_path(package), "ROS_NOBUILD")):
      self.nobuild.add(package)

    if not os.path.exists(os.path.join(self.get_path(package), "Makefile")):
      self.nomakefile.add(package)                      

    self.packages_tested.add(package)

  def is_blacklisted(self, package):
    # this will noop if already run
    self._check_package_flags(package)

    # make sure it's not dependent on a blacklisted package
    for p in self.dependency_tracker.get_deps(package):
      if p not in self.packages_tested:
        self._check_package_flags(p)
        
    # test result after checking all dependents.
    if package in self.blacklisted:
      return self.blacklisted[package]
        
    return []

  def is_blacklisted_osx(self, package):
    # this will noop if already run
    self._check_package_flags(package)

    # make sure it's not dependent on a blacklisted_osx package
    for p in self.dependency_tracker.get_deps(package):
      if p not in self.packages_tested:
        self._check_package_flags(p)
        
    # test result after checking all dependents.
    if package in self.blacklisted_osx:
      return self.blacklisted_osx[package]
        
    return []

  def has_nobuild(self, package):
    # this will noop if already run
    self._check_package_flags(package)

    # Short circuit if known result
    if package in self.nobuild:
      return True
    return False

  def has_makefile(self, package):
    # this will noop if already run
    self._check_package_flags(package)

    # Short circuit if known result
    if package in self.nomakefile:
      return False
    return True

  def add_nobuild(self, package):
    with open(os.path.join(self.get_path(package), "ROS_NOBUILD"), 'w') as f:
      f.write("created by rosmake to mark as installed")
      self.nobuild.add(package)
      return True
    return False
    

  def remove_nobuild(self, package):
    if not self.has_nobuild(package):
      return True
    try:
      os.remove(os.path.join(self.get_path(package), "ROS_NOBUILD"))
      self.nobuild.remove(package)
      return True  
    except:
      return False


  def mark_build_failed(self, package):
      self.build_failed.add(package)

  def build_failed(self, package):
      return package in self.build_failed

  def is_whitelisted(self, package):
      return roslib.packages.platform_supported(package, self.os_name, self.os_version)
        
  def can_build(self, pkg, use_whitelist = False, use_whitelist_recursive = False, use_blacklist = False, failed_packages = [], use_makefile = True):
    """
    Return (buildable, error, "reason why not")
    """


        
    output_str = ""
    output_state = True
    buildable = True
        
    previously_failed_pkgs = [ pk for pk in failed_packages if pk in self.dependency_tracker.get_deps(pkg)]
    if len(previously_failed_pkgs) > 0:
        buildable = False
        output_state = False
        output_str += " Package %s cannot be built for dependent package(s) %s failed. \n"%(pkg, previously_failed_pkgs)


    if use_whitelist:
        non_whitelisted_packages = []
        if not self.is_whitelisted(pkg):
            buildable = False
            output_state = False
            non_whitelisted_packages.append(pkg)
        if use_whitelist_recursive:
            for p in [pk for pk in self.dependency_tracker.get_deps(pkg) if not self.is_whitelisted(pk)]:
                non_whitelisted_packages.append(p)
        if len(non_whitelisted_packages) > 0:
            output_state = False
            output_str += " Package(s) %s are not supported on this OS\n"%non_whitelisted_packages                

    if use_blacklist:
        black_listed_dependents = self.is_blacklisted(pkg)
        if len(black_listed_dependents) > 0:
            buildable = False
            output_str += "Cannot build %s ROS_BUILD_BLACKLIST found in packages %s"%(pkg, black_listed_dependents)

    if self.has_nobuild(pkg):
        buildable = False
        output_state = True # dependents are ok, it should already be built
        output_str += "ROS_NOBUILD in package %s\n"%pkg


    if use_makefile and not self.has_makefile(pkg):
        output_state = True # dependents are ok no need to build
        buildable = False
        output_str += " No Makefile in package %s\n"%pkg

    

    return (buildable, output_state, output_str)
