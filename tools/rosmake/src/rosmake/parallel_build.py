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
import re
import distutils.version
import sys, string
import subprocess
import time
import getopt
import roslib
import roslib.rospack
import roslib.rosenv
import roslib.stacks
import threading


def num_cpus():
  """
  Detects the number of CPUs on a system. Cribbed from pp.
  """
  # Linux, Unix and MacOS:
  if hasattr(os, "sysconf"):
    if os.sysconf_names.has_key("SC_NPROCESSORS_ONLN"):
      # Linux & Unix:
      ncpus = os.sysconf("SC_NPROCESSORS_ONLN")
      if isinstance(ncpus, int) and ncpus > 0:
        return ncpus
    else: # OSX:
      return int(os.popen2("sysctl -n hw.ncpu")[1].read())
  # Windows:
  if os.environ.has_key("NUMBER_OF_PROCESSORS"):
    ncpus = int(os.environ["NUMBER_OF_PROCESSORS"]);
    if ncpus > 0:
      return ncpus
  return 1 # Default

class DependencyTracker:
  """ Track dependencies between packages.  This is basically a
  caching way to call roslib. It also will allow you to specifiy a
  range of packages over which to track dependencies.  This is useful
  if you are only building a subset of the tree. For example with the
  --specified-only option. """
  def __init__(self, valid_packages = roslib.rospack.rospackexec(["list-names"]).split()):
    self.valid_packages = valid_packages
    self.deps_1 = {}
    self.deps = {}

  def get_deps_1(self, package):
    if not package in self.deps_1:
      self.deps_1[package] = []
      potential_dependencies = roslib.rospack.rospack_depends_1(package) 
      for p in potential_dependencies:
        if p in self.valid_packages:
          self.deps_1[package].append(p)
      
    return self.deps_1[package]

  def get_deps(self, package):
    if not package in self.deps:
      self.deps[package] = []
      potential_dependencies = roslib.rospack.rospack_depends(package) 
      for p in potential_dependencies:
        if p in self.valid_packages:
          self.deps[package].append(p)
    return self.deps[package]

  def load_fake_deps(self, deps, deps1):
    self.deps = deps
    self.deps_1 = deps1
    return


class CompileThread(threading.Thread):
  """ This is the class which is used as the thread for parallel
  builds.  This class will query the build queue object for new
  commands and block on its calls until the build queue says that
  building is done. """
  def __init__(self, name, build_queue, rosmakeall, argument = None):
    threading.Thread.__init__(self)
    self.build_queue = build_queue
    self.rosmakeall = rosmakeall
    self.argument = argument
    self.name = name
    self.logging_enabled = True

  def run(self):
    #init_total_pkgs = len(self.build_queue.to_build)
    while not self.build_queue.is_done():
      (pkg, build_count, total_pkgs)  = self.build_queue.get_valid_package()
      if not pkg:
        if self.build_queue.succeeded():
          self.rosmakeall.print_verbose("[ Build Completed Thread Exiting ]", thread_name=self.name);
        else:
          self.rosmakeall.print_verbose("[ Build Terminated Thread Exiting ]", thread_name=self.name)
        break # no more packages must be done

      self.rosmakeall.print_all("[ %d of %d  Completed ]"%(  build_count, total_pkgs));

      if self.argument:
        self.rosmakeall.print_all (">>> %s >>> [ make %s ]"%(pkg, self.argument), thread_name=self.name)
      else:
        self.rosmakeall.print_all (">>> %s >>> [ make ]"%(pkg), thread_name=self.name)

      (result, result_string) = self.rosmakeall.build(pkg, self.argument, self.build_queue.robust_build) 
      self.rosmakeall.print_all("<<< %s <<< %s"%(pkg, result_string), thread_name= self.name)
      if result or self.build_queue.robust_build:
        self.build_queue.return_built(pkg)
        if result_string.find("[Interrupted]") != -1:
          self.rosmakeall.print_all("Caught Interruption", thread_name=self.name)
          self.build_queue.stop()
          break # unnecessary since build_queue is done now while will quit
      else:
        self.rosmakeall.print_all("Halting due to failure in package %s.\n[ rosmake ] Re-run with -r to ignore failures and keep building."%pkg)
        self.build_queue.stop()
        break # unnecessary since build_queue is done now, while will quit

class BuildQueue:
  """ This class provides a thread safe build queue.  Which will do
  the sequencing for many CompileThreads. """
  def __init__(self, package_list, dependency_tracker, robust_build = False):
    self._total_pkgs = len(package_list)
    self.dependency_tracker = dependency_tracker
    self.to_build = package_list[:] # do a copy not a reference
    self.built = []
    self.condition = threading.Condition()
    self._done = False
    self.robust_build = robust_build

  def is_done(self):
    """Return if the build queue has been completed """
    return self._done

  def succeeded(self):
    """ Return whether the build queue has completed all packages successfully. """
    return len( self.built)  == self._total_pkgs and self._done

  def stop(self): 
    """ Stop the build queue, including waking all blocking
    threads. It will not stop in flight builds."""
    self._done = True
    with self.condition:
      self.condition.notifyAll() # wake any blocking threads
      
  def return_built(self, package): # mark that a package is built
    """ The thread which completes a package marks it as done with
    this method."""
    with self.condition:
      self.built.append(package)
      if len(self.built) == self._total_pkgs:  #flag that we're finished
        self._done = True
      self.condition.notifyAll() #wake up any waiting threads

  def get_valid_package(self): # blocking call to get a package to build returns none if done
    """ This is a blocking call which will return a package which has
    all dependencies met.  If interrupted or done it will return
    None"""
    with self.condition:
      while (not self._done and len(self.to_build) > 0):
        for p in self.to_build:
          dependencies_met = True
          for d in self.dependency_tracker.get_deps(p):
            if d not in self.built:
              dependencies_met = False
              break
          if dependencies_met:  # all dependencies met
            self.to_build.remove(p)
            count = len(self.built)
            return (p, count, self._total_pkgs) # break out and return package if found


        self.condition.wait()  # failed to find a package wait for a notify before looping
        if self.is_done():
          break

    return (None, None, None)

