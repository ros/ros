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

import os
import subprocess
import sys
import threading
import time

import rospkg

if sys.hexversion > 0x03000000:  # Python3
    python3 = True
else:
    python3 = False


def _read_stdout(cmd):
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    std_out, std_err = p.communicate()
    if python3:
        return std_out.decode()
    else:
        return std_out


def num_cpus():
    """
    Detects the number of CPUs on a system. Cribbed from pp.
    """
    # Linux, Unix and MacOS:
    if hasattr(os, 'sysconf'):
        if 'SC_NPROCESSORS_ONLN' in os.sysconf_names:
            # Linux & Unix:
            ncpus = os.sysconf('SC_NPROCESSORS_ONLN')
            if isinstance(ncpus, int) and ncpus > 0:
                return ncpus
        else:  # OSX:
            return int(_read_stdout(['sysctl', '-n', 'hw.ncpu'])) or 1
    # Windows:
    if 'NUMBER_OF_PROCESSORS' in os.environ:
        ncpus = int(os.environ['NUMBER_OF_PROCESSORS'])
        if ncpus > 0:
            return ncpus
    return 1  # Default


# TODO: may no longer need this now that we've ported to rospkg
class DependencyTracker:
    """Track dependencies between packages.  This is basically a
    caching way to call rospkg. It also will allow you to specify a
    range of packages over which to track dependencies.  This is useful
    if you are only building a subset of the tree. For example with the
    --specified-only option."""
    def __init__(self, valid_packages=None, rospack=None):
        """
        @param valid_packages: defaults to rospack list
        """
        if rospack is None:
            self.rospack = rospkg.RosPack()
        else:
            self.rospack = rospack
        if valid_packages is None:
            valid_packages = self.rospack.list()
        self.valid_packages = valid_packages
        self.deps_1 = {}
        self.deps = {}

    def get_deps_1(self, package):
        if package not in self.deps_1:
            self.deps_1[package] = []
            try:
                potential_dependencies = self.rospack.get_depends(package, implicit=False)
            except rospkg.ResourceNotFound:
                potential_dependencies = []
            for p in potential_dependencies:
                if p in self.valid_packages:
                    self.deps_1[package].append(p)

        return self.deps_1[package]

    def get_deps(self, package):
        if package not in self.deps:
            self.deps[package] = []
            try:
                potential_dependencies = self.rospack.get_depends(package)
            except rospkg.ResourceNotFound:
                potential_dependencies = []

            for p in potential_dependencies:
                if p in self.valid_packages:
                    self.deps[package].append(p)
        return self.deps[package]

    def load_fake_deps(self, deps, deps1):
        self.deps = deps
        self.deps_1 = deps1
        return


class CompileThread(threading.Thread):
    """This is the class which is used as the thread for parallel
    builds.  This class will query the build queue object for new
    commands and block on its calls until the build queue says that
    building is done."""
    def __init__(self, name, build_queue, rosmakeall, argument=None):
        threading.Thread.__init__(self)
        self.build_queue = build_queue
        self.rosmakeall = rosmakeall
        self.argument = argument
        self.name = name
        self.logging_enabled = True

    def run(self):
        while not self.build_queue.is_done():
            pkg = self.build_queue.get_valid_package()
            if not pkg:
                if self.build_queue.succeeded():
                    self.rosmakeall.printer.print_verbose('[ Build Completed Thread Exiting ]', thread_name=self.name)
                else:
                    self.rosmakeall.printer.print_verbose('[ Build Terminated Thread Exiting ]', thread_name=self.name)
                break  # no more packages must be done

            # update status after accepting build
            self.rosmakeall.update_status(self.argument,
                                          self.build_queue.get_started_threads(),
                                          self.build_queue.progress_str())

            if self.argument:
                self.rosmakeall.printer.print_all('Starting >>> %s [ make %s ]' % (pkg, self.argument), thread_name=self.name)
            else:
                self.rosmakeall.printer.print_all('Starting >>> %s [ make ] ' % pkg, thread_name=self.name)
            (result, result_string) = self.rosmakeall.build(pkg, self.argument, self.build_queue.robust_build)
            self.rosmakeall.printer.print_all('Finished <<< %s %s' % (pkg, result_string), thread_name=self.name)
            # print "Finished2"
            self.build_queue.return_built(pkg, result)
            # print "returned"
            if result or self.build_queue.robust_build:
                pass  # print "result", result, "robust", self.build_queue.robust_build
            else:
                if result_string.find('[Interrupted]') != -1:
                    self.rosmakeall.printer.print_all('Caught Interruption', thread_name=self.name)
                    self.build_queue.stop()  # todo move this logic into BuildQueue itself
                    break  # unnecessary since build_queue is done now while will quit
                self.rosmakeall.printer.print_all('Halting due to failure in package %s. \n[ rosmake ] Waiting for other threads to complete.' % pkg)
                self.build_queue.stop()
                break  # unnecessary since build_queue is done now, while will quit
            # update status after at end of build
            # print "updating status"
            self.rosmakeall.update_status(self.argument,
                                          self.build_queue.get_started_threads(),
                                          self.build_queue.progress_str())
            # print "done built", len(self.build_queue.built), self.build_queue.built
            # print "failed", len(self.build_queue.failed), self.build_queue.failed
            # print "to_build", len(self.build_queue.to_build), self.build_queue.to_build
            # print "in progress", len(self.build_queue._started), self.build_queue._started

        # print "last update"
        # update status before ending thread
        self.rosmakeall.update_status(self.argument,
                                      self.build_queue.get_started_threads(),
                                      self.build_queue.progress_str())
        # print "thread finished"


class BuildQueue:
    """This class provides a thread safe build queue.  Which will do
    the sequencing for many CompileThreads."""
    def __init__(self, package_list, dependency_tracker, robust_build=False):
        self._total_pkgs = len(package_list)
        self.dependency_tracker = dependency_tracker
        self.to_build = package_list[:]  # do a copy not a reference
        self.built = []
        self.failed = []
        self.condition = threading.Condition()
        self._done = False
        self.robust_build = robust_build
        self._started = {}
        self._hack_end_counter = 0

    def progress_str(self):
        return '[ %d Active %d/%d Complete ]' % (len(self._started), len(self.built), self._total_pkgs)

    def get_started_threads(self):  # TODO sort this other than hash order
        return self._started.copy()

    def is_completed(self):
        """Return if the build queue has been completed."""
        return len(self.built) + len(self.failed) == self._total_pkgs

    def is_done(self):
        """Return if the build queue has been completed."""
        return self.is_completed() or self._done  # finished or halted

    def succeeded(self):
        """Return whether the build queue has completed all packages successfully."""
        return len(self.built) == self._total_pkgs  # flag that we're finished

    def stop(self):
        """Stop the build queue, including waking all blocking
        threads. It will not stop in flight builds."""
        self._done = True
        with self.condition:
            self.condition.notifyAll()  # wake any blocking threads

    def return_built(self, package, successful=True):  # mark that a package is built
        """The thread which completes a package marks it as done with
        this method."""
        with self.condition:
            if successful:
                self.built.append(package)
            else:
                self.failed.append(package)
            if package in self._started.keys():
                self._started.pop(package)
            else:
                pass  # used early on print "\n\n\nERROR THIS SHOULDN't RETURN %s\n\n\n"%package
            if self.is_completed():
                self._done = True
            self.condition.notifyAll()  # wake up any waiting threads

    def get_valid_package(self):  # blocking call to get a package to build returns none if done
        """This is a blocking call which will return a package which has
        all dependencies met.  If interrupted or done it will return
        None"""
        with self.condition:
            while (not self.is_done() and len(self.to_build) > 0):
                for p in self.to_build:
                    dependencies_met = True
                    for d in self.dependency_tracker.get_deps(p):
                        if d not in self.built and not (self.robust_build and d in self.failed):
                            dependencies_met = False
                            # print "Dependency %s not met for %s"%(d, p)
                            break
                    if dependencies_met:  # all dependencies met
                        self.to_build.remove(p)
                        self._started[p] = time.time()
                        self._hack_end_counter = 0  # reset end counter if success
                        return p  # break out and return package if found
                    elif len(self._started) == 0 and self._hack_end_counter > 2:
                        # we're hung with broken dependencies
                        return None
                # print "TTGTTTTHTHT Waiting on condition"
                self.condition.wait(1.0)  # failed to find a package wait for a notify before looping

                self._hack_end_counter += 1  # if we're here too often we will quit
                if self.is_done():
                    break

        return None
