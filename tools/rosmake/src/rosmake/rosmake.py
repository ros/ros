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
import math

import parallel_build
import package_stats

from optparse import OptionParser

import rosdep

def make_command():
    return os.environ.get("MAKE", "make")


class RosMakeAll:
    def __init__(self):
        self.result = {}
        self.paths = {}
        self.dependency_tracker = parallel_build.DependencyTracker()
        self.flag_tracker = parallel_build.PackageFlagTracker(self.dependency_tracker)
        self.output = {}
        self.verbose = False
        self.full_verbose = False
        self.profile = {}
        self.ros_parallel_jobs = 0
        self.build_list = []
        self.start_time = time.time()
        self.log_dir = ""
        self.logging_enabled = True
        self.obey_whitelist = False
        self.obey_whitelist_recursively = False


    def num_packages_built(self):
        return len(self.result[argument].keys())

    def get_path(self, package):
        if not package in self.paths:
            self.paths[package] = roslib.packages.get_pkg_dir(package)
        return self.paths[package]
        
    def check_rosdep(self, packages):
        self.print_all("Checking rosdeps compliance for packages %s.  This may take a few seconds."%(', '.join(packages)))
        r = rosdep.core.Rosdep(packages, robust=True)
        output = r.check()
        if len(output) == 0:
            self.print_all( "Rosdep check passed all packages")# %s"% packages)
            return True
        else:
            self.print_all("Rosdep check failed packages: %s"% output)
            return False

    def install_rosdeps(self, packages, default_yes):
        self.print_all("Generating Install Script using rosdep then executing. This may take a minute, you will be prompted for permissions. . .")
        r = rosdep.core.Rosdep(packages, robust=True)
        try:
            r.install(include_duplicates=False, default_yes=default_yes);
            self.print_all("Rosdep successfully installed all packages")
            return True
        except rosdep.RosdepException, e:
            self.print_all( "ERROR: %s"%e)
            return False

    def build_or_recurse(self,p):
        if p in self.build_list:
            return
        for d in self.dependency_tracker.get_deps_1(p):
            self.build_or_recurse(d)
        try: # append it ot the list only if present
          self.get_path(p)
          self.build_list.append(p)
        except roslib.packages.InvalidROSPkgException, ex:
          if not self.robust_build:
            self.print_all("Exiting due to missing package: %s"%ex)
            sys.exit(-1)
          else:
            self.print_all("!"*20 + " Package %s does not exist. %s"%(p, ex) + "!"*20)


    def parallel_build_pkgs(self, build_queue, argument = None, threads = 1):
        self.profile[argument] = {}
        self.output[argument] = {}
        self.result[argument] = {}

        cts = []
        for i in  xrange(0, threads):
          ct = parallel_build.CompileThread(str(i), build_queue, self, argument)
          ct.start()
          cts.append(ct)
        for ct in cts:
          try:
            ct.join()
            #print "naturally ended thread", ct
          except KeyboardInterrupt:
            self.print_all( "Caught KeyboardInterrupt. Stopping build.")
            build_queue.stop()
            ct.join()
            pass
        all_pkgs_passed = True
        for v in self.result[argument].values():
          all_pkgs_passed = v and all_pkgs_passed

        build_passed = build_queue.succeeded() and all_pkgs_passed
        return build_passed

    def build_package(self, package, argument=None):
        local_env = os.environ.copy()
        if self.ros_parallel_jobs > 0:
            local_env['ROS_PARALLEL_JOBS'] = "-j%d" % self.ros_parallel_jobs
        elif "ROS_PARALLEL_JOBS" not in os.environ: #if no environment setup and no args fall back to # cpus
            local_env['ROS_PARALLEL_JOBS'] = "-j%d" % parallel_build.num_cpus()
        local_env['SVN_CMDLINE'] = "svn --non-interactive"
        cmd = ["bash", "-c", "cd %s && %s "%(self.get_path(package), make_command()) ]
        if argument:
            cmd[-1] += argument
        self.print_full_verbose (cmd)
        command_line = subprocess.Popen(cmd, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT, env=local_env)
        (pstd_out, pstd_err) = command_line.communicate() # pstd_err should be None due to pipe above
        return (command_line.returncode, pstd_out)

    def build(self, p, argument = None, robust_build=False):
        return_string = ""
        try:
            if p == "rospack":
                return_string = ("[SKIP] rosmake uses rospack.  If building it is already built, if cleaning it will be cleaned at the end.")
                return (True, return_string) # This will be caught later
            # warn if ROS_BUILD_BLACKLIST encountered if applicable
            failed_packages = [j for j in self.result[argument] if not self.result[argument][j] == True]
            (buildable, error, why) = package_stats.can_build(p, self.obey_whitelist, self.obey_whitelist_recursively, self.skip_blacklist, failed_packages)
            if buildable or self.robust_build:
                start_time = time.time()
                (returncode, pstd_out) = self.build_package(p, argument)
                self.profile[argument][p] = time.time() - start_time
                self.output[argument][p] = pstd_out
                if argument:
                    log_type = "build_%s"%argument
                else:
                    log_type = "build"
                if not returncode:
                    self.print_full_verbose( pstd_out)
                    self.result[argument][p] = True
                    num_warnings = len(re.findall("warning:", pstd_out))
                    if num_warnings > 0:
                        return_string =  ("[PASS] [ %.2f seconds ] -- WARNING: %d compiler warnings"%(self.profile[argument][p], num_warnings))
                    else:
                        return_string =  ("[PASS] [ %.2f seconds ]"%( self.profile[argument][p]))
                    self.output_to_file(p, log_type, pstd_out, num_warnings > 0)
                else:
                    no_target = len(re.findall("No rule to make target", pstd_out)) > 0
                    interrupt = len(re.findall("Interrupt", pstd_out)) > 0
                    if no_target:
                        return_string = ( "[SKIP] No rule to make target %s"%( argument))
                    elif interrupt:
                        return_string = ("[Interrupted]" )
                    else:
                        return_string = ( "[FAIL] [ %.2f seconds ]"%( self.profile[argument][p]))
                    self.result[argument][p] = True if no_target else False
                    if self.robust_build or interrupt:
                      self.print_verbose( pstd_out)
                    else:
                      self.print_tail( pstd_out)
                    self.output_to_file(p, log_type, pstd_out, always_print= not (no_target or interrupt))

                    return (False, return_string)
            else:
                self.result[argument][p] = error
                return_string += why
                return(error, return_string)
            return (True, return_string) # this means that we didn't error in any case above
        except roslib.packages.InvalidROSPkgException, ex:
            self.result[argument][p] = False
            self.print_verbose ("[SKIP] Package not found\n")
            self.output[argument][p] = "Package not found %s"%ex
            return (False, return_string)
            

    def output_to_file(self, package, log_type, stdout, always_print= False):
        if not self.logging_enabled:
          return
        package_log_dir = os.path.join(self.log_dir, package)

        std_out_filename = os.path.join(package_log_dir, log_type + "_output.log")
        if not os.path.exists (package_log_dir):
            os.makedirs (package_log_dir)
        with open(std_out_filename, 'w') as stdout_file:
            stdout_file.write(stdout)
            print_string = "Output from build of package %s written to:\n[ rosmake ]    %s"%(package, std_out_filename)
            if always_print:
                self.print_all(print_string)
            else:
                self.print_full_verbose(print_string)

    def generate_summary_output(self, log_dir):
        if not self.logging_enabled:
          return

        self.print_all("Summary output to directory")
        self.print_all("%s"%self.log_dir)


        if None in self.result.keys():
            if len(self.result[None].keys()) > 0:
                buildfail_filename = os.path.join(log_dir, "buildfailures.txt")
                with open(buildfail_filename, 'w') as bf:
                    bf.write("Build failures:\n")
                    for key in self.build_list:
                        if key in self.result[None].keys() and self.result[None][key] == False:
                            bf.write("%s\n"%key)
                if  None in self.output.keys():
                    buildfail_context_filename = os.path.join(log_dir, "buildfailures-with-context.txt")
                    with open(buildfail_context_filename, 'w') as bfwc:
                        bfwc.write("Build failures with context:\n")
                        for key in self.build_list:
                            if key in self.result[None].keys() and self.result[None][key] == False:
                                bfwc.write("---------------------\n")
                                bfwc.write("%s\n"%key)
                                if key in self.output[None]:
                                    bfwc.write(self.output[None][key])

        if "test" in self.result.keys():
            if len(self.result["test"].keys()) > 0:
                testfail_filename = os.path.join(log_dir, "testfailures.txt")
                with open(testfail_filename, 'w') as btwc:
                    btwc.write("Test failures:\n")
                    for key in self.build_list:
                        if key in self.result["test"].keys() and self.result["test"][key] == False:
                            btwc.write("%s\n"%key)

                if "test" in self.output.keys():
                    testfail_filename = os.path.join(log_dir, "testfailures-with-context.txt")
                    with open(testfail_filename, 'w') as btwc:
                        btwc.write("Test failures with context:\n")
                        for key in self.build_list:
                            if key in self.result["test"].keys() and self.result["test"][key] == False:
                                btwc.write("%s\n"%key)
                                if key in self.output["test"]:
                                    btwc.write(self.output["test"][key])

        profile_filename = os.path.join(log_dir, "profile.txt")
        with open(profile_filename, 'w') as pf:
            pf.write(self.get_profile_string())
                            
                            

    def get_profile_string(self):
        output = '--------------\nProfile\n--------------\n'
        total = 0.0
        count = 1
        for key in self.build_list:
            build_results = ["[Not Built ]", "[  Built   ]", "[Build Fail]"];
            test_results =  ["[Untested ]", "[Test Pass]", "[Test Fail]"];
            build_result = 0
            test_result = 0
            test_time = 0.0
            build_time = 0.0

            if None in self.result.keys():
                if key in self.result[None].keys():
                    if self.result[None][key] == True:
                        build_result = 1
                    else:
                        build_result = 2

            if "test" in self.profile.keys():
                if key in self.result["test"].keys():
                    if self.result["test"][key] == True:
                        test_result = 1
                    else:
                        test_result = 2

            if None in self.profile.keys():
                if key in self.profile[None].keys():
                    build_time = self.profile[None][key]

            if "test" in self.profile.keys():
                if key in self.profile["test"].keys():
                    test_time = self.profile["test"][key]
                
                    
            output = output + "%3d: %s in %d:%.2f %s in %.2f --- %s\n"% (count, build_results[build_result], math.floor(build_time/60), build_time%60 , test_results[test_result], test_time, key)
            total = total + build_time
            count = count + 1

        elapsed_time  =  self.finish_time - self.start_time
        output = output + "----------------\n" + "%.2f Cumulative,  %.2f Elapsed, %.2f Speedup \n"%(total, elapsed_time, float(total) / float(elapsed_time))
        return output

    def print_all(self, s, newline = True, thread_name=None):
      if thread_name == None:
        if newline:
          print "[ rosmake ]", s
        else:
          print "[ rosmake ]", s,
          sys.stdout.flush()
      else:
        if newline:
          print "[rosmake-%s]"%thread_name, s
        else:
          print "[rosmake-%s]"%thread_name, s
          sys.stdout.flush()

    def print_verbose(self, s, thread_name=None):
        if self.verbose or self.full_verbose:
          if thread_name:
            self.print_all(s, thread_name=thread_name)
          else:
            print "[ rosmake ]", s

    def print_full_verbose(self, s):
        if self.full_verbose:
            print "[ rosmake ] ", s

    def print_tail(self, s, tail_lines=40):
      lines = s.splitlines()

      num_lines = min(len(lines), tail_lines)
      if num_lines == tail_lines:
        print "[ rosmake ] Last %d lines"%num_lines
      else:
        print "[ rosmake ] All %d lines"%num_lines
      print "{" + "-"*79
      for l in xrange(-num_lines, -1):
        print "  %s"%lines[l]
      print "-"*79 + "}"

    def assert_rospack_built(self):
        if self.flag_tracker.has_nobuild("rospack"):
            return True
        ret_val = subprocess.call(["bash", "-c", "cd %s && %s "%(os.path.join(os.environ["ROS_ROOT"], "tools/rospack"), make_command())]) 
        ret_val2 = subprocess.call(["bash", "-c", "cd %s && %s "%(os.path.join(os.environ["ROS_ROOT"], "3rdparty/gtest"), make_command())]) 
        return ret_val and ret_val2
            
        # The check for presence doesn't check for updates
        #if os.path.exists(os.path.join(os.environ["ROS_ROOT"], "bin/rospack")):
        #    return True
        #else:
        #    print "Rosmake detected that rospack was not built.  Building it for you because it is required."
        #    return subprocess.call(["make", "-C", os.path.join(os.environ["ROS_ROOT"], "tools/rospack")])



    def is_rosout_built(self):
        return os.path.exists(os.path.join(roslib.packages.get_pkg_dir("rosout"), "rosout"))
            

    def main(self):
        parser = OptionParser(usage="usage: %prog [options] [PACKAGE]...", prog='rosmake')
        parser.add_option("--test-only", dest="test_only", default=False,
                          action="store_true", help="only run tests")
        parser.add_option("-t", dest="test", default=False,
                          action="store_true", help="build and test packages")
        parser.add_option("-a", "--all", dest="build_all", default=False,
                          action="store_true", help="select all packages")
        parser.add_option("-i", "--mark-installed", dest="mark_installed", default=False,
                          action="store_true", help="On successful build, mark packages as installed with ROS_NOBUILD")
        parser.add_option("-u", "--unmark-installed", dest="unmark_installed", default=False,
                          action="store_true", help="Remove ROS_NOBUILD from the specified packages.  This will not build anything.")
        parser.add_option("-v", dest="verbose", default=False,
                          action="store_true", help="display errored builds")
        parser.add_option("-r","-k", "--robust", dest="best_effort", default=False,
                           action="store_true", help="do not stop build on error")
        parser.add_option("--build-everything", dest="robust", default=False,
                           action="store_true", help="build all packages regardless of errors")
        parser.add_option("-V", dest="full_verbose", default=False,
                          action="store_true", help="display all builds")
        parser.add_option("-s", "--specified-only", dest="specified_only", default=False,
                          action="store_true", help="only build packages specified on the command line")
        parser.add_option("--buildtest", dest="buildtest",
                          action="append", help="package to buildtest")
        parser.add_option("--buildtest1", dest="buildtest1",
                          action="append", help="package to buildtest1")
        parser.add_option("--output", dest="output_dir",
                          action="store", help="where to output results")
        parser.add_option("--pre-clean", dest="pre_clean",
                          action="store_true", help="run make clean first")
        parser.add_option("--disable-logging", dest="logging_enabled", default=True,
                          action="store_false", help="turn off all logs")
        parser.add_option("--target", dest="target",
                          action="store", help="run make with this target")
        parser.add_option("--pjobs", dest="ros_parallel_jobs", type="int",
                          action="store", help="Override ROS_PARALLEL_JOBS environment variable with this number of jobs.")
        parser.add_option("--threads", dest="threads", type="int", default = os.environ.get("ROSMAKE_THREADS", parallel_build.num_cpus()),
                          action="store", help="Build up to N packages in parallel")
        parser.add_option("--profile", dest="print_profile", default=False,
                          action="store_true", help="print time profile after build")
        parser.add_option("--skip-blacklist", dest="skip_blacklist", 
                          default=False, action="store_true", 
                          help="skip packages containing a file called ROS_BUILD_BLACKLIST (Default behavior will ignore the presence of ROS_BUILD_BLACKLIST)")
        parser.add_option("--skip-blacklist-osx", dest="skip_blacklist_osx", 
                          default=False, action="store_true", 
                          help="deprecated option. it will do nothing, please use platform declarations and --require-platform instead")

        parser.add_option("--rosdep-install", dest="rosdep_install",
                          action="store_true", help="call rosdep install before running")
        parser.add_option("--rosdep-yes", dest="rosdep_yes",
                          action="store_true", help="call rosdep install with default yes argument")
        parser.add_option("--no-rosdep", dest="rosdep_disabled",
                          action="store_true", help="disable the default check of rosdep")

        parser.add_option("--require-platform", dest="obey_whitelist",
                          action="store_true", help="do not build a package unless it is marked as supported on this platform")
        parser.add_option("--require-platform-recursive", dest="obey_whitelist_recursively",
                          action="store_true", help="do not build a package unless it is marked as supported on this platform, and all dependents are also marked")
        

        options, args = parser.parse_args()

        testing = False
        building = True
        if options.test_only:
            testing = True
            building = False
        elif options.test:
            testing = True

        if options.ros_parallel_jobs:
            self.ros_parallel_jobs = options.ros_parallel_jobs

        self.robust_build = options.robust
        self.best_effort = options.best_effort
        self.threads = options.threads
        self.skip_blacklist = options.skip_blacklist
        if options.skip_blacklist_osx:
            self.print_all("Option --skip-blacklist-osx is deprecated. It will do nothing, please use platform declarations and --require-platform instead");
        self.logging_enabled = options.logging_enabled
        if options.obey_whitelist or options.obey_whitelist_recursively:
            self.obey_whitelist = True
            if options.obey_whitelist_recursively:
                self.obey_whitelist_recursively = True

        # pass through verbosity options
        self.full_verbose = options.full_verbose
        self.verbose = options.verbose

        packages = []
        #load packages from arguments
        if options.build_all:
            packages = roslib.packages.list_pkgs()
            self.print_all( "Building all packages")
        else:      # no need to extend if all already selected   
            if options.buildtest:
              for p in options.buildtest:
                packages.extend(roslib.rospack.rospack_depends_on(p)) 
                self.print_all( "buildtest requested for package %s adding it and all dependent packages: "%p)

            if options.buildtest1:
              for p in options.buildtest1:
                packages.extend(roslib.rospack.rospack_depends_on_1(p)) 
                self.print_all( "buildtest1 requested for package %s adding it and all depends-on1 packages: "%p)

        if len(packages) == 0 and len(args) == 0:
            p = os.path.basename(os.path.abspath('.'))
            try:
              if (os.path.samefile(roslib.packages.get_pkg_dir(p), '.')):
                packages = [p]
                self.print_all( "No package specified.  Building %s"%packages)
              else:
                self.print_all("No package selected and the current directory is not the correct path for package '%s'."%p)
                
            except roslib.packages.InvalidROSPkgException, ex:
              try:
                if (roslib.stacks.get_stack_dir(p) == os.path.abspath('.')):
                  packages = [p]
                  self.print_all( "No package specified.  Building stack %s"%packages)
                else:
                  self.print_all("No stack selected and the current directory is not the correct path for stack '%s'."%p)
                  
              except roslib.stacks.InvalidROSStackException, ex2:

                self.print_all("No package or stack specified.  And current directory '%s' is not a package name or stack name."%p)
                #sys.exit(-1)
        else:
            packages.extend(args)

        if not self.is_rosout_built():
            packages.append("rosout")
            self.print_all("Detected rosout not built, adding it to the build")

        self.print_all( "Packages requested are: %s"%packages)
        

        # Setup logging
        if self.logging_enabled:
          date_time_stamp =  "rosmake_output-" + time.strftime("%Y%m%d-%H%M%S")
          if options.output_dir:
              #self.log_dir = os.path.join(os.getcwd(), options.output_dir, date_time_stamp);
              self.log_dir = os.path.abspath(options.output_dir)
          else:
              self.log_dir = os.path.join(roslib.rosenv.get_ros_home(), "rosmake", date_time_stamp);

          self.print_all("Logging to directory")
          self.print_all("%s"%self.log_dir)
          if os.path.exists (self.log_dir) and not os.path.isdir(self.log_dir):
              self.print_all( "Log destination %s is a file; please remove it or choose a new destination"%self.log_dir)
              sys.exit(1)
          if not os.path.exists (self.log_dir):
              os.makedirs (self.log_dir)


        (specified_packages, rejected_packages) = roslib.stacks.expand_to_packages(packages)
        self.print_all("Expanded args %s to:\n%s"%(packages, specified_packages))
        if rejected_packages:
            self.print_all("WARNING: The following args could not be parsed as stacks or packages: %s"%rejected_packages)
        if len(specified_packages) == 0:
            self.print_all("ERROR: No arguments could be parsed into valid package or stack names.")
            return False

        # make sure all dependencies are satisfied and if not warn
        if options.rosdep_install:
            self.install_rosdeps(specified_packages, options.rosdep_yes)
        elif not options.rosdep_disabled:
            self.check_rosdep(specified_packages)

        if options.unmark_installed:
            for p in specified_packages:
                if self.flag_tracker.remove_nobuild(p):
                    self.print_all("Removed ROS_NOBUILD from %s"%p)
            return True
            
        required_packages = specified_packages[:]
        # these packages are not in the dependency tree but are needed they only cost 0.01 seconds to build
        if "paramiko" not in specified_packages:
            required_packages.append("paramiko")
        if "pycrypto" not in specified_packages:
            required_packages.append("pycrypto")
    
        #generate the list of packages necessary to build(in order of dependencies)
        counter = 0
        for p in required_packages:

            counter = counter + 1
            self.print_verbose( "Processing %s and all dependencies(%d of %d requested)"%(p, counter, len(packages)))
            self.build_or_recurse(p)

        # remove extra packages if specified-only flag is set
        if options.specified_only:
          new_list = []
          for pkg in self.build_list:
            if pkg in specified_packages:
              new_list.append(pkg)
              self.dependency_tracker = parallel_build.DependencyTracker(specified_packages) # this will make the tracker only respond to packages in the list
        
          self.print_all("specified-only option was used, only building packages %s"%new_list)
          self.build_list = new_list

        if options.pre_clean:
          build_queue = parallel_build.BuildQueue(self.build_list, self.dependency_tracker, robust_build = True)
          self.parallel_build_pkgs(build_queue, "clean", threads = options.threads)
          if "rospack" in self.build_list:
              self.print_all( "Rosmake detected that rospack was requested to be cleaned.  Cleaning it for it was skipped earlier.")
              subprocess.check_call(["make", "-C", os.path.join(os.environ["ROS_ROOT"], "tools/rospack"), "clean"])


        if building:
            self.assert_rospack_built()

        build_passed = True
        if building:
          self.print_verbose ("Building packages %s"% self.build_list)
          build_queue = parallel_build.BuildQueue(self.build_list, self.dependency_tracker, robust_build = options.robust or options.best_effort)
          build_passed = self.parallel_build_pkgs(build_queue, options.target, threads = options.threads)
          if "rospack" in self.build_list and options.target == "clean":
              self.print_all( "Rosmake detected that rospack was requested to be cleaned.  Cleaning it, because it was skipped earlier.")
              subprocess.check_call(["make", "-C", os.path.join(os.environ["ROS_ROOT"], "tools/rospack"), "clean"])

        tests_passed = True
        if build_passed and testing:
            self.print_verbose ("Testing packages %s"% packages)
            build_queue = parallel_build.BuildQueue(specified_packages, parallel_build.DependencyTracker(specified_packages), robust_build = True)
            tests_passed = self.parallel_build_pkgs(build_queue, "test", threads = 1)

        if  options.mark_installed:
            if build_passed and tests_passed: 
                for p in specified_packages:
                    if self.flag_tracker.add_nobuild(p):
                        self.print_all("Marking %s as installed with a ROS_NOBUILD file"%p)
            else:
                self.print_all("All builds and tests did not pass cannot mark packages as installed. ")


        self.finish_time = time.time() #note: before profiling
        self.generate_summary_output(self.log_dir)
        
        if options.print_profile:
            self.print_all (self.get_profile_string())

        return build_passed and tests_passed


