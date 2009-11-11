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

import parallel_build

from optparse import OptionParser

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
        self.ros_parallel_jobs = parallel_build.num_cpus()
        self.build_list = []
        self.start_time = time.time()
        self.log_dir = ""
        self.logging_enabled = True

    def num_packages_built(self):
        return len(self.result[argument].keys())

    def get_path(self, package):
        if not package in self.paths:
            self.paths[package] = roslib.packages.get_pkg_dir(package)
        return self.paths[package]
        
    def check_rosdep(self, packages):
        cmd = ["rosdep", "check"]
        cmd.extend(packages)
        try:
            command_line = subprocess.Popen(cmd, stdout=subprocess.PIPE,  stderr=subprocess.PIPE)
            (pstd_out, pstd_err) = command_line.communicate()
            if len(pstd_out) !=0:
                self.print_all("------------------------\n Warning: rosdep check not satisfied for these packages\n------------------------\n %s\n------------------------"%(pstd_out))
            else:
                self.print_full_verbose("rosdep satisfied")
        except OSError:
            self.print_verbose("Failed to invoke rosdep, it's likely not built")

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


    def build(self, p, argument = None, robust_build=False):
        return_string = ""
        try:
            local_env = os.environ.copy()
            local_env['ROS_PARALLEL_JOBS'] = "-j%d" % self.ros_parallel_jobs
            local_env['SVN_CMDLINE'] = "svn --non-interactive"
            cmd = ["make", "-C", self.get_path(p) ]
            if argument:
                cmd.append(argument)
            self.print_full_verbose (cmd)

            if p == "rospack" and argument == "clean":
                return_string = ("[SKIP] rosmake will not clean rospack. rosmake cannot operate without it.")
                return (False, return_string)
            # warn if ROS_BUILD_BLACKLIST encountered if applicable
            if not self.skip_blacklist and self.flag_tracker.is_blacklisted(p):
              self.print_all ("!"*20 + " ROS_BUILD_BLACKLIST ENCOUNTERED in package: %s or one of its dependents --- TRYING TO BUILD ANYWAY"%p + "!"*20)

            if self.skip_blacklist and self.flag_tracker.is_blacklisted(p):
                self.result[argument][p] = True
                return_string =  ("[SKIP] due to ROS_BUILD_BLACKLIST")
                self.output[argument][p] = "ROS_BUILD_BLACKLIST"
            elif self.skip_blacklist_osx and self.flag_tracker.is_blacklisted_osx(p):
                self.result[argument][p] = True
                return_string =  ("[SKIP] due to ROS_BUILD_BLACKLIST_OSX")
                self.output[argument][p] = "ROS_BUILD_BLACKLIST_OSX"
            elif self.flag_tracker.has_nobuild(p):
                self.result[argument][p] = True
                return_string =  ("[SKIP] due to ROS_NOBUILD")
                self.output[argument][p] = "ROS_NOBUILD"
            elif not self.flag_tracker.has_makefile(p):
                self.result[argument][p] = True
                return_string =  ("[SKIP] due do to no Makefile")
                self.output[argument][p] = "No Makefile Present"
            else:
                start_time = time.time()
                command_line = subprocess.Popen(cmd, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT, env=local_env)
                (pstd_out, pstd_err) = command_line.communicate() # pstd_err should be None due to pipe above
                self.profile[argument][p] = time.time() - start_time
                self.output[argument][p] = pstd_out
                if argument:
                    log_type = "build_%s"%argument
                else:
                    log_type = "build"
                if not command_line.returncode:
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
                    self.result[argument][p] = no_target
                    if self.robust_build or interrupt:
                      self.print_verbose( pstd_out)
                    else:
                      self.print_tail( pstd_out)
                    self.output_to_file(p, log_type, pstd_out, always_print= not (no_target or interrupt))

                    return (False, return_string)
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
                
                    
            output = output + "%3d: %s in %.2f %s in %.2f --- %s\n"% (count, build_results[build_result], build_time, test_results[test_result], test_time, key)
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

    def main(self):
        parser = OptionParser(usage="usage: %prog [options]", prog='rosmake')
        parser.add_option("--test-only", dest="test_only", default=False,
                          action="store_true", help="only run tests")
        parser.add_option("-t", dest="test", default=False,
                          action="store_true", help="build and test packages")
        parser.add_option("-a", "--all", dest="build_all", default=False,
                          action="store_true", help="select all packages")
        parser.add_option("-v", dest="verbose", default=False,
                          action="store_true", help="display errored builds")
        parser.add_option("-r","-k", "--robust", dest="robust", default=False,
                           action="store_true", help="do not stop build on error")
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
                          action="store", help="run make with this N jobs '-j=N'")
        parser.add_option("--threads", dest="threads", type="int", default = parallel_build.num_cpus(),
                          action="store", help="Build up to N packages in parallel")
        parser.add_option("--profile", dest="print_profile", default=False,
                          action="store_true", help="print time profile after build")
        parser.add_option("--skip-blacklist", dest="skip_blacklist", 
                          default=False, action="store_true", 
                          help="skip packages containing a file called ROS_BUILD_BLACKLIST (Default behavior will ignore the presence of ROS_BUILD_BLACKLIST)")
        parser.add_option("--skip-blacklist-osx", dest="skip_blacklist_osx", 
                          default=False, action="store_true", 
                          help="skip packages containing a file called ROS_BUILD_BLACKLIST_OSX (Default behavior will ignore the presence of ROS_BUILD_BLACKLIST_OSX)")

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
        self.threads = options.threads
        self.skip_blacklist = options.skip_blacklist
        self.skip_blacklist_osx = options.skip_blacklist_osx
        self.logging_enabled = options.logging_enabled

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
            self.print_all( "Packages requested are: %s"%packages)
                    


        # Setup logging
        if self.logging_enabled:
          date_time_stamp =  "rosmake_output-" + time.strftime("%Y%m%d-%H%M%S")
          if options.output_dir:
              #self.log_dir = os.path.join(os.getcwd(), options.output_dir, date_time_stamp);
              self.log_dir = os.path.abspath(options.output_dir)
          else:
              self.log_dir = os.path.join(roslib.rosenv.get_ros_home(), date_time_stamp);

          self.print_all("Logging to directory")
          self.print_all("%s"%self.log_dir)
          if os.path.exists (self.log_dir) and not os.path.isdir(self.log_dir):
              self.print_all( "Log destination %s is a file; please remove it or choose a new destination"%self.log_dir)
              sys.exit(1)
          if not os.path.exists (self.log_dir):
              os.makedirs (self.log_dir)


        counter = 0
        verified_packages = []
        for p in packages:
          try:
            roslib.packages.get_pkg_dir(p)
            verified_packages.append(p)
          except roslib.packages.InvalidROSPkgException, ex:
            try: 
              roslib.stacks.get_stack_dir(p)
              packages_in_stack = roslib.stacks.packages_of(p)
              verified_packages.extend(packages_in_stack)
              self.print_all("Found stack %s.  Expanding to packages: %s"%(p, packages_in_stack))
            except roslib.stacks.InvalidROSStackException, ex2:
              self.print_all("Could not resolve %s as a package or as a stack [ %s ] [ %s ]"%(p, ex, ex2))
            
        # make sure all dependencies are satisfied and if not warn
        self.check_rosdep(verified_packages)

        #generate the list of packages necessary to build(in order of dependencies)
        for p in verified_packages:

            counter = counter + 1
            self.print_verbose( "Processing %s and all dependencies(%d of %d requested)"%(p, counter, len(packages)))
            self.build_or_recurse(p)

        # remove extra packages if specified-only flag is set
        if options.specified_only:
          new_list = []
          for pkg in self.build_list:
            if pkg in verified_packages:
              new_list.append(pkg)
              self.dependency_tracker = parallel_build.DependencyTracker(verified_packages) # this will make the tracker only respond to packages in the list
        
          self.print_all("specified-only option was used, only building packages %s"%new_list)
          self.build_list = new_list

        if options.pre_clean:
          build_queue = parallel_build.BuildQueue(self.build_list, self.dependency_tracker, robust_build = True)
          self.parallel_build_pkgs(build_queue, "clean", threads = options.threads)

        build_passed = True
        if building:
          self.print_verbose ("Building packages %s"% self.build_list)
          build_queue = parallel_build.BuildQueue(self.build_list, self.dependency_tracker, robust_build = options.robust)
          build_passed = self.parallel_build_pkgs(build_queue, options.target, threads = options.threads)

        tests_passed = True
        if build_passed and testing:
            self.print_verbose ("Testing packages %s"% packages)
            build_queue = parallel_build.BuildQueue(self.build_list, self.dependency_tracker, robust_build = True)
            tests_passed = self.parallel_build_pkgs(build_queue, "test", threads = 1)

        self.finish_time = time.time() #note: before profiling
        self.generate_summary_output(self.log_dir)
        
        if options.print_profile:
            self.print_all (self.get_profile_string())

        return build_passed and tests_passed


