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

from __future__ import print_function

import os
import re
import sys
import subprocess
import time
import roslib
import roslib.rospack
import roslib.rosenv
import roslib.stacks
import threading
import traceback
import signal

try: 
    from exceptions import SystemExit #Python 2.x
except ImportError: 
    pass #Python 3.x (in Python 3, 'exceptions' is always imported) 

from operator import itemgetter

from . import parallel_build
from . import package_stats

from optparse import OptionParser

import rosdep

def make_command():
    """
    @return: name of 'make' command
    @rtype: str
    """
    return os.environ.get("MAKE", "make")

class Printer:
   # storage for the instance reference
    __instance = None

    def __init__(self):
        """ Create singleton instance """
        # Check whether we already have an instance
        if Printer.__instance is None:
            # Create and remember instance
            Printer.__instance = Printer.__impl()

        # Store instance reference as the only member in the handle
        self.__dict__['_Printer__instance'] = Printer.__instance

    def __getattr__(self, attr):
        """ Delegate access to implementation """
        return getattr(self.__instance, attr)

    def __setattr__(self, attr, value):
        """ Delegate access to implementation """
        return setattr(self.__instance, attr, value)

    class __impl(threading.Thread):
        def __init__(self):
            threading.Thread.__init__(self)
            self.build_queue = None
            self.condition = threading.Condition()
            self.running = True
            self.done = False
            self.status = ""
            self.verbose = False
            self.full_verbose = False
            self.duration = 1./10.
            self._last_status = None

            # Rosmake specific data
            self.cache_argument = None
            self.cache_right = ''
            self.pkg_start_times = {}

        def shutdown(self):
            self.running = False
            cycles = 10
            for i in range(0,cycles):# sleep for at least 2 cycles of the status testing 'cycles' times
                if self.done:
                    #print "SUCCESSFULLY SHUTDOWN"
                    return True
                #print "Sleeping for %f FOR SHUTDOWN.  %d threads running"%(max(self.duration/cycles*2, 0.01), threading.activeCount())
                time.sleep(max(self.duration, 0.1)/cycles*2) 
            raise Exception("Failed to shutdown status thread in %.2f seconds"%(self.duration * 2))

        def __enter__(self):
            self.start()
        def __exit__(self, mtype, value, tb):
            self.shutdown()
            if value:
                if not mtype == type(SystemExit()):
                    traceback.print_exception(mtype, value, tb)
                else:
                    sys.exit(value)

        def run(self):
            while self.running:
                #shutdown if duration set to zero
                if self.duration <= 0:
                    self.running = False
                    break
                self.set_status_from_cache()
                if len(self.pkg_start_times.keys()) > 0:
                    n = self.terminal_width() - len(self.status)
                    status = self.status
                    if n > 0:
                        status = " "*n + self.status
                    if status != self._last_status:
                        self._print_status("%s"%status)
                        self._last_status = status
                time.sleep(self.duration) 
            self.done = True
            #print "STATUS THREAD FINISHED"

        def rosmake_cache_info(self, argument, start_times, right):
            self.cache_argument = argument
            self.pkg_start_times = start_times
            self.cache_right = right

        def rosmake_pkg_times_to_string(self, start_times):
            threads = []
            for p, t in sorted(start_times.items(), key=itemgetter(1)): #py3k
                threads.append("[ %s: %.1f sec ]"%(p, time.time() - t))

            return " ".join(threads)

        def set_status_from_cache(self):
            if self.cache_argument:
                self.set_status("[ make %s ] "%self.cache_argument + self.rosmake_pkg_times_to_string(self.pkg_start_times), self.cache_right)
            else:
                self.set_status("[ make ] " + self.rosmake_pkg_times_to_string(self.pkg_start_times), self.cache_right)

        def set_status(self, left, right = ''):
            header = "[ rosmake ] "
            h = len(header)
            l = len(left)
            r = len(right)
            w = self.terminal_width()
            if l + r < w - h:
                padding = w - h - l - r
                self.status = header + left + " "*padding + right
            else:
                self.status = header + left[:(w - h - r - 4)] + "... " + right

        def print_all(self, s, thread_name=None):
            if thread_name is None:
                str = "[ rosmake ] %s"%s
                

            else:
                str = "[rosmake-%s] %s"%(thread_name, s)
            sys.stdout.write(self.pad_str_to_width(str, self.terminal_width())+"\n")
            sys.stdout.flush()

        def print_verbose(self, s, thread_name=None):
            if self.verbose or self.full_verbose:
                self.print_all(s, thread_name=thread_name)

        def print_full_verbose(self, s):
            if self.full_verbose:
                print("[ rosmake ] %s"%(s))

        def print_tail(self, s, tail_lines=40):
            lines = s.splitlines()
            if self.full_verbose:
                tail_lines = len(lines)

            num_lines = min(len(lines), tail_lines)
            if num_lines == tail_lines:
                print("[ rosmake ] Last %d lines"%(num_lines))
            else:
                print("[ rosmake ] All %d lines"%(num_lines))
            print("{" + "-"*79)
            for l in range(-num_lines, -1):
                print("  %s"%(lines[l]))
            print("-"*79 + "}")

        def _print_status(self, s):
            sys.stdout.write("%s\r"%(s))
            sys.stdout.flush()

        @staticmethod
        def terminal_width():
            """Estimate the width of the terminal"""
            width = 0
            try:
                import struct, fcntl, termios
                s = struct.pack('HHHH', 0, 0, 0, 0)
                x = fcntl.ioctl(1, termios.TIOCGWINSZ, s)
                width = struct.unpack('HHHH', x)[1]
            except IOError:
                pass
            if width <= 0:
                try:
                    width = int(os.environ['COLUMNS'])
                except:
                    pass
            if width <= 0:
                width = 80

            return width

        @staticmethod
        def pad_str_to_width(str, width):
            """ Pad the string to be terminal width"""
            length = len(str)
            excess = 0
            if length < width:
                excess = width - length
            return str + " "* excess



class RosMakeAll:
    def __init__(self):
        self._result_lock = threading.Lock()
        self.printer = Printer()
        self.result = {}
        self.paths = {}
        self.dependency_tracker = parallel_build.DependencyTracker()
        self.flag_tracker = package_stats.PackageFlagTracker(self.dependency_tracker)
        self.output = {}
        self.profile = {}
        self.ros_parallel_jobs = 0
        self.build_list = []
        self.start_time = time.time()
        self.log_dir = ""
        self.logging_enabled = True
        self.obey_whitelist = False
        self.obey_whitelist_recursively = False

        self.rosdep_install_result = None
        self.rosdep_check_result = None


    def num_packages_built(self):
        """
        @return: number of packages that were built
        @rtype: int
        """
        return len(list(self.result[argument].keys())) #py3k

    def update_status(self, argument, start_times, right):
        self.printer.rosmake_cache_info(argument, start_times, right)

    def get_path(self, package):
        if not package in self.paths:
            self.paths[package] = roslib.packages.get_pkg_dir(package)
        return self.paths[package]
        
    def check_rosdep(self, packages):
        failed_rosdeps = []
        try:
            r = rosdep.core.Rosdep(packages, robust=True)
            failed_rosdeps = r.check()
        except roslib.exceptions.ROSLibException as ex:
            return ("rosdep ABORTED: %s"%ex, '')

        return ', '.join(failed_rosdeps)
        
    def install_rosdeps(self, packages, default_yes):
        """
        Install all rosdeps of packages.
        @param packages: list of package name
        @type  packages: [str]
        @param default_yes: if True, assume 'yes' to all package manager prompts.
        @type  default_yes: bool
        """
        try:
            r = rosdep.core.Rosdep(packages, robust=True)
            error = r.install(include_duplicates=False, default_yes=default_yes)
            if not error:
                return None
            else:
                return "%s"%error
        except rosdep.core.RosdepException as e:
            return "%s"%e
        except roslib.exceptions.ROSLibException as ex:
            return "%s"%ex

    def build_or_recurse(self,p):
        if p in self.build_list:
            return
        for d in self.dependency_tracker.get_deps_1(p):
            self.build_or_recurse(d)
        try: # append it ot the list only if present
          self.get_path(p)
          self.build_list.append(p)
        except roslib.packages.InvalidROSPkgException as ex:
          if not self.robust_build:
            self.printer.print_all("Exiting due to missing package: %s"%ex)
            sys.exit(-1)
          else:
            self.printer.print_all("!"*20 + " Package %s does not exist. %s"%(p, ex) + "!"*20)


    def parallel_build_pkgs(self, build_queue, argument = None, threads = 1):
        self.profile[argument] = {}
        self.output[argument] = {}
        with self._result_lock:
            if argument not in self.result.keys():
                self.result[argument] = {}

        cts = []
        for i in range(0, threads):
          ct = parallel_build.CompileThread(str(i), build_queue, self, argument)
          #print "TTTH starting thread ", ct
          ct.start()
          cts.append(ct)
        for ct in cts:
          try:
            #print "TTTT Joining", ct
            ct.join()
            #print "TTTH naturally ended thread", ct
          except KeyboardInterrupt:
            self.printer.print_all( "TTTH Caught KeyboardInterrupt. Stopping build.")
            build_queue.stop()
            ct.join()
          except: #catch all
              self.printer.print_all("TTTH OTHER exception thrown!!!!!!!!!!!!!!!!!!!!!")
              ct.join()
        #print "All threads joined"
        all_pkgs_passed = True
        with self._result_lock:
            for v in self.result[argument].values():
                all_pkgs_passed = v and all_pkgs_passed

        build_passed = build_queue.succeeded() and all_pkgs_passed
        return build_passed

    # This function taken from
    # http://www.chiark.greenend.org.uk/ucgi/~cjwatson/blosxom/2009-07-02-python-sigpipe.html
    def _subprocess_setup(self):
        # Python installs a SIGPIPE handler by default. This is usually not
        # what non-Python subprocesses expect.
        signal.signal(signal.SIGPIPE, signal.SIG_DFL)

    def _build_package(self, package, argument=None):
        """
        Lower-level routine for building a package. Handles execution of actual build command.
        @param package: package name
        @type  package: str
        """
        local_env = os.environ.copy()
        if self.ros_parallel_jobs > 0:
            local_env['ROS_PARALLEL_JOBS'] = "-l%d" % self.ros_parallel_jobs
        elif "ROS_PARALLEL_JOBS" not in os.environ: #if no environment setup and no args fall back to # cpus
            local_env['ROS_PARALLEL_JOBS'] = "-l%d" % parallel_build.num_cpus()
        local_env['SVN_CMDLINE'] = "svn --non-interactive"
        cmd = ["bash", "-c", "cd %s && %s "%(self.get_path(package), make_command()) ] #UNIXONLY
        if argument:
            cmd[-1] += argument
        self.printer.print_full_verbose (cmd)
        command_line = subprocess.Popen(cmd, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT, env=local_env, preexec_fn=self._subprocess_setup)
        (pstd_out, pstd_err) = command_line.communicate() # pstd_err should be None due to pipe above
        return (command_line.returncode, pstd_out)

    def build(self, p, argument = None, robust_build=False):
        """
        Build package
        @param p: package name
        @type  p: str
        """
        return_string = ""
        try:
            if p == "rospack":
                return_string = ("[SKIP] rosmake uses rospack.  If building it is already built, if cleaning it will be cleaned at the end.")
                return (True, return_string) # This will be caught later
            # warn if ROS_BUILD_BLACKLIST encountered if applicable

            # do not build packages for which the build has failed
            if argument == "test":  # Tests are not build dependent
                failed_packages = []
            else:
                with self._result_lock:
                    failed_packages = [j for j in self.result[argument] if not self.result[argument][j] == True]

            (buildable, error, why) = self.flag_tracker.can_build(p, self.obey_whitelist, self.obey_whitelist_recursively, self.skip_blacklist, failed_packages)
            if buildable or self.robust_build:
                start_time = time.time()
                (returncode, pstd_out) = self._build_package(p, argument)
                self.profile[argument][p] = time.time() - start_time
                self.output[argument][p] = pstd_out
                if argument:
                    log_type = "build_%s"%argument
                else:
                    log_type = "build"
                if not returncode:
                    self.printer.print_full_verbose( pstd_out)
                    with self._result_lock:
                        self.result[argument][p] = True

                    num_warnings = len(re.findall("warning:", pstd_out))
                    if num_warnings > 0:
                        return_string =  ("[PASS] [ %.2f seconds ] -- WARNING: %d compiler warnings"%(self.profile[argument][p], num_warnings))
                    else:
                        return_string =  ("[PASS] [ %.2f seconds ]"%( self.profile[argument][p]))
                    self.output_to_file(p, log_type, pstd_out, num_warnings > 0)
                else:
                    success = False
                    no_target = len(re.findall("No rule to make target", pstd_out)) > 0
                    interrupt = len(re.findall("Interrupt", pstd_out)) > 0
                    if no_target:
                        return_string = ( "[SKIP] No rule to make target %s"%( argument))
                        success = True
                    elif interrupt:
                        return_string = ("[Interrupted]" )
                    else:
                        return_string = ( "[FAIL] [ %.2f seconds ]"%( self.profile[argument][p]))
                    with self._result_lock:
                        self.result[argument][p] = True if no_target else False

                    if success == False: #don't print tail if [SKIP] target
                        self.printer.print_tail( pstd_out)
                    self.output_to_file(p, log_type, pstd_out, always_print= not (no_target or interrupt))

                    return (success, return_string)
            else:
                with self._result_lock:
                    self.result[argument][p] = error

                return_string += why
                return(error, return_string)
            return (True, return_string) # this means that we didn't error in any case above
        except roslib.packages.InvalidROSPkgException as ex:
            with self._result_lock:
                self.result[argument][p] = False
            self.printer.print_verbose ("[SKIP] Package not found\n")
            self.output[argument][p] = "Package not found %s"%ex
            return (False, return_string)
            

    def output_to_file(self, package, log_type, stdout, always_print= False):
        if not self.logging_enabled:
            return
        package_log_dir = os.path.join(self.log_dir, package)

        std_out_filename = os.path.join(package_log_dir, log_type + "_output.log")
        if not os.path.exists (package_log_dir):
            roslib.rosenv.makedirs_with_parent_perms(package_log_dir)
        with open(std_out_filename, 'w') as stdout_file:
            stdout_file.write(stdout)
            print_string = "Output from build of package %s written to:\n[ rosmake ]    %s"%(package, std_out_filename)
            if always_print:
                self.printer.print_all(print_string)
            else:
                self.printer.print_full_verbose(print_string)

    def generate_summary_output(self, log_dir):
        if not self.logging_enabled:
            return

        self.printer.print_all("Results:")
        if 'clean' in self.result.keys():
            self.printer.print_all("Cleaned %d packages."%len(self.result['clean']))
        if None in self.result.keys():
            build_failure_count = len([p for p in self.result[None].keys() if self.result[None][p] == False])
            self.printer.print_all("Built %d packages with %d failures."%(len(self.result[None]), build_failure_count))
        if 'test' in self.result.keys():
            test_failure_count = len([p for p in self.result['test'].keys() if self.result['test'][p] == False])
            self.printer.print_all("Tested %d packages with %d failures."%(len(self.result['test']), test_failure_count))
        self.printer.print_all("Summary output to directory")
        self.printer.print_all("%s"%self.log_dir)
        if self.rosdep_install_result:
            self.printer.print_all("ERROR: Rosdep installation failed with error: %s"%self.rosdep_install_result)
        if self.rosdep_check_result:
            self.printer.print_all("WARNING: Rosdep did not detect the following system dependencies as installed: %s Consider using --rosdep-install option or `rosdep install %s`"%(self.rosdep_check_result, ' '.join(self.specified_packages)))
        if self.rejected_packages:
            self.printer.print_all("WARNING: Skipped command line arguments: %s because they could not be resolved to a stack name or a package name. "%self.rejected_packages)

                           

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
                
                    
            output = output + "%3d: %s in %.2f %s in %.2f --- %s\n"% (count, build_results[build_result], build_time , test_results[test_result], test_time, key)
            total = total + build_time
            count = count + 1

        elapsed_time  =  self.finish_time - self.start_time
        output = output + "----------------\n" + "%.2f Cumulative,  %.2f Elapsed, %.2f Speedup \n"%(total, elapsed_time, float(total) / float(elapsed_time))
        return output

    def assert_prebuild_built(self, ros_package_path_list, target=''):
        ret_val = True
        count = 0
        for pkg in ros_package_path_list:
            count = count + 1
            pkg_name = pkg.split('/')[-1]

            if self.flag_tracker.has_nobuild(pkg_name):
                ret_val &= True
            else:
                try:
                    self.printer.print_all("Starting >>> %s"%pkg)
                    self.update_status(target, {pkg:time.time()}, "%d/%d Bootstrap"%(count, len(ros_package_path_list)))
                    #Special Case: %s [ %s %s ] [ %d of %d ] "%(pkg_name, make_command(), target, count, len(ros_package_path_list)))
                    cmd = ["bash", "-c", "cd %s && %s %s"%(os.path.join(os.environ["ROS_ROOT"], pkg), make_command(), target)]
                    command_line = subprocess.Popen(cmd, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)
                    (pstd_out, pstd_err) = command_line.communicate() # pstd_err should be None due to pipe above

                    self.printer.print_verbose(pstd_out)
                    if command_line.returncode:
                        sys.stderr.write("Failed to build %s\n"%(pkg_name))
                        sys.stderr.write("Error:\n{{{\n%s\n}}}\n"%(pstd_out))
                        sys.exit(-1)
                    self.printer.print_all("Finished <<< %s"%pkg)
                except KeyboardInterrupt as ex:
                    self.printer.print_all("Keyboard interrupt caught in pkg %s"%pkg)
                    return False
        return True
            
        # The check for presence doesn't check for updates
        #if os.path.exists(os.path.join(os.environ["ROS_ROOT"], "bin/rospack")):
        #    return True
        #else:
        #    print "Rosmake detected that rospack was not built.  Building it for you because it is required."
        #    return subprocess.call(["make", "-C", os.path.join(os.environ["ROS_ROOT"], "tools/rospack")])



    def is_rosout_built(self):
        """
        @return: True if rosout package has been built
        @rtype: bool
        """
        return os.path.exists(os.path.join(roslib.packages.get_pkg_dir("rosout"), "rosout"))
            

    def main(self):
        """
        main command-line entrypoint
        """
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
        parser.add_option("--bootstrap", dest="bootstrap", default=False,
                          action="store_true", help="Do the bootstrap packages even if there are no arguments")
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
                          action="store_true", help="if calling rosdep install use the default yes argument")
        parser.add_option("--no-rosdep", dest="rosdep_disabled",
                          action="store_true", help="disable the default check of rosdep")

        parser.add_option("--require-platform", dest="obey_whitelist",
                          action="store_true", help="do not build a package unless it is marked as supported on this platform")
        parser.add_option("--require-platform-recursive", dest="obey_whitelist_recursively",
                          action="store_true", help="do not build a package unless it is marked as supported on this platform, and all dependents are also marked")
        parser.add_option("--status-rate", dest="status_update_rate",
                          action="store", help="How fast to update the status bar in Hz.  Default: 5Hz")
        

        options, args = parser.parse_args()


        # force a rebuild of the package cache at the top                
        cmd = ["rospack", "profile"]
        command_line = subprocess.Popen(cmd, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)
        (pstd_out, pstd_err) = command_line.communicate() # pstd_err should be None due to pipe above          
        # both above and below are necessary for "roscreate-pkg foo && rosmake foo" to work
        roslib.packages._invalidate_cache(roslib.packages._pkg_dir_cache)

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
            self.printer.print_all("Option --skip-blacklist-osx is deprecated. It will do nothing, please use platform declarations and --require-platform instead");
        self.logging_enabled = options.logging_enabled
        if options.obey_whitelist or options.obey_whitelist_recursively:
            self.obey_whitelist = True
            if options.obey_whitelist_recursively:
                self.obey_whitelist_recursively = True

        # pass through verbosity options
        self.printer.full_verbose = options.full_verbose
        self.printer.verbose = options.verbose
        if options.status_update_rate:
            if float(options.status_update_rate)> 0:
                self.printer.duration = 1.0/float(options.status_update_rate)
            else:
                self.printer.duration = 0
        packages = []
        #load packages from arguments
        if options.build_all:
            packages = roslib.packages.list_pkgs()
            self.printer.print_all( "Building all packages")
        else:      # no need to extend if all already selected   
            if options.buildtest:
              for p in options.buildtest:
                packages.extend(roslib.rospack.rospack_depends_on(p)) 
                self.printer.print_all( "buildtest requested for package %s adding it and all dependent packages: "%p)

            if options.buildtest1:
              for p in options.buildtest1:
                packages.extend(roslib.rospack.rospack_depends_on_1(p)) 
                self.printer.print_all( "buildtest1 requested for package %s adding it and all depends-on1 packages: "%p)

        if len(packages) == 0 and len(args) == 0:
            p = os.path.basename(os.path.abspath('.'))
            try:
              if (os.path.samefile(roslib.packages.get_pkg_dir(p), '.')):
                packages = [p]
                self.printer.print_all( "No package specified.  Building %s"%packages)
              else:
                self.printer.print_all("No package selected and the current directory is not the correct path for package '%s'."%p)
                
            except roslib.packages.InvalidROSPkgException as ex:
                try:
                    stack_dir = roslib.stacks.get_stack_dir(p)
                    if os.path.samefile(stack_dir, '.'):
                        packages = [p]
                        self.printer.print_all( "No package specified.  Building stack %s"%packages)
                    else:
                        self.printer.print_all("No package or stack arguments and the current directory is not the correct path for stack '%s'. Stack directory is: %s."%(p, roslib.stacks.get_stack_dir(p)))
                except:
                    self.printer.print_all("No package or stack specified.  And current directory '%s' is not a package name or stack name."%p)
        else:
            packages.extend(args)

        #if not self.is_rosout_built():
        #    packages.append("rosout")
        #    self.printer.print_all("Detected rosout not built, adding it to the build")

        self.printer.print_all( "Packages requested are: %s"%packages)
        

        # Setup logging
        if self.logging_enabled:
          date_time_stamp =  "rosmake_output-" + time.strftime("%Y%m%d-%H%M%S")
          if options.output_dir:
              #self.log_dir = os.path.join(os.getcwd(), options.output_dir, date_time_stamp);
              self.log_dir = os.path.abspath(options.output_dir)
          else:
              self.log_dir = os.path.join(roslib.rosenv.get_ros_home(), "rosmake", date_time_stamp);

          self.printer.print_all("Logging to directory%s"%self.log_dir)
          if os.path.exists (self.log_dir) and not os.path.isdir(self.log_dir):
              self.printer.print_all( "Log destination %s is a file; please remove it or choose a new destination"%self.log_dir)
              sys.exit(1)
          if not os.path.exists (self.log_dir):
              self.printer.print_verbose("%s doesn't exist: creating"%self.log_dir)
              roslib.rosenv.makedirs_with_parent_perms(self.log_dir)

          self.printer.print_verbose("Finished setting up logging")
        stacks_arguments = [s for s in packages if s in roslib.stacks.list_stacks()]
        (self.specified_packages, self.rejected_packages) = roslib.stacks.expand_to_packages(packages)
        self.printer.print_all("Expanded args %s to:\n%s"%(packages, self.specified_packages))
        if self.rejected_packages:
            self.printer.print_all("WARNING: The following args could not be parsed as stacks or packages: %s"%self.rejected_packages)
        if len(self.specified_packages) + len(stacks_arguments) == 0 and options.bootstrap == False:
            self.printer.print_all("ERROR: No arguments could be parsed into valid package or stack names.")
            self.printer.running = False
            return False

        if options.unmark_installed:
            for p in self.specified_packages:
                if self.flag_tracker.remove_nobuild(p):
                    self.printer.print_all("Removed ROS_NOBUILD from %s"%p)
            self.printer.running = False
            return True
            
        required_packages = self.specified_packages[:]

        # catch dependent packages which are inside of zero sized stacks #3528 
        # add them to required list but not the specified list. 
        for s in stacks_arguments:
            for d in roslib.rospack.rosstack_depends_1(s):
                required_packages.extend(roslib.stacks.packages_of(d))

        # deduplicate required_packages
        required_packages = list(set(required_packages))

        # make sure all dependencies are satisfied and if not warn
        buildable_packages = []
        for p in required_packages:
            (buildable, error, str) = self.flag_tracker.can_build(p, self.obey_whitelist, self.obey_whitelist_recursively, self.skip_blacklist, [], False)
            if buildable: 
                buildable_packages.append(p)

        if options.rosdep_install:
            self.printer.print_all("Generating Install Script using rosdep then executing. This may take a minute, you will be prompted for permissions. . .")
            self.rosdep_install_result = self.install_rosdeps(buildable_packages, options.rosdep_yes)
            if self.rosdep_install_result:
                self.printer.print_all( "rosdep install failed: %s"%self.rosdep_install_result)
                return False
            else:
                self.printer.print_all("rosdep successfully installed all system dependencies")


        elif not options.rosdep_disabled:
            self.printer.print_all("Checking rosdeps compliance for packages %s.  This may take a few seconds."%(', '.join(packages)))
            self.rosdep_check_result = self.check_rosdep(buildable_packages)

            if len(self.rosdep_check_result) == 0:
                self.printer.print_all( "rosdep check passed all system dependencies in packages")# %s"% packages)
            else:
                self.printer.print_all("rosdep check failed to find system dependencies: %s"% self.rosdep_check_result)

        #generate the list of packages necessary to build(in order of dependencies)
        counter = 0
        for p in required_packages:

            counter = counter + 1
            self.printer.print_verbose( "Processing %s and all dependencies(%d of %d requested)"%(p, counter, len(packages)))
            self.build_or_recurse(p)

        # remove extra packages if specified-only flag is set
        if options.specified_only:
          new_list = []
          for pkg in self.build_list:
            if pkg in self.specified_packages:
              new_list.append(pkg)
              self.dependency_tracker = parallel_build.DependencyTracker(self.specified_packages) # this will make the tracker only respond to packages in the list
        
          self.printer.print_all("specified-only option was used, only building packages %s"%new_list)
          self.build_list = new_list

        if options.pre_clean:
          build_queue = parallel_build.BuildQueue(self.build_list, parallel_build.DependencyTracker([]), robust_build = True)
          self.parallel_build_pkgs(build_queue, "clean", threads = options.threads)
          if "rospack" in self.build_list:
              self.printer.print_all( "Rosmake detected that rospack was requested to be cleaned.  Cleaning it for it was skipped earlier.")
              self.assert_prebuild_built(["tools/rospack"], 'clean')
              #subprocess.check_call(["make", "-C", os.path.join(os.environ["ROS_ROOT"], "tools/rospack"), "clean"])


        build_passed = True

        if building:
          #make sure required packages are built before continuing (These are required by internal functions
          prebuild_result = False
          if options.target == "clean":
              prebuild_result = self.assert_prebuild_built(["tools/rospack"])
          else:
              prebuild_result = self.assert_prebuild_built(["tools/rospack"])
          if not prebuild_result:
              self.printer.print_all("Failed to finish prebuild, aborting")
              self.printer.running = False
              return


          self.printer.print_verbose ("Building packages %s"% self.build_list)
          build_queue = parallel_build.BuildQueue(self.build_list, self.dependency_tracker, robust_build = options.robust or options.best_effort)
          build_queue.register_prebuilt(["rospack"])
          if None not in self.result.keys():
                self.result[None] = {}
          if 'rospack' in self.build_list:
              self.result[None]["rospack"] = True

          build_passed = self.parallel_build_pkgs(build_queue, options.target, threads = options.threads)
          if "rospack" in self.build_list and options.target == "clean":
              self.printer.print_all( "Rosmake detected that rospack was requested to be cleaned.  Cleaning it, because it was skipped earlier.")
              self.assert_prebuild_built(["tools/rospack"], 'clean')

        tests_passed = True
        if build_passed and testing:
            self.printer.print_verbose ("Testing packages %s"% packages)
            build_queue = parallel_build.BuildQueue(self.specified_packages, parallel_build.DependencyTracker(self.specified_packages), robust_build = True)
            tests_passed = self.parallel_build_pkgs(build_queue, "test", threads = 1)


        if  options.mark_installed:
            if build_passed and tests_passed: 
                for p in self.specified_packages:
                    if self.flag_tracker.add_nobuild(p):
                        self.printer.print_all("Marking %s as installed with a ROS_NOBUILD file"%p)
            else:
                self.printer.print_all("All builds and tests did not pass cannot mark packages as installed. ")


        self.finish_time = time.time() #note: before profiling
        self.generate_summary_output(self.log_dir)
        
        if options.print_profile:
            self.printer.print_all (self.get_profile_string())

        self.printer.running = False
        return build_passed and tests_passed


