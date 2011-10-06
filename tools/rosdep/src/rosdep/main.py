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

from __future__ import print_function

import roslib; roslib.load_manifest("rosdep")
import roslib.stacks

import rospkg

import sys
import os

import rosdep.core as core

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
    parser.add_option("-r", dest="robust", default=False, 
                      action="store_true", help="Continue installing despite errors.")
    parser.add_option("-a", "--all", dest="rosdep_all", default=False, 
                      action="store_true", help="select all packages")

    options, args = parser.parse_args()


    if len(args) == 0:
        parser.error("Please enter a command")
    command = args[0]
    if not command in _commands:
        parser.error("Unsupported command %s."%command)
    if len(args) < 2 and not options.rosdep_all:
        parser.error("Please enter arguments for '%s'"%command)
    rdargs = args[1:]

    verified_packages = []


    if not (command == "what_needs" or command == "where_defined" ): # package mode
        if options.rosdep_all:
            rdargs = rospack.RosPack().list()
        
        (verified_packages, rejected_packages) = roslib.stacks.expand_to_packages(rdargs)
        valid_stacks = [s for s in roslib.stacks.list_stacks() if s in rdargs]
    
        if len(rejected_packages) > 0:
            print("Warning: could not identify %s as a package"%rejected_packages)
        if len(verified_packages) == 0 and len(valid_stacks) == 0:
            parser.error("No Valid Packages or stacks listed as arguments")
                
    else: # rosdep as argumets 
        if options.rosdep_all:
            parser.error("-a, --all is not a valid option for this command")

    ### Find all dependencies
    try:
        r = core.Rosdep(verified_packages, robust=options.robust)
    except roslib.os_detect.OSDetectException as ex:
        print("rosdep ABORTING.  Failed to detect OS: %s"%ex)
        return 1

    except roslib.exceptions.ROSLibException as ex:
        print("rosdep ABORTING: %s"%ex)
        return 1

    if options.verbose:
        print("Detected OS: " + r.osi.get_name())
        print("Detected Version: " + r.osi.get_version())

    try:
        if command == "generate_bash" or command == "satisfy":
            missing_packages = r.satisfy()
            if not missing_packages:
                return 0
            else:
                print("The following rosdeps are not installed but are required", missing_packages)
                return 1
        
        elif command == "install":
            error = r.install(options.include_duplicates, options.default_yes)
            if error:
                print("rosdep install ERROR:\n%s"%error, file=sys.stderr)
                return 1
            else:
                print("All required rosdeps installed successfully")
                return 0
    except core.RosdepException as e:
        print("ERROR: %s"%e, file=sys.stderr)
        return 1

    try:
        if command == "depdb":
            print(r.depdb(verified_packages))
            return 0

        elif command == "what_needs":
            print('\n'.join(r.what_needs(rdargs)))
            return 0

        elif command == "where_defined":
            print(r.where_defined(rdargs))
            return 0

        elif command == "check":
            return_val = 0
            missing_packages = r.check()
            if len(rejected_packages) > 0:
                print("Arguments %s are not packages"%rejected_packages, file=sys.stderr)
                return_val = 1
            if len(missing_packages) == 0:
                print("All required rosdeps are installed")
                return 0
            else:
                print("The following rosdeps were not installed", missing_packages)
                return 1

    except core.RosdepException as e:
        print(str(e), file=sys.stderr)
        return 1
