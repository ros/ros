#! /usr/bin/env python

# Copyright (c) 2008, Willow Garage, Inc.
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

from __future__ import with_statement

import os
import time
import sys, string
import subprocess
import roslib
import roslib.scriptutil

from optparse import OptionParser


def vdmain():
    parser = OptionParser(usage="usage: %prog [options]", prog='audit_stack')
    parser.add_option("-d", dest="check_deps", default=False,
                      action="store_true", help="Make sure dependencies of packages are ok")

    options, args = parser.parse_args()
    if len(args) != 1:
        print "Please enter a stack name as an argument"
        exit(-1)
    else:
        stack = args[0]

    # find all deps1 packages
    stack_contents = roslib.scriptutil.rosstackexec(["contents", stack]).split()
    dependent_packages = []
    for pkg in stack_contents:
        dependent_packages.extend(roslib.scriptutil.rospack_depends_1(pkg))
    dependent_packages = list(set(dependent_packages))
    #print "Stack contents:\n", stack_contents

    # find all deps1 stacks
    stack_deps = roslib.scriptutil.rosstack_depends_1(stack)
    #print "Stack dependencies", stack_deps

    deps1_contents = stack_contents[:]
    for stk in stack_deps:
        deps1_contents.extend(roslib.scriptutil.rosstackexec(["contents", stk]).split())
    deps1_contents = list(set(deps1_contents))
    #print "Packages in deps1 list", deps1_contents

    # check that all deps1 packages are in deps1 stacks or local
    found_problem = False
    for pkg in dependent_packages:
        if pkg not in deps1_contents:
            print "Package %s is not contained in packages in stack dependency list"%(pkg)
            found_problem = True

    if not found_problem:
        print "Done checking stack/package dependency consistency"
    else:
        print "Package dependencies not all contained in stack dependencies!"
        print "Package dependencies:\n%s"%dependent_packages
        print "Stack dependencies:\n%s"%deps1_contents

if __name__ == '__main__':
    start_time = time.time()
    vdmain()
    print "Elapsed Time: %.2f seconds."%(time.time() - start_time)
