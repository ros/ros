#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

"""
Prints summary of aggregated test results to stdout. This is useful
when running several tests across a package.
"""

from __future__ import print_function

import os
import sys
import cStringIO

import rospkg
import rosunit.junitxml as junitxml

def create_summary(result, packages):
    buff = cStringIO.StringIO()

    buff.write('-'*80+'\n')
    buff.write('\033[1m[AGGREGATED TEST RESULTS SUMMARY]\033[0m\n\n')

    errors_failures = [r for r in result.test_case_results if r.errors or r.failures]
    if errors_failures:
        buff.write('ERRORS/FAILURES:\n')
        for tc_result in errors_failures:
            buff.write(tc_result.description)

    buff.write("PACKAGES: \n%s\n\n"%'\n'.join([" * %s"%p for p in packages]))

    buff.write('\nSUMMARY\n')
    if (result.num_errors + result.num_failures) == 0:
        buff.write("\033[32m * RESULT: SUCCESS\033[0m\n")
    else:
        buff.write("\033[1;31m * RESULT: FAIL\033[0m\n")

    # TODO: still some issues with the numbers adding up if tests fail to launch

    # number of errors from the inner tests, plus add in count for tests
    # that didn't run properly ('result' object).
    buff.write(" * TESTS: %s\n"%result.num_tests)
    if result.num_errors:
        buff.write("\033[1;31m * ERRORS: %s\033[0m\n"%result.num_errors)
    else:
        buff.write(" * ERRORS: 0\n")
    if result.num_failures:
        buff.write("\033[1;31m * FAILURES: %s\033[0m\n"%result.num_failures)
    else:
        buff.write(" * FAILURES: 0\n")
    return buff.getvalue()

def main():
    from optparse import OptionParser
    parser = OptionParser(usage="usage: summarize_results.py [options] package")
    parser.add_option("--nodeps",
                      dest="no_deps", default=False,
                      action="store_true",
                      help="don't compute test results for the specified package only")
    (options, args) = parser.parse_args()
    if len(args) != 1:
        parser.error("Only one package may be specified")
    
    package = args[0]
    r = rospkg.RosPack()
    if options.no_deps:
        packages = [package]
    else:
        packages = [package] + r.get_depends_on(package, implicit=True)
        packages = [p for p in packages if p]

    result = junitxml.read_all(packages)
    print(create_summary(result, packages))
    if result.num_errors or result.num_failures:
        sys.exit(1)
  
if __name__ == '__main__':
    main()
