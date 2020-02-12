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

from __future__ import print_function
from __future__ import with_statement

import os
import sys
import unittest

import rospkg

from . import pmon
from .baretest import BareTestCase
from .baretest import print_runner_summary
from . core import create_xml_runner
from . core import xml_results_file
from .junitxml import Result
from .junitxml import print_summary  # noqa: F401

_NAME = 'rosunit'


def rosunitmain():
    from optparse import OptionParser
    parser = OptionParser(usage='usage: %prog [options] <file> [test args...]', prog=_NAME)
    parser.add_option('-t', '--text',
                      action='store_true', dest='text_mode', default=False,
                      help='Run with stdout output instead of XML output')
    parser.add_option('--time-limit', metavar='TIME_LIMIT',
                      dest='time_limit', default=60,
                      help='Set time limit for test')
    parser.add_option('--name', metavar='TEST_NAME',
                      dest='test_name', default=None,
                      help='Test name')
    parser.add_option('--package', metavar='PACKAGE_NAME',
                      dest='pkg', default=None,
                      help='Package name (optional)')
    (options, args) = parser.parse_args()

    if len(args) < 1:
        parser.error('You must supply a test file.')

    test_file = args[0]

    if options.test_name:
        test_name = options.test_name
    else:
        test_name = os.path.basename(test_file)
        if '.' in test_name:
            test_name = test_name[:test_name.rfind('.')]
    time_limit = float(options.time_limit) if options.time_limit else None

    # If the caller didn't tell us the package name, we'll try to infer it.
    # compute some common names we'll be using to generate test names and files
    pkg = options.pkg
    if not pkg:
        pkg = rospkg.get_package_name(test_file)
    if not pkg:
        print("Error: failed to determine package name for file '%s'; maybe you should supply the --package argument to rosunit?" % (test_file))
        sys.exit(1)

    try:
        runner_result = None
        results = Result('rosunit', 0, 0, 0)

        test_case = BareTestCase(test_file, args[1:],
                                 retry=0, time_limit=time_limit,
                                 test_name=test_name, text_mode=options.text_mode, package_name=pkg)
        suite = unittest.TestSuite()
        suite.addTest(test_case)

        if options.text_mode:
            result = unittest.TextTestRunner(stream=sys.stdout, verbosity=2).run(suite)
        else:
            results_file = xml_results_file(pkg, test_name, True)
            # the is_rostest really just means "wrapper"
            xml_runner = create_xml_runner(pkg, test_name,
                                           results_file=results_file,
                                           is_rostest=True)
            runner_result = xml_runner.run(suite)
    finally:
        pmon.pmon_shutdown()

    # summary is worthless if textMode is on as we cannot scrape .xml results
    results = test_case.results
    if not options.text_mode:
        print_runner_summary(runner_result, results)
    else:
        print('WARNING: overall test result is not accurate when --text is enabled')

    if runner_result is not None and not runner_result.wasSuccessful():
        sys.exit(1)
    elif results.num_errors or results.num_failures:
        sys.exit(2)


if __name__ == '__main__':
    rosunitmain()
