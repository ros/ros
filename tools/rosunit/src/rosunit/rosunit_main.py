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

from __future__ import with_statement

import os
import sys
import time
import unittest
import logging

import roslib.packages 
import roslib.roslogging

from . import pmon
from . core import xml_results_file, create_xml_runner

from .junitxml import print_summary, Result
from .baretest import BareTestCase


_NAME = 'rosunit'

def configure_logging():
    import socket
    logfile_basename = 'rosunit-%s-%s.log'%(socket.gethostname(), os.getpid())
    logfile_name = roslib.roslogging.configure_logging('rosunit', filename=logfile_basename)
    if logfile_name:
        print "... logging to %s"%logfile_name
    return logfile_name

def rosunitmain():
    # make sure all loggers are configured properly
    logfile_name = configure_logging()
    logger = logging.getLogger('rosunit')

    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog [options] <file> <test-name>", prog=_NAME)
    parser.add_option("-t", "--text",
                      action="store_true", dest="text_mode", default=False,
                      help="Run with stdout output instead of XML output")
    parser.add_option("--bare-limit", metavar="TIME_LIMIT",
                      dest="bare_limit", default=60,
                      help="Set time limit for --bare executable")
    parser.add_option("--bare-name", metavar="TEST_NAME",
                      dest="bare_name", default=None,
                      help="Test name for --bare executable")
    (options, args) = parser.parse_args()

    if len(args) != 2:
        parser.error("You must supply a test file and test name argument to rosunit.")

    test_file, bare_name = args
    logger.info('rosunit starting with options %s, args %s'%(options, args))

    # compute some common names we'll be using to generate test names and files
    pkg_dir, pkg = roslib.packages.get_dir_pkg(test_file) 

    try:
        time_limit = float(options.bare_limit) if options.bare_limit else None
        results = Result('rosunit', 0, 0, 0)

        test_case = BareTestCase(test_file, [], \
                                     retry=0, time_limit=time_limit, \
                                     test_name=options.bare_name)
        suite = unittest.TestSuite()
        suite.addTest(test_case)

        if options.text_mode:
            result = unittest.TextTestRunner(verbosity=2).run(suite)
        else:
            results_file = xml_results_file(pkg, bare_name, True)        
            xml_runner = create_xml_runner(pkg, bare_name, \
                                               results_file=results_file, \
                                               is_rostest=True)
            result = xml_runner.run(suite)
    finally:
        logger.info("calling pmon_shutdown")
        pmon.pmon_shutdown()
        logger.info("... done calling pmon_shutdown")

    # summary is worthless if textMode is on as we cannot scrape .xml results
    results = test_case.results
    if not options.text_mode:
        print "TODO: PRINT SUMMARY"
        if 0:
            print_summary(result)
    else:
        print "WARNING: overall test result is not accurate when --text is enabled"

    if logfile_name:
        print "rosunit log file is in %s"%logfile_name
        
    if not result.wasSuccessful():
        sys.exit(1)
    elif subtest_results.num_errors or subtest_results.num_failures:
        sys.exit(2)
    
if __name__ == '__main__':
    rosunitmain()
