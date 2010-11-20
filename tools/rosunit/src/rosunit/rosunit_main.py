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
from .baretest import BareTestCase, print_runner_summary


_NAME = 'rosunit'

def configure_logging(test_name):
    logfile_basename = 'rosunit-%s.log'%(test_name)
    logfile_name = roslib.roslogging.configure_logging('rosunit-%s'%(test_name), filename=logfile_basename)
    if logfile_name:
        print "... logging to %s"%logfile_name
    return logfile_name

def rosunitmain():
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog [options] <file> [test args...]", prog=_NAME)
    parser.add_option("-t", "--text",
                      action="store_true", dest="text_mode", default=False,
                      help="Run with stdout output instead of XML output")
    parser.add_option("--time-limit", metavar="TIME_LIMIT",
                      dest="time_limit", default=60,
                      help="Set time limit for test")
    parser.add_option("--name", metavar="TEST_NAME",
                      dest="test_name", default=None,
                      help="Test name")
    (options, args) = parser.parse_args()

    if len(args) < 1:
        parser.error("You must supply a test file.")

    test_file = args[0]
    
    if options.test_name:
        test_name = options.test_name
    else:
        test_name = os.path.basename(test_file)
        if '.' in test_name:
            test_name = test_name[:test_name.rfind('.')]
    time_limit = float(options.time_limit) if options.time_limit else None

    logfile_name = configure_logging(test_name)
    logger = logging.getLogger('rosunit')
    logger.info('rosunit starting with options %s, args %s'%(options, args))

    # compute some common names we'll be using to generate test names and files
    pkg_dir, pkg = roslib.packages.get_dir_pkg(test_file) 

    try:
        
        results = Result('rosunit', 0, 0, 0)

        test_case = BareTestCase(test_file, args[1:], \
                                     retry=0, time_limit=time_limit, \
                                     test_name=test_name)
        suite = unittest.TestSuite()
        suite.addTest(test_case)

        if options.text_mode:
            result = unittest.TextTestRunner(verbosity=2).run(suite)
        else:
            results_file = xml_results_file(pkg, test_name, True)
            # the is_rostest really just means "wrapper"
            xml_runner = create_xml_runner(pkg, test_name, \
                                               results_file=results_file, \
                                               is_rostest=True)
            runner_result = xml_runner.run(suite)
    finally:
        logger.info("calling pmon_shutdown")
        pmon.pmon_shutdown()
        logger.info("... done calling pmon_shutdown")

    # summary is worthless if textMode is on as we cannot scrape .xml results
    results = test_case.results
    if not options.text_mode:
        print_runner_summary(runner_result, results)
    else:
        print "WARNING: overall test result is not accurate when --text is enabled"

    if logfile_name:
        print "rosunit log file is in %s"%logfile_name
        
    if not runner_result.wasSuccessful():
        sys.exit(1)
    elif results.num_errors or results.num_failures:
        sys.exit(2)
    
if __name__ == '__main__':
    rosunitmain()
