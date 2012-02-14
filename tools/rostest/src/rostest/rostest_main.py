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

# NOTE: this has not survived the many refactorings and roslaunch changes well. There are too many ugly globals and bad
# code organizational choices at this point, but it's not a high priority to cleanup.

import os
import sys
import time
import unittest
import logging

import roslaunch
import rospkg
import rosgraph.roslogging

from rostest.rostestutil import createXMLRunner, printRostestSummary, \
    xmlResultsFile, rostest_name_from_path
from rostest.rostest_parent import ROSTestLaunchParent

import rostest.runner

_NAME = 'rostest'

def configure_logging():
    import socket
    logfile_basename = 'rostest-%s-%s.log'%(socket.gethostname(), os.getpid())
    logfile_name = rosgraph.roslogging.configure_logging('rostest', filename=logfile_basename)
    if logfile_name:
        print("... logging to %s"%logfile_name)
    return logfile_name

def write_bad_filename_failure(test_file, results_file, outname):
    # similar to rostest-check-results
    results_file_dir = os.path.dirname(results_file)
    if not os.path.isdir(results_file_dir):
        os.makedirs(results_file_dir)
    with open(results_file, 'w') as f:
        d = {'test': outname, 'test_file': test_file }
        f.write("""<?xml version="1.0" encoding="UTF-8"?>
<testsuite tests="1" failures="1" time="1" errors="0" name="%(test)s">
  <testcase name="test_ran" status="run" time="1" classname="Results">
    <failure message="rostest file [%(test_file)s] does not exist" type=""/>
  </testcase>
</testsuite>"""%d)
    
def rostestmain():
    import roslaunch.rlutil
    
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog [options] [package] <filename>", prog=_NAME)
    parser.add_option("-t", "--text",
                      action="store_true", dest="text_mode", default=False,
                      help="Run with stdout output instead of XML output")
    parser.add_option("--pkgdir", metavar="PKG_DIR",
                      dest="pkg_dir", default=None,
                      help="package dir")
    parser.add_option("--package", metavar="PACKAGE",
                      dest="package", default=None,
                      help="package")
    (options, args) = parser.parse_args()
    try:
        args = roslaunch.rlutil.resolve_launch_arguments(args)
    except roslaunch.core.RLException as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)

    # make sure all loggers are configured properly
    logfile_name = configure_logging()
    logger = logging.getLogger('rostest')
    import roslaunch.core
    roslaunch.core.add_printlog_handler(logger.info)
    roslaunch.core.add_printerrlog_handler(logger.error)        
        
    logger.info('rostest starting with options %s, args %s'%(options, args))
    if len(args) == 0:
        parser.error("You must supply a test file argument to rostest.")
    if len(args) != 1:
        parser.error("rostest only accepts a single test file")

    # compute some common names we'll be using to generate test names and files
    test_file = args[0]
    if options.pkg_dir and options.package:  
        # rosbuild2: the build system knows what package and directory, so let it tell us,
        # instead of shelling back out to rospack
        pkg_dir, pkg = options.pkg_dir, options.package
    else:
        pkg = rospkg.get_package_name(test_file)
        r = rospkg.RosPack()
        pkg_dir = r.get_path(pkg)

    outname = rostest_name_from_path(pkg_dir, test_file)

    # #1140
    if not os.path.isfile(test_file):
        results_file = xmlResultsFile(pkg, outname, True)
        write_bad_filename_failure(test_file, results_file, outname)
        parser.error("test file is invalid. Generated failure case result file in %s"%results_file)
        
    try:
        testCase = rostest.runner.createUnitTest(pkg, test_file)
        suite = unittest.TestLoader().loadTestsFromTestCase(testCase)

        if options.text_mode:
            rostest.runner.setTextMode(True)
            result = unittest.TextTestRunner(verbosity=2).run(suite)
        else:
            is_rostest = True
            results_file = xmlResultsFile(pkg, outname, is_rostest)        
            xml_runner = createXMLRunner(pkg, outname, \
                                             results_file=results_file, \
                                             is_rostest=is_rostest)
            result = xml_runner.run(suite)
    finally:
        # really make sure that all of our processes have been killed
        test_parents = rostest.runner.getRostestParents()
        for r in test_parents:
            logger.info("finally rostest parent tearDown [%s]", r)
            r.tearDown()
        del test_parents[:]
        from roslaunch.pmon import pmon_shutdown
        logger.info("calling pmon_shutdown")
        pmon_shutdown()
        logger.info("... done calling pmon_shutdown")

    # print config errors after test has run so that we don't get caught up in .xml results
    config = rostest.runner.getConfig()
    if config:
        if config.config_errors:
            print("\n[ROSTEST WARNINGS]"+'-'*62+'\n', file=sys.stderr)
        for err in config.config_errors:
            print(" * %s"%err, file=sys.stderr)
        print('')

    # summary is worthless if textMode is on as we cannot scrape .xml results
    subtest_results = rostest.runner.getResults()
    if not options.text_mode:
        printRostestSummary(result, subtest_results)
    else:
        print("WARNING: overall test result is not accurate when --text is enabled")

    if logfile_name:
        print("rostest log file is in %s"%logfile_name)
        
    if not result.wasSuccessful():
        sys.exit(1)
    elif subtest_results.num_errors or subtest_results.num_failures:
        sys.exit(2)
    
if __name__ == '__main__':
    rostestmain()
