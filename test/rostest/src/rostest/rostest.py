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

import roslaunch
import roslib.packages 
import roslib.roslogging

from rostestutil import createXMLRunner, printSummary, printRostestSummary, xmlResultsFile, XML_OUTPUT_FLAG, rostest_name_from_path

from rostest_parent import ROSTestLaunchParent

import xmlresults

_NAME = 'rostest'
_DEFAULT_TEST_PORT = 22422


logger = logging.getLogger('roslaunch.rostest')

def printlog(msg, *args):
    if args:
        msg = msg%args
    logger.info(msg)
    print "[ROSTEST]"+msg
def printlogerr(msg, *args):
    if args:
        msg = msg%args
    logger.error(msg)
    print >> sys.stderr, "[ROSTEST]"+msg

# NOTE: ignoring Python style guide as unittest is sadly written with Java-like camel casing

_results = xmlresults.Result('rostest', 0, 0, 0)
def _accumulateResults(results):
    _results.accumulate(results)
def _getResults():
    return _results

_textMode = False
def _setTextMode(val):
    global _textMode 
    _textMode = val

# global store of all ROSLaunchRunners so we can do an extra shutdown
# in the rare event a tearDown fails to execute
_test_parents = []
def _addRostestParent(runner):
    global _test_parents
    logger.info("_addRostestParent [%s]", runner)
    _test_parents.append(runner)
    
# TODO: convert most of this into a run() routine of a RoslaunchRunner subclass

## generate test failure if tests with same name in launch file
def failDuplicateRunner(testName):
    def fn(self):
        print "Duplicate tests named [%s] in rostest suite"%testName
        self.fail("Duplicate tests named [%s] in rostest suite"%testName)
    return fn
    
## Test function generator that takes in a roslaunch Test object and
## returns a class instance method that runs the test. TestCase
## setUp() is responsible for ensuring that the rest of the roslaunch
## state is correct and tearDown() is responsible for tearing
## everything down cleanly.
## @param test roslaunch.Test: rost test to run
## @return fn: function object to run \a testObj
def rostestRunner(test, test_pkg):
    ## test case pass/fail is a measure of whether or not the test ran
    def fn(self):
        done = False
        while not done:
            self.assert_(self.test_parent is not None, "ROSTestParent initialization failed")

            test_name = test.test_name

            printlog("Running test [%s]", test_name)

            #launch the other nodes
            succeeded, failed = self.test_parent.launch()
            self.assert_(not failed, "Test Fixture Nodes %s failed to launch"%failed)

            #setup the test
            # - we pass in the output test_file name so we can scrape it
            test_file = xmlResultsFile(test_pkg, test_name, False)
            if os.path.exists(test_file):
                printlog("removing previous test results file [%s]", test_file)
                os.remove(test_file)
            test.args = "%s %s%s"%(test.args, XML_OUTPUT_FLAG, test_file)
            if _textMode:
                test.output = 'screen'
                test.args = test.args + " --text"

            # run the test, blocks until completion
            printlog("running test %s"%test_name)
            timeout_failure = False
            try:
                self.test_parent.run_test(test)
            except roslaunch.launch.RLTestTimeoutException, e:
                if test.retry:
                    timeout_failure = True
                else:
                    raise

            if not timeout_failure:
                printlog("test [%s] finished"%test_name)
            else:
                printlogerr("test [%s] timed out"%test_name)                
        
            # load in test_file
            if not _textMode or timeout_failure:
                
                if not timeout_failure:
                    self.assert_(os.path.isfile(test_file), "test [%s] did not generate test results"%test_name)
                    printlog("test [%s] results are in [%s]", test_name, test_file)
                    results = xmlresults.read(test_file, test_name)
                    test_fail = results.num_errors or results.num_failures
                else:
                    test_fail = True

                if test.retry > 0 and test_fail:
                    test.retry -= 1
                    printlog("test [%s] failed, retrying. Retries left: %s"%(test_name, test.retry))
                    self.tearDown()
                    self.setUp()
                else:
                    done = True
                    _accumulateResults(results)
                    printlog("test [%s] results summary: %s errors, %s failures, %s tests",
                             test_name, results.num_errors, results.num_failures, results.num_tests)

                    #self.assertEquals(0, results.num_errors, "unit test reported errors")
                    #self.assertEquals(0, results.num_failures, "unit test reported failures")
            else:
                if test.retry:
                    printlogerr("retry is disabled in --text mode")
                done = True
        printlog("[ROSTEST] test [%s] done", test_name)

    return fn

## Function that becomes TestCase.setup()
def setUp(self):
    # new test_parent for each run. we are a bit inefficient as it would be possible to
    # reuse the roslaunch base infrastructure for each test, but the roslaunch code
    # is not abstracted well enough yet
    self.test_parent = ROSTestLaunchParent(self.config, [self.test_file], port=_DEFAULT_TEST_PORT)
    
    printlog("setup[%s] run_id[%s] starting", self.test_file, self.test_parent.run_id)

    self.test_parent.setUp()
    
    # the config attribute makes it easy for tests to access the ROSLaunchConfig instance
    self.config = self.test_parent.config

    _addRostestParent(self.test_parent)

    printlog("setup[%s] run_id[%s] done", self.test_file, self.test_parent.run_id)
    
## Function that becomes TestCase.tearDown()    
def tearDown(self):
    printlog("tearDown[%s]", self.test_file)
    
    if self.test_parent:
        self.test_parent.tearDown()
        
    printlog("rostest teardown %s complete", self.test_file)
    
## Unit test factory. Constructs a unittest class based on the roslaunch
def createUnitTest(pkg, test_file):
    # parse the config to find the test files
    config = roslaunch.parent.load_config_default([test_file], _DEFAULT_TEST_PORT)
    config.master.log_output = True

    # pass in config to class as a property so that test_parent can be initialized
    classdict = { 'setUp': setUp, 'tearDown': tearDown, 'config': config,
                  'test_parent': None, 'test_file': test_file }
    
    # add in the tests
    testNames = []
    for test in config.tests:
        testName = 'test%s'%(test.test_name)
        if testName in testNames:
            classdict[testName] = failDuplicateRunner(test.test_name)
        else:
            classdict[testName] = rostestRunner(test, pkg)
            testNames.append(testName)

    # instantiate the TestCase instance with our magically-created tests
    return type('RosTest',(unittest.TestCase,),classdict)

def configure_logging():
    import socket
    logfile_basename = 'rostest-%s-%s.log'%(socket.gethostname(), os.getpid())
    logfile_name = roslib.roslogging.configure_logging('rostest', filename=logfile_basename, additional=['roslaunch', 'rospy', 'roslib', "paramiko"])
    if logfile_name:
        print "... logging to %s"%logfile_name
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

    # make sure all loggers are configured properly
    logfile_name = configure_logging()
    import roslaunch.core
    roslaunch.core.add_printlog_handler(logger.info)
    roslaunch.core.add_printerrlog_handler(logger.error)        
    
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog [options] file", prog=_NAME)
    parser.add_option("-t", "--text",
                      action="store_true", dest="text_mode", default=False,
                      help="Run with stdout output instead of XML output")
    (options, args) = parser.parse_args()
    try:
        args = roslaunch.rlutil.resolve_launch_arguments(args)
    except roslaunch.core.RLException, e:
        print >> sys.stderr, str(e)
        sys.exit(1)

    logger.info('rostest starting with options %s, args %s'%(options, args))
    if len(args) == 0:
        parser.error("You must supply a test file argument to rostest.")
    if len(args) != 1:
        parser.error("rostest only accepts a single test file")

    # compute some common names we'll be using to generate test names and files
    test_file = args[0]
    pkg_dir, pkg = roslib.packages.get_dir_pkg(test_file) 
    outname = rostest_name_from_path(pkg_dir, test_file)

    # #1140
    if not os.path.isfile(test_file):
        results_file = xmlResultsFile(pkg, outname, True)
        write_bad_filename_failure(test_file, results_file, outname)
        parser.error("test file is invalid. Generated failure case result file in %s"%results_file)
        
    try:
        testCase = createUnitTest(pkg, test_file)
        suite = unittest.TestLoader().loadTestsFromTestCase(testCase)

        if options.text_mode:
            _setTextMode(True)
            result = unittest.TextTestRunner(verbosity=2).run(suite)
        else:
            is_rostest = True
            results_file = xmlResultsFile(pkg, outname, is_rostest)        
            xml_runner = createXMLRunner(pkg, outname, \
                                             results_file=results_file, \
                                             is_rostest=is_rostest)
            result = xml_runner.run(suite)
            #_accumulateResults(xmlresults.read(results_file))
    finally:
        # really make sure that all of our processes have been killed
        for r in _test_parents:
            logger.info("finally rostest parent tearDown [%s]", r)
            r.tearDown()
        del _test_parents[:]
        from roslaunch.pmon import pmon_shutdown
        logger.info("calling pmon_shutdown")
        pmon_shutdown()
        logger.info("... done calling pmon_shutdown")
        
    # summary is worthless if textMode is on as we cannot scrape .xml results
    subtest_results = _getResults()
    if not _textMode:
        printRostestSummary(result, subtest_results)
    else:
        print "WARNING: overall test result is not accurate when --text is enabled"

    if logfile_name:
        print "rostest log file is in %s"%logfile_name
        
    if not result.wasSuccessful():
        sys.exit(1)
    elif subtest_results.num_errors or subtest_results.num_failures:
        sys.exit(2)
    
if __name__ == '__main__':
    rostestmain()
