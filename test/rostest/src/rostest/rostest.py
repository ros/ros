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

import os
import sys
import time
import unittest
import logging

import roslaunch
import roslib.packages 

from rostestutil import createXMLRunner, printSummary, printRostestSummary, xmlResultsFile, XML_OUTPUT_FLAG

import xmlresults

_NAME = 'rostest'
_DEFAULT_TEST_PORT = 22422

logger = logging.getLogger('roslaunch.rostest')

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
_runners = []
def _addRunner(runner):
    global _runners
    _runners.append(runner)
    
# TODO: convert most of this into a run() routine of a RoslaunchRunner subclass

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
        self.assert_(self.runner is not None, "ROSLaunchRunner initialization failed")

        test_name = test.test_name
        print "[ROSTEST] Running test [%s]"%test_name
        logger.info("Running test [%s]", test_name)

        #launch the other nodes
        succeeded, failed = self.runner.launch()
        self.assert_(not failed, "Test Fixture Nodes %s failed to launch"%failed)

        #setup the test
        # - we pass in the output test_file name so we can scrape it
        test_file = xmlResultsFile(test_pkg, test_name, False)
        if os.path.exists(test_file):
            print "[ROSTEST] removing previous test results file [%s]"%test_file
            logger.info("removing previous test results file [%s]", test_file)
            os.remove(test_file)
        test.args = "%s %s%s"%(test.args, XML_OUTPUT_FLAG, test_file)
        if _textMode:
            test.output = 'screen'
            test.args = test.args + " --text"

        # run the test, blocks until completion
        self.runner.run_test(test)
        
        # load in test_file
        if not _textMode:
            self.assert_(os.path.isfile(test_file), "test [%s] did not generate test results"%test_name)
            print "[ROSTEST] test [%s] results are in [%s]"%(test_name, test_file)
            results = xmlresults.read(test_file, test_name)
            _accumulateResults(results)
            print "[ROSTEST] test [%s] results summary: %s errors, %s failures, %s tests"%(\
                        test_name, results.num_errors, results.num_failures, results.num_tests)

            #self.assertEquals(0, results.num_errors, "unit test reported errors")
            #self.assertEquals(0, results.num_failures, "unit test reported failures")        
        print "[ROSTEST] test [%s] done"%test_name
        logger.info("test [%s] done", test_name)

    return fn

## Function that becomes TestCase.setup()
def setUp(self):
    print "[ROSTEST] setup[%s]"%self.test_file
    logger.info("ROSTEST SETUP %s", self.test_file)
    self.runner = roslaunch.ROSLaunchRunner(self.config)
    _addRunner(self.runner)
    
## Function that becomes TestCase.tearDown()    
def tearDown(self):
    print "[ROSTEST] tearDown[%s]"%self.test_file
    logger.info("ROSTEST TEAR DOWN %s", self.test_file)
    runner = self.runner
    self.runner = None
    runner.stop()
    logger.info("rostest tear down %s complete", self.config)
    
## Unit test factory. Constructs a unittest class based on the roslaunch
def createUnitTest(config, pkg, test_file):
    # the config attribute makes it easy for tests to access the ROSLaunchConfig instance
    classdict = { 'setUp': setUp, 'tearDown': tearDown, 'config': config, 'runner': None, 'test_file': test_file }
    
    # add in the tests
    for test in config.tests:
        testName = 'test%s'%(test.test_name)
        classdict[testName] = rostestRunner(test, pkg)

    # instantiate the TestCase instance with our magically-created tests
    return type('RosTest',(unittest.TestCase,),classdict)

## create and initialize the ROSLaunchConfig
## @param file: XML file for roslaunch configuration
## @return ROSLaunchConfig: initialized roslaunch configuration
def load_launch_config(file):
    logger.info('loading launch config %s', file)
    config = roslaunch.ROSLaunchConfig()
    loader = roslaunch.XmlLoader()
    try:
        # #764: load roscore into rostest
        roslaunch.load_roscore(loader, config)

        # load test config
        loader.load(file, config)
    except roslaunch.XmlParseException, e:
        #TODO wrap so that unittest catches this instead
        print >> sys.stderr, "[ROSTEST] ERROR", e
        sys.exit(1)

    # override master setting for test
    config.master.auto = config.master.AUTO_RESTART
    config.master.set_port(_DEFAULT_TEST_PORT)
    
    # send master output to log file instead of screen
    config.master.log_output = True
    return config

def configure_logging():
    from roslib.scriptutil import configure_logging
    logfile_name = configure_logging('rostest', additional=['roslaunch'])
    if logfile_name:
        print "... logging to %s"%logfile_name

def rostestmain():
    configure_logging()
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog [options] file", prog=_NAME)
    parser.add_option("-t", "--text",
                      action="store_true", dest="text_mode", default=False,
                      help="Run with stdout output instead of XML output")
    (options, args) = parser.parse_args()
    logger.info('rostest starting with options %s, args %s'%(options, args))
    if len(args) == 0:
        parser.error("You must supply a test file argument to rostest.")
    if len(args) != 1:
        parser.error("rostest only accepts a single test file")

    test_file = args[0]        
    config = load_launch_config(test_file)
    
    _, pkg = roslib.packages.get_dir_pkg(test_file) or 'unknownpkg'

    try:
        testCase = createUnitTest(config, pkg, test_file)
        suite = unittest.TestLoader().loadTestsFromTestCase(testCase)

        if options.text_mode:
            _setTextMode(True)
            result = unittest.TextTestRunner(verbosity=2).run(suite)
        else:
            outname = os.path.basename(test_file)
            if outname.lower().endswith('.xml'):
                outname = outname[:-4]

            is_rostest = True
            results_file = xmlResultsFile(pkg, outname, is_rostest)        
            runner = createXMLRunner(pkg, outname, \
                                     results_file=results_file, \
                                     is_rostest=is_rostest)
            result = runner.run(suite)
            #_accumulateResults(xmlresults.read(results_file))
    finally:
        # really make sure that all of our processes have been killed
        for r in _runners:
            r.stop()
        del _runners[:]
    
    # summary is worthless if textMode is on as we cannot scrape .xml results
    subtest_results = _getResults()
    if not _textMode:
        printRostestSummary(result, subtest_results)
    else:
        print "WARNING: overall test result is not accurate when --text is enabled"
    if not result.wasSuccessful():
        sys.exit(1)
    elif subtest_results.num_errors or subtest_results.num_failures:
        sys.exit(2)
    
if __name__ == '__main__':
    rostestmain()
