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

from __future__ import print_function

import os
import sys
import logging
import time
import unittest

import rospkg
import roslaunch
import roslib.packages 

from rostest.rostestutil import createXMLRunner, printSummary, printRostestSummary, \
    xmlResultsFile, printlog, printlogerr
from rostest.rostest_parent import ROSTestLaunchParent
import rosunit.junitxml

_DEFAULT_TEST_PORT = 22422

# NOTE: ignoring Python style guide as unittest is sadly written with Java-like camel casing

_results = rosunit.junitxml.Result('rostest', 0, 0, 0)
def _accumulateResults(results):
    _results.accumulate(results)

def getResults():
    return _results

_textMode = False
def setTextMode(val):
    global _textMode 
    _textMode = val

# global store of all ROSLaunchRunners so we can do an extra shutdown
# in the rare event a tearDown fails to execute
_test_parents = []
_config = None
def _addRostestParent(runner):
    global _test_parents, _config
    logging.getLogger('rostest').info("_addRostestParent [%s]", runner)
    _test_parents.append(runner)
    _config = runner.config

def getConfig():
    return _config

def getRostestParents():
    return _test_parents
    
# TODO: convert most of this into a run() routine of a RoslaunchRunner subclass

## generate test failure if tests with same name in launch file
def failDuplicateRunner(testName):
    def fn(self):
        print("Duplicate tests named [%s] in rostest suite"%testName)
        self.fail("Duplicate tests named [%s] in rostest suite"%testName)
    return fn

def failRunner(testName, message):
    def fn(self):
        print(message, file=sys.stderr)
        self.fail(message)
    return fn
    
def rostestRunner(test, test_pkg):
    """
    Test function generator that takes in a roslaunch Test object and
    returns a class instance method that runs the test. TestCase
    setUp() is responsible for ensuring that the rest of the roslaunch
    state is correct and tearDown() is responsible for tearing
    everything down cleanly.
    @param test: rost test to run
    @type  test: roslaunch.Test
    @return: function object to run testObj
    @rtype: fn
    """
    
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

            # TODO: have to redeclare this due to a bug -- this file
            # needs to be renamed as it aliases the module where the
            # constant is elsewhere defined. The fix is to rename
            # rostest.py
            XML_OUTPUT_FLAG='--gtest_output=xml:' #use gtest-compatible flag
            
            test.args = "%s %s%s"%(test.args, XML_OUTPUT_FLAG, test_file)
            if _textMode:
                test.output = 'screen'
                test.args = test.args + " --text"

            # run the test, blocks until completion
            printlog("running test %s"%test_name)
            timeout_failure = False
            try:
                self.test_parent.run_test(test)
            except roslaunch.launch.RLTestTimeoutException as e:
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
                    results = rosunit.junitxml.read(test_file, test_name)
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
    
def createUnitTest(pkg, test_file):
    """
    Unit test factory. Constructs a unittest class based on the roslaunch

    @param pkg: package name
    @type  pkg: str
    @param test_file: rostest filename
    @type  test_file: str
    """
    # parse the config to find the test files
    config = roslaunch.parent.load_config_default([test_file], _DEFAULT_TEST_PORT)

    # pass in config to class as a property so that test_parent can be initialized
    classdict = { 'setUp': setUp, 'tearDown': tearDown, 'config': config,
                  'test_parent': None, 'test_file': test_file }
    
    # add in the tests
    testNames = []
    for test in config.tests:
        # #1989: find test first to make sure it exists and is executable
        err_msg = None
        try:
            rp = rospkg.RosPack()
            cmd = roslib.packages.find_node(test.package, test.type, rp)
            if not cmd:
                err_msg = "Test node [%s/%s] does not exist or is not executable"%(test.package, test.type)
        except rospkg.ResourceNotFound as e:
            err_msg = "Package [%s] for test node [%s/%s] does not exist"%(test.package, test.package, test.type)

        testName = 'test%s'%(test.test_name)
        if err_msg:
            classdict[testName] = failRunner(test.test_name, err_msg)
        elif testName in testNames:
            classdict[testName] = failDuplicateRunner(test.test_name)
        else:
            classdict[testName] = rostestRunner(test, pkg)
            testNames.append(testName)

    # instantiate the TestCase instance with our magically-created tests
    return type('RosTest',(unittest.TestCase,),classdict)

