# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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
# Revision $Id: launch.py 8960 2010-04-07 23:54:50Z kwc $

"""
rostest implementation of running bare (gtest-compatible) unit test
executables. These do not run in a ROS environment.
"""

import os
import unittest
import time

import roslib.packages

import roslaunch.pmon

from rostest.rostestutil import createXMLRunner, printSummary, printRostestSummary, \
    xmlResultsFile, rostest_name_from_path, printlog, printlogerr
import rostest.xmlresults

BARE_TIME_LIMIT = 60.

class BareTestCase(unittest.TestCase):

    def __init__(self, exe, args, results, retry=0, time_limit=None, test_name=None):
        """
        @param exe: path to executable to run
        @type  exe: str
        @param args: arguments to exe
        @type  args: [str]
        @param results: test results accumulator
        @param retry: (optional) number of retries for test
        @type  retry: int
        @param time_limit: (optional) time limit for test. Defaults to BARE_TIME_LIMIT.
        @type  time_limit: float
        @param test_name: (optional) override automatically geneated test name
        @type  test_name: str
        """
        super(BareTestCase, self).__init__()
        self.results = results
        _, self.package = roslib.packages.get_dir_pkg(exe)
        self.exe = exe
        if test_name is None:
            self.test_name = os.path.basename(exe)
        else:
            self.test_name = test_name

        self.args = [self.exe] + args
        self.retry = retry
        self.time_limit = time_limit or BARE_TIME_LIMIT
        self.pmon = None
        
    def setUp(self):
        self.pmon = roslaunch.pmon.start_process_monitor()
        
    def tearDown(self):
        if self.pmon is not None:
            roslaunch.pmon.shutdown_process_monitor(self.pmon)
            self.pmon = None
        
    def runTest(self):
        self.failIf(self.package is None, "unable to determine package of executable")
            
        done = False
        while not done:
            test_name = self.test_name

            printlog("Running test [%s]", test_name)

            #setup the test
            # - we pass in the output test_file name so we can scrape it
            test_file = xmlResultsFile(self.package, test_name, False)
            if os.path.exists(test_file):
                printlog("removing previous test results file [%s]", test_file)
                os.remove(test_file)

            self.args.append('--gtest_output=xml:%s'%test_file)

            # run the test, blocks until completion
            printlog("running test %s"%test_name)
            timeout_failure = False

            run_id = None
            #TODO: really need different, non-node version of LocalProcess instead of these extra args
            process = roslaunch.nodeprocess.LocalProcess(run_id, self.package, self.test_name, self.args, os.environ, False, cwd='cwd', is_node=False)

            pm = self.pmon
            pm.register(process)
            success = process.start()
            self.assert_(success, "test failed to start")

            #poll until test terminates or alloted time exceed
            timeout_t = time.time() + self.time_limit
            try:
                while pm.mainthread_spin_once() and process.is_alive():
                    #test fails on timeout
                    if time.time() > timeout_t:
                        raise roslaunch.launch.RLTestTimeoutException("test max time allotted")
                    time.sleep(0.1)
                
            except roslaunch.launch.RLTestTimeoutException, e:
                if self.retry:
                    timeout_failure = True
                else:
                    raise

            if not timeout_failure:
                printlog("test [%s] finished"%test_name)
            else:
                printlogerr("test [%s] timed out"%test_name)                
        
            # load in test_file
            if not timeout_failure:
                self.assert_(os.path.isfile(test_file), "test [%s] did not generate test results"%test_name)
                printlog("test [%s] results are in [%s]", test_name, test_file)
                results = rostest.xmlresults.read(test_file, test_name)
                test_fail = results.num_errors or results.num_failures
            else:
                test_fail = True

            if self.retry > 0 and test_fail:
                self.retry -= 1
                printlog("test [%s] failed, retrying. Retries left: %s"%(test_name, self.retry))
            else:
                done = True
                self.results.accumulate(results)
                printlog("test [%s] results summary: %s errors, %s failures, %s tests",
                         test_name, results.num_errors, results.num_failures, results.num_tests)

        printlog("[ROSTEST] test [%s] done", test_name)
