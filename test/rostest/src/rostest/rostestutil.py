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

## module for isolating helper routines that are not ROS-specific. 

# IMPORTANT: no routine here can in anyway cause rospy to be loaded (that includes roslaunch)

import os
import cStringIO

import roslib.rosenv 

import xmlrunner

## command-line flag for designating XML output destination. 
XML_OUTPUT_FLAG='--gtest_output=xml:' #use gtest-compatible flag

_errors = None
def getErrors():
    return _errors

def rostest_name_from_path(pkg_dir, test_file):
    test_file_abs = os.path.abspath(test_file)
    if test_file_abs.startswith(pkg_dir):
        # compute package-relative path
        test_file = test_file_abs[len(pkg_dir):]
        if test_file[0] == os.sep:
            test_file = test_file[1:]
    outname = test_file.replace(os.sep, '_')
    if '.' in outname:
        outname = outname[:outname.rfind('.')]
    return outname

def printRostestSummary(result, rostest_results):
    # we have two separate result objects, which can be a bit
    # confusing. 'result' counts successful _running_ of tests
    # (i.e. doesn't check for actual test success). The 'r' result
    # object contains results of the actual tests.
    
    global _errors
    _errors = result.errors
    
    buff = cStringIO.StringIO()
    buff.write("[ROSTEST]"+'-'*71+'\n\n')
    for tc_result in rostest_results.test_case_results:
        buff.write(tc_result.description)
    for tc_result in result.failures:
        buff.write("[%s][failed]\n"%tc_result[0]._testMethodName)

    buff.write('\nSUMMARY\n')
    if result.wasSuccessful() and (rostest_results.num_errors + rostest_results.num_failures) == 0:
        buff.write("\033[32m * RESULT: SUCCESS\033[0m\n")
    else:
        buff.write("\033[1;31m * RESULT: FAIL\033[0m\n")

    # TODO: still some issues with the numbers adding up if tests fail to launch

    # number of errors from the inner tests, plus add in count for tests
    # that didn't run properly ('result' object).
    buff.write(" * TESTS: %s\n"%rostest_results.num_tests)
    num_errors = rostest_results.num_errors+len(result.errors)
    if num_errors:
        buff.write("\033[1;31m * ERRORS: %s\033[0m\n"%num_errors)
    else:
        buff.write(" * ERRORS: 0\n")
    num_failures = rostest_results.num_failures+len(result.failures)
    if num_failures:
        buff.write("\033[1;31m * FAILURES: %s\033[0m\n"%num_failures)
    else:
        buff.write(" * FAILURES: 0\n")
        
    if result.failures:
        buff.write("\nERROR: The following tests failed to run:\n")
        for tc_result in result.failures:
            buff.write(" * " +tc_result[0]._testMethodName + "\n")

    print buff.getvalue()

def printSummary(result):
    buff = cStringIO.StringIO()
    buff.write("-------------------------------------------------------------\nSUMMARY:\n")
    if result.wasSuccessful():
        buff.write("\033[32m * RESULT: SUCCESS\033[0m\n")
    else:
        buff.write(" * RESULT: FAIL\n")
    buff.write(" * TESTS: %s\n"%result.testsRun)
    buff.write(" * ERRORS: %s [%s]\n"%(len(result.errors), ', '.join([e[0]._testMethodName for e in result.errors])))
    buff.write(" * FAILURES: %s [%s]\n"%(len(result.failures), ','.join([e[0]._testMethodName for e in result.failures])))
    print buff.getvalue()

## Create the unittest test runner with XML output
## @param test_pkg str: package name
## @param test_name str: test name
## @param is_rostest bool: if True, use naming scheme for rostest itself instead of individual unit test naming
def createXMLRunner(test_pkg, test_name, results_file=None, is_rostest=False):
    test_name = os.path.basename(test_name)
    # determine output xml file name
    if not results_file:
        results_file = xmlResultsFile(test_pkg, test_name, is_rostest)
    test_dir = os.path.dirname(results_file)
    if not os.path.exists(test_dir):
        os.makedirs(test_dir) #NOTE: this will pass up an error exception if it fails
    elif os.path.isfile(test_dir):
        raise Exception("ERROR: cannot run test suite, file is preventing creation of test dir: %s"%test_dir)
    
    print "[ROSTEST] Outputting test results to %s"%results_file
    outstream = open(results_file, 'w')
    return xmlrunner.XMLTestRunner(stream=outstream)
    
## @param test_pkg str: name of test's package 
## @param test_name str: name of test
## @param is_rostest bool: True if the results file is for a rostest-generated unit instance
## @return str: name of xml results file for specified test
def xmlResultsFile(test_pkg, test_name, is_rostest=False):
    test_dir = os.path.join(roslib.rosenv.get_ros_root(), 'test', 'test_results', test_pkg)
    # #576: strip out chars that would bork the filename
    # this is fairly primitive, but for now just trying to catch some common cases
    for c in ' "\'&$!`/\\':
        if c in test_name:
            test_name = test_name.replace(c, '_')
    if is_rostest:
        return os.path.join(test_dir, 'TEST-rostest__%s.xml'%test_name)
    else:
        return os.path.join(test_dir, 'TEST-%s.xml'%test_name)        
    
