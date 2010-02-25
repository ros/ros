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
rostest helper routines.
"""

# IMPORTANT: no routine here can in anyway cause rospy to be loaded (that includes roslaunch)

import os
import cStringIO

import roslib.rosenv 

import xmlrunner

_errors = None
def getErrors():
    return _errors

def rostest_name_from_path(pkg_dir, test_file):
    """
    Derive name of rostest name based on file name/path.
    
    @return: name of rostest
    @rtype: str
    """
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
    """
    Print summary of rostest results to stdout.
    """
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
    """
    Print summary of unit test result to stdout
    @param result: test results
    """
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

def createXMLRunner(test_pkg, test_name, results_file=None, is_rostest=False):
    """
    Create the unittest test runner with XML output
    @param test_pkg: package name
    @type  test_pkg: str
    @param test_name: test name
    @type  test_name: str
    @param is_rostest: if True, use naming scheme for rostest itself instead of individual unit test naming
    @type  is_rostest: bool
    """
    test_name = os.path.basename(test_name)
    # determine output xml file name
    if not results_file:
        results_file = xmlResultsFile(test_pkg, test_name, is_rostest)
    test_dir = os.path.abspath(os.path.dirname(results_file))
    if not os.path.exists(test_dir):
        try:
            roslib.rosenv.makedirs_with_parent_perms(test_dir) #NOTE: this will pass up an error exception if it fails
        except OSError:
            raise IOError("cannot create test results directory [%s]. Please check permissions."%(test_dir))

    elif os.path.isfile(test_dir):
        raise Exception("ERROR: cannot run test suite, file is preventing creation of test dir: %s"%test_dir)
    
    print "[ROSTEST] Outputting test results to %s"%results_file
    outstream = open(results_file, 'w')
    return xmlrunner.XMLTestRunner(stream=outstream)
    
def xmlResultsFile(test_pkg, test_name, is_rostest=False):
    """
    @param test_pkg: name of test's package 
    @type  test_pkg: str
    @param test_name str: name of test
    @type  test_name: str
    @param is_rostest: True if the results file is for a rostest-generated unit instance
    @type  is_rostest: bool
    @return: name of xml results file for specified test
    @rtype:  str
    """
    test_dir = os.path.join(roslib.rosenv.get_test_results_dir(), test_pkg)
    if not os.path.exists(test_dir):
        try:
            roslib.rosenv.makedirs_with_parent_perms(test_dir)
        except OSError:
            raise IOError("cannot create test results directory [%s]. Please check permissions."%(test_dir))
        
    # #576: strip out chars that would bork the filename
    # this is fairly primitive, but for now just trying to catch some common cases
    for c in ' "\'&$!`/\\':
        if c in test_name:
            test_name = test_name.replace(c, '_')
    if is_rostest:
        return os.path.join(test_dir, 'TEST-rostest__%s.xml'%test_name)
    else:
        return os.path.join(test_dir, 'TEST-%s.xml'%test_name)        
    
def test_failure_junit_xml(test_name, message):
    """
    Generate JUnit XML file for a unary test suite where the test failed
    
    @param test_name: Name of test that failed
    @type  test_name: str
    @param message: failure message
    @type  message: str
    """
    return """<?xml version="1.0" encoding="UTF-8"?>
<testsuite tests="1" failures="1" time="1" errors="0" name="%s">
  <testcase name="test_ran" status="run" time="1" classname="Results">
  <failure message="%s" type=""/>
  </testcase>
</testsuite>"""%(test_name, message)

def test_success_junit_xml(test_name):
    """
    Generate JUnit XML file for a unary test suite where the test succeeded.
    
    @param test_name: Name of test that passed
    @type  test_name: str
    """
    return """<?xml version="1.0" encoding="UTF-8"?>
<testsuite tests="1" failures="0" time="1" errors="0" name="%s">
  <testcase name="test_ran" status="run" time="1" classname="Results">
  </testcase>
</testsuite>"""%(test_name)
