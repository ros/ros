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

## Interface for using rostest from other Python code.

XML_OUTPUT_FLAG='--gtest_output=xml:' #use gtest-compatible flag

def is_subscriber(topic, subscriber_id):
    """
    Predicate to check whether or not master think subscriber_id
    subscribes to topic.
    @return: True if still register as a subscriber
    @rtype: bool
    """
    import roslib.scriptutil as scriptutil
    return scriptutil.is_subscriber(topic, subscriber_id)

def is_publisher(topic, publisher_id):
    """
    Predicate to check whether or not master think publisher_id
    publishes topic.
    @return: True if still register as a publisher
    @rtype: bool
    """    
    import roslib.scriptutil as scriptutil
    return scriptutil.is_publisher(topic, publisher_id)

def rosrun(package, test_name, test, sysargs=None):
    """
    Run a rostest/unittest-based integration test.
    
    @param package: name of package that test is in
    @type  package: str
    @param test_name: name of test that is being run
    @type  test_name: str
    @param test: test class 
    @type  test: unittest.TestCase
    @param sysargs: command-line args. If not specified, this defaults to sys.argv. rostest
      will look for the --text and --gtest_output parameters
    @type  sysargs: list
    """
    if sysargs is None:
        # lazy-init sys args
        import sys
        sysargs = sys.argv
        
    #parse sysargs
    result_file = None
    for arg in sysargs:
        if arg.startswith(XML_OUTPUT_FLAG):
            result_file = arg[len(XML_OUTPUT_FLAG):]
    text_mode = '--text' in sysargs
    coverage_mode = '--cov' in sysargs
    if coverage_mode:
        _start_coverage(package)

    # lazy-import so that these don't affect coverage tests
    from rostestutil import createXMLRunner, printSummary
    import unittest
    import rospy
    
    suite = unittest.TestLoader().loadTestsFromTestCase(test)
    if text_mode:
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    else:
        result = createXMLRunner(package, test_name, result_file).run(suite)
    if coverage_mode:
        _stop_coverage(package)
    printSummary(result)
    
    # shutdown any node resources in case test forgets to
    rospy.signal_shutdown('test complete')
    if not result.wasSuccessful():
        sys.exit(1)
    
# TODO: rename to rosrun -- migrating name to avoid confusion and enable easy xmlrunner use 
run = rosrun

def unitrun(package, test_name, test, sysargs=None, coverage_packages=[]):
    """
    Wrapper routine from running python unitttests with
    JUnit-compatible XML output.  This is meant for unittests that do
    not not need a running ROS graph (i.e. offline tests only).
    
    This enables JUnit-compatible test reporting so that
    test results can be reported to higher-level tools. 
    
    @param package: name of ROS package that is running the test
    @type  package: str
    @param coverage_packages: list of Python package to compute coverage results for. Defaults to package
    @type  coverage_packages: [str]
    """
    if sysargs is None:
        # lazy-init sys args
        import sys
        sysargs = sys.argv

    import unittest
    
    if not coverage_packages:
        coverage_packages = [package]
        
    #parse sysargs
    result_file = None
    for arg in sysargs:
        if arg.startswith(XML_OUTPUT_FLAG):
            result_file = arg[len(XML_OUTPUT_FLAG):]
    text_mode = '--text' in sysargs

    coverage_mode = '--cov' in sysargs

    if coverage_mode:
        _start_coverage(coverage_packages)

    # lazy-import after coverage tests begin
    from rostestutil import createXMLRunner, printSummary
        
    suite = unittest.TestLoader().loadTestsFromTestCase(test)
    if text_mode:
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    else:
        result = createXMLRunner(package, test_name, result_file).run(suite)
    if coverage_mode:
        _stop_coverage(coverage_packages)
    printSummary(result)
    
    if not result.wasSuccessful():
        sys.exit(1)

def _start_coverage(packages):
    try:
        import coverage
        coverage.erase()
        coverage.start()
    except ImportError, e:
        print >> sys.stderr, """WARNING: cannot import python-coverage, coverage tests will not run.
To install coverage, run 'easy_install coverage'"""
    try:
        # reload the module to get coverage
        for package in packages:
            if package in sys.modules:
                reload(sys.modules[package])
    except ImportError, e:
        print >> sys.stderr, "WARNING: cannot import '%s', will not generate coverage report"%package
        return

def _stop_coverage(packages):
    try:
        import coverage
        coverage.stop()
        try:
            for package in packages:
                pkg = __import__(package)
                m = [v for v in sys.modules.values() if v and v.__name__.startswith(package)]
                coverage.report(m, show_missing=0)
                for mod in m:
                    res = coverage.analysis(mod)
                    print "\n%s:\nMissing lines: %s"%(res[0], res[3])
        except ImportError, e:
            print >> sys.stderr, "WARNING: cannot import '%s', will not generate coverage report"%package
    except ImportError, e:
        print >> sys.stderr, """WARNING: cannot import python-coverage, coverage tests will not run.
To install coverage, run 'easy_install coverage'"""
    
    
#502: backwards compatibility for unbuilt rostest packages
def rostestmain():
    #NOTE: this is importing from rostest.rostest
    from rostest import rostestmain as _main
    _main()
