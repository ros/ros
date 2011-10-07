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
Interface for using rostest from other Python code as well as running
Python unittests with additional reporting mechanisms and rosbuild
(CMake) integration.
"""

from __future__ import print_function

import sys
import rosunit

import rosgraph

XML_OUTPUT_FLAG = '--gtest_output=xml:' #use gtest-compatible flag

_GLOBAL_CALLER_ID = '/script'
#TODO: replace with rosgraph.masterapi
def get_master():
    """
    Get an XMLRPC handle to the Master. It is recommended to use the
    `rosgraph.masterapi` library instead, as it provides many
    conveniences.
    
    @return: XML-RPC proxy to ROS master
    @rtype: xmlrpclib.ServerProxy
    """
    try:
        import xmlrpc.client as xmlrpcclient  #Python 3.x
    except ImportError:
        import xmlrpclib as xmlrpcclient #Python 2.x
    uri = rosgraph.get_master_uri()
    return xmlrpcclient.ServerProxy(uri)

def is_subscriber(topic, subscriber_id):
    """
    Check whether or not master think subscriber_id subscribes to topic

    :returns: ``True`` if still register as a subscriber, ``bool``
    :raises: IOError If communication with master fails
    """
    m = get_master()
    code, msg, state = m.getSystemState(_GLOBAL_CALLER_ID)
    if code != 1:
        raise IOError("Unable to retrieve master state: %s"%msg)
    _, subscribers, _ = state
    for t, l in subscribers:
        if t == topic:
            return subscriber_id in l
    else:
        return False

def is_publisher(topic, publisher_id):
    """
    Predicate to check whether or not master think publisher_id
    publishes topic
    :returns: ``True`` if still register as a publisher, ``bool``
    :raises: IOError If communication with master fails
    """
    m = get_master()
    code, msg, state = m.getSystemState(_GLOBAL_CALLER_ID)
    if code != 1:
        raise IOError("Unable to retrieve master state: %s"%msg)
    pubs, _, _ = state
    for t, l in pubs:
        if t == topic:
            return publisher_id in l
    else:
        return False

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

    import unittest
    import rospy
    
    suite = unittest.TestLoader().loadTestsFromTestCase(test)
    if text_mode:
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    else:
        result = rosunit.create_xml_runner(package, test_name, result_file).run(suite)
    if coverage_mode:
        _stop_coverage(package)
    rosunit.print_unittest_summary(result)
    
    # shutdown any node resources in case test forgets to
    rospy.signal_shutdown('test complete')
    if not result.wasSuccessful():
        import sys
        sys.exit(1)
    
# TODO: rename to rosrun -- migrating name to avoid confusion and enable easy xmlrunner use 
run = rosrun

import warnings
def deprecated(func):
    """This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emmitted
    when the function is used."""
    def newFunc(*args, **kwargs):
        warnings.warn("Call to deprecated function %s." % func.__name__,
                      category=DeprecationWarning, stacklevel=2)
        return func(*args, **kwargs)
    newFunc.__name__ = func.__name__
    newFunc.__doc__ = func.__doc__
    newFunc.__dict__.update(func.__dict__)
    return newFunc

@deprecated
def unitrun(package, test_name, test, sysargs=None, coverage_packages=None):
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
    rosunit.unitrun(package, test_name, test, sysargs=sysargs, coverage_packages=coverage_packages)

# coverage instance
_cov = None
def _start_coverage(packages):
    global _cov
    try:
        import coverage
        try:
            _cov = coverage.coverage()
            # load previous results as we need to accumulate
            _cov.load()
            _cov.start()
        except coverage.CoverageException:
            print("WARNING: you have an older version of python-coverage that is not support. Please update to the version provided by 'easy_install coverage'", file=sys.stderr)
    except ImportError as e:
        print("""WARNING: cannot import python-coverage, coverage tests will not run.
To install coverage, run 'easy_install coverage'""", file=sys.stderr)
    try:
        # reload the module to get coverage
        for package in packages:
            if package in sys.modules:
                reload(sys.modules[package])
    except ImportError as e:
        print("WARNING: cannot import '%s', will not generate coverage report"%package, file=sys.stderr)
        return

def _stop_coverage(packages, html=None):
    """
    @param packages: list of packages to generate coverage reports for
    @type  packages: [str]
    @param html: (optional) if not None, directory to generate html report to
    @type  html: str
    """
    if _cov is None:
        return
    import sys, os
    try:
        _cov.stop()
        # accumulate results
        _cov.save()
        
        # - update our own .coverage-modules file list for
        #   coverage-html tool. The reason we read and rewrite instead
        #   of append is that this does a uniqueness check to keep the
        #   file from growing unbounded
        if os.path.exists('.coverage-modules'):
            with open('.coverage-modules','r') as f:
                all_packages = set([x for x in f.read().split('\n') if x.strip()] + packages)
        else:
            all_packages = set(packages)
        with open('.coverage-modules','w') as f:
            f.write('\n'.join(all_packages)+'\n')
            
        try:
            # list of all modules for html report
            all_mods = []

            # iterate over packages to generate per-package console reports
            for package in packages:
                pkg = __import__(package)
                m = [v for v in sys.modules.values() if v and v.__name__.startswith(package)]
                all_mods.extend(m)

                # generate overall report and per module analysis
                _cov.report(m, show_missing=0)
                for mod in m:
                    res = _cov.analysis(mod)
                    print("\n%s:\nMissing lines: %s"%(res[0], res[3]))
                    
            if html:
                
                print("="*80+"\ngenerating html coverage report to %s\n"%html+"="*80)
                _cov.html_report(all_mods, directory=html)
        except ImportError as e:
            print("WARNING: cannot import '%s', will not generate coverage report"%package, file=sys.stderr)
    except ImportError as e:
        print("""WARNING: cannot import python-coverage, coverage tests will not run.
To install coverage, run 'easy_install coverage'""", file=sys.stderr)
    
    
#502: backwards compatibility for unbuilt rostest packages
def rostestmain():
    #NOTE: this is importing from rostest.rostest
    from rostest.rostest_main import rostestmain as _main
    _main()
