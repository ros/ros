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
Wrapper for running Python unittest within rosunit/rostest framework.
"""
from __future__ import with_statement, print_function

import sys

from .core import create_xml_runner, XML_OUTPUT_FLAG
from .baretest import print_unittest_summary

def unitrun(package, test_name, test, sysargs=None, coverage_packages=None):
    """
    Wrapper routine from running python unitttests with
    JUnit-compatible XML output.  This is meant for unittests that do
    not not need a running ROS graph (i.e. offline tests only).

    This enables JUnit-compatible test reporting so that
    test results can be reported to higher-level tools.

    WARNING: unitrun() will trigger a sys.exit() on test failure in
    order to properly exit with an error code. This routine is meant
    to be used as a main() routine, not as a library.
    
    @param package: name of ROS package that is running the test
    @type  package: str
    @param coverage_packages: list of Python package to compute coverage results for. Defaults to package
    @type  coverage_packages: [str]
    @param sysargs: (optional) alternate sys.argv
    @type  sysargs: [str]
    """
    if sysargs is None:
        # lazy-init sys args
        import sys
        sysargs = sys.argv

    import unittest
    
    if coverage_packages is None:
        coverage_packages = [package]
        
    #parse sysargs
    result_file = None
    for arg in sysargs:
        if arg.startswith(XML_OUTPUT_FLAG):
            result_file = arg[len(XML_OUTPUT_FLAG):]
    text_mode = '--text' in sysargs

    coverage_mode = '--cov' in sysargs or '--covhtml' in sysargs
    if coverage_mode:
        start_coverage(coverage_packages)

    # create and run unittest suite with our xmllrunner wrapper
    suite = unittest.TestLoader().loadTestsFromTestCase(test)
    if text_mode:
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    else:
        result = create_xml_runner(package, test_name, result_file).run(suite)
    if coverage_mode:
        cov_html_dir = 'covhtml' if '--covhtml' in sysargs else None
        stop_coverage(coverage_packages, html=cov_html_dir)

    # test over, summarize results and exit appropriately
    print_unittest_summary(result)
    
    if not result.wasSuccessful():
        import sys
        sys.exit(1)

# coverage instance
_cov = None
def start_coverage(packages):
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
    except ImportError, e:
        print("""WARNING: cannot import python-coverage, coverage tests will not run.
To install coverage, run 'easy_install coverage'""", file=sys.stderr)

def stop_coverage(packages, html=None):
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
        except ImportError, e:
            print("WARNING: cannot import '%s', will not generate coverage report"%package, file=sys.stderr)
    except ImportError, e:
        print("""WARNING: cannot import python-coverage, coverage tests will not run.
To install coverage, run 'easy_install coverage'""", file=sys.stderr)
    
    
