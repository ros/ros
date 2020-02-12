#!/usr/bin/env python
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
# Revision $Id: $

import os
import shutil
import tempfile
import unittest

junitxml = None


# Basic test of xmlresult functionality of reading gtest xml files and
# summarizing their results into a new file.
class MockResult():

    def __init__(self, directory, filename, suites=[], noSuitesRoot=False):
        self.filename = os.path.join(directory, filename)
        self.suites = suites
        # whether to suppress <testsuites> root node
        self.noSuitesRoot = noSuitesRoot


class MockSuite():

    def __init__(self, cases, name, tests=0, errors=0, fail=0, time=1):
        self.cases = cases
        self.tests = tests
        self.time = time
        self.fail = fail
        self.errors = errors
        self.name = name


class MockCase():

    def __init__(self, name, errorList=[], classname='', time=1):
        self.classname = classname
        self.name = name
        self.time = time
        self.errorList = errorList


class MockErrorType(Exception):

    def __init__(self, value, etype=''):
        self.value = value
        self.__name__ = value
        self.type = etype


def _writeMockResultFile(result):
    """writes a test result as a gtest compatible test runner would do"""
    with open(result.filename, 'w') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        if len(result.suites) > 1 or result.noSuitesRoot is False:
            f.write('<testsuites>\n')
        for suite in result.suites:
            f.write('<testsuite tests="'+str(suite.tests)+'" failures="'+str(suite.fail)+'" time="'+str(suite.time)+'" errors="'+str(suite.errors)+'" name="'+suite.name+'">\n')
            for case in suite.cases:
                f.write('<testcase name="'+case.name+'" status="run" time="'+str(case.time)+'" classname="'+case.classname+'">\n')
                for error in case.errorList:
                    f.write('<failure message="'+error.value+'" type="'+error.value+'"/>\n')
                f.write('</testcase>\n')
            f.write('</testsuite>\n')
        if len(result.suites) > 1 or result.noSuitesRoot is False:
            f.write('</testsuites>\n')


class XmlResultTestGeneration(unittest.TestCase):

    def setUp(self):
        global junitxml
        if junitxml is None:
            import rosunit.junitxml
            junitxml = rosunit.junitxml

    def tearDown(self):
        pass

    def testGenerateError(self):
        error = junitxml.TestError('error_type', 'error_text')
        error_str = error.xml()
        self.assertEquals(b"""<error type="error_type">&lt;![CDATA[
error_text
]]&gt;</error>""", error_str)

    def testGenerateFailure(self):
        failure = junitxml.TestFailure('failure_type', 'failure_text')
        failure_str = failure.xml()
        self.assertEquals(b"""<failure type="failure_type">&lt;![CDATA[
failure_text
]]&gt;</failure>""", failure_str)

    def testGenerateTestCaseResult(self):
        testcase = junitxml.TestCaseResult('test_case')
        error = junitxml.TestError('error_type', 'error_text')
        error_str = error.xml()
        failure = junitxml.TestFailure('failure_type', 'failure_text')
        failure_str = failure.xml()
        testcase.add_error(error)
        testcase.add_failure(failure)
        testcase_str = testcase.xml()
        self.assertEquals(b"""<testcase classname="" name="test_case" time="0.0"><failure type="failure_type">&lt;![CDATA[
failure_text
]]&gt;</failure><error type="error_type">&lt;![CDATA[
error_text
]]&gt;</error></testcase>""", testcase_str)


class XmlResultTestRead(unittest.TestCase):

    def setUp(self):
        # lazy-import to get coverage
        global junitxml
        if junitxml is None:
            import rosunit.junitxml
            junitxml = rosunit.junitxml

        self.directory = tempfile.mkdtemp()

        # setting up mock results as dict so results can be checked individually
        self.mockresults = {
            'empty': MockResult(self.directory, 'empty.xml', []),
            'emptysuite': MockResult(self.directory, 'emptysuite.xml', [MockSuite([], 'emptySuite', 0, 0, 0, 0)]),
            'succ1': MockResult(self.directory, 'succ1.xml', [MockSuite([MockCase('succCase')], 'succ1suite', 1, 0, 0, 1)]),
            'err1': MockResult(self.directory, 'err1.xml',  [MockSuite([MockCase('errCase')], 'err1suite', 1, 1, 0, 1)]),
            'fail1': MockResult(self.directory, 'fail1.xml', [MockSuite([MockCase('failCase')], 'fail1suite', 1, 0, 1, 1)]),
            'noroot': MockResult(self.directory, 'succ1.xml', [MockSuite([MockCase('succCase')], 'succ1suite', 1, 0, 0, 1)], noSuitesRoot=True),
            'multicase': MockResult(self.directory,
                                    'multicase.xml',
                                    [MockSuite([MockCase('succCase'),
                                                MockCase('errCase'),
                                                MockCase('failCase')],
                                               'succ1suite', 3, 1, 1, time=3)]),
            'multisuite': MockResult(self.directory,
                                     'multisuite.xml',
                                     [MockSuite([MockCase('succCase')], 'succ1suite', 1, 0, 0, 1),
                                      MockSuite([MockCase('errCase')], 'err1suite', 1, 1, 0, 1),
                                      MockSuite([MockCase('failCase')], 'fail1suite', 1, 0, 1, 1)])
            }

        for name, result in self.mockresults.items():
            _writeMockResultFile(result)

    def tearDown(self):
        shutil.rmtree(self.directory)
        # pass

    def testReadNoSuites(self):
        result = junitxml.read(self.mockresults['empty'].filename, 'fooname')
        self.assert_(result is not None)
        self.assertEquals(0.0, result.time)
        self.assertEquals(0, result.num_tests)
        self.assertEquals(0, result.num_errors)
        self.assertEquals(0, result.num_failures)

    def testReadEmptySuite(self):
        result = junitxml.read(self.mockresults['emptysuite'].filename, 'fooname')
        self.assert_(result is not None)
        self.assertEquals(0.0, result.time)
        self.assertEquals(0, result.num_tests)
        self.assertEquals(0, result.num_errors)
        self.assertEquals(0, result.num_failures)

    def testReadSuccess(self):
        result = junitxml.read(self.mockresults['succ1'].filename, 'fooname')
        self.assert_(result is not None)
        self.assertEquals(1.0, result.time)
        self.assertEquals(1, result.num_tests)
        self.assertEquals(0, result.num_errors)
        self.assertEquals(0, result.num_failures)

    def testReadError(self):
        result = junitxml.read(self.mockresults['err1'].filename, 'fooname')
        self.assert_(result is not None)
        self.assertEquals(1.0, result.time)
        self.assertEquals(1, result.num_tests)
        self.assertEquals(1, result.num_errors)
        self.assertEquals(0, result.num_failures)

    def testReadFail(self):
        result = junitxml.read(self.mockresults['fail1'].filename, 'fooname')
        self.assert_(result is not None)
        self.assertEquals(1.0, result.time)
        self.assertEquals(1, result.num_tests)
        self.assertEquals(0, result.num_errors)
        self.assertEquals(1, result.num_failures)

    def testReadMulticase(self):
        result = junitxml.read(self.mockresults['multicase'].filename, 'fooname')
        self.assert_(result is not None)
        self.assertEquals(3.0, result.time)
        self.assertEquals(3, result.num_tests)
        self.assertEquals(1, result.num_errors)
        self.assertEquals(1, result.num_failures)

    def testReadMultisuite(self):
        result = junitxml.read(self.mockresults['multisuite'].filename, 'fooname')
        self.assert_(result is not None)
        self.assertEquals(3.0, result.time)
        self.assertEquals(3, result.num_tests)
        self.assertEquals(1, result.num_errors)
        self.assertEquals(1, result.num_failures)
