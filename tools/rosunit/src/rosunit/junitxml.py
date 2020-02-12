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
# Revision $Id$

"""
Library for reading and manipulating Ant JUnit XML result files.
"""

from __future__ import print_function

import codecs
import os
try:
    from cStringIO import StringIO
    python2 = True
except ImportError:
    from io import StringIO
    python2 = False
import re
import xml.etree.ElementTree as ET
from functools import reduce
from xml.dom import Node as DomNode
from xml.dom.minidom import parseString

import rospkg

pattern = r'[^\x09\x0A\x0D\x20-\x7E\x85\xA0-\xFF\u0100-\uD7FF\uE000-\uFDCF\uFDE0-\uFFFD]'
if python2:
    pattern = pattern.decode('unicode_escape')
else:
    pattern = codecs.decode(pattern, 'unicode_escape')
invalid_chars = re.compile(pattern)


def invalid_char_replacer(m):
    return '&#x' + ('%04X' % ord(m.group(0))) + ';'


def filter_nonprintable_text(text):
    return re.sub(invalid_chars, invalid_char_replacer, text)


def cdata(cdata_text):
    return '<![CDATA[\n{}\n]]>'.format(cdata_text)


class TestInfo(object):
    """
    Common container for 'error' and 'failure' results
    """

    def __init__(self, type_, text):
        """
        @param type_: type attribute from xml
        @type  type_: str
        @param text: text property from xml
        @type  text: str
        """
        self.type = type_
        self.text = text


class TestError(TestInfo):
    """
    'error' result container
    """
    def xml(self):
        """
        @return XML tag representing the object, with non-XML text filtered out
        @rtype: str
        """
        return ET.tostring(self.xml_element(), encoding='utf-8', method='xml')

    def xml_element(self):
        """
        @return XML tag representing the object, with non-XML text filtered out
        @rtype: xml.etree.ElementTree.Element
        """
        error = ET.Element('error')
        error.set('type', self.type)
        error.text = cdata(filter_nonprintable_text(self.text))
        return error


class TestFailure(TestInfo):
    """
    'failure' result container
    """
    def xml(self):
        """
        @return XML tag representing the object, with non-XML text filtered out
        @rtype: str
        """
        return ET.tostring(self.xml_element(), encoding='utf-8', method='xml')

    def xml_element(self):
        """
        @return XML tag representing the object, with non-XML text filtered out
        @rtype: xml.etree.ElementTree.Element
        """
        error = ET.Element('failure')
        error.set('type', self.type)
        error.text = cdata(filter_nonprintable_text(self.text))
        return error


class TestCaseResult(object):
    """
    'testcase' result container
    """

    def __init__(self, name):
        """
        @param name: name of testcase
        @type  name: str
        """
        self.name = name
        self.failures = []
        self.errors = []
        self.time = 0.0
        self.classname = ''

    def _passed(self):
        """
        @return: True if test passed
        @rtype: bool
        """
        return not self.errors and not self.failures
    # bool: True if test passed without errors or failures
    passed = property(_passed)

    def _failure_description(self):
        """
        @return: description of testcase failure
        @rtype: str
        """
        if self.failures:
            tmpl = '[%s][FAILURE]' % self.name
            tmpl = tmpl + '-'*(80 - len(tmpl))
            tmpl = tmpl+'\n%s\n' + '-' * 80 + '\n\n'
            return '\n'.join(tmpl % x.text for x in self.failures)
        return ''

    def _error_description(self):
        """
        @return: description of testcase error
        @rtype: str
        """
        if self.errors:
            tmpl = '[%s][ERROR]' % self.name
            tmpl = tmpl + '-' * (80 - len(tmpl))
            tmpl = tmpl+'\n%s\n' + '-' * 80 + '\n\n'
            return '\n'.join(tmpl % x.text for x in self.errors)
        return ''

    def _description(self):
        """
        @return: description of testcase result
        @rtype: str
        """
        if self.passed:
            return '[%s][passed]\n' % self.name
        else:
            return self._failure_description() + \
                   self._error_description()
    # str: printable description of testcase result
    description = property(_description)

    def add_failure(self, failure):
        """
        @param failure TestFailure
        """
        self.failures.append(failure)

    def add_error(self, error):
        """
        @param failure TestError
        """
        self.errors.append(error)

    def xml(self):
        """
        @return XML tag representing the object, with non-XML text filtered out
        @rtype: str
        """
        return ET.tostring(self.xml_element(), encoding='utf-8', method='xml')

    def xml_element(self):
        """
        @return XML tag representing the object, with non-XML text filtered out
        @rtype: xml.etree.ElementTree.Element
        """
        testcase = ET.Element('testcase')
        testcase.set('classname', self.classname)
        testcase.set('name', self.name)
        testcase.set('time', str(self.time))
        for f in self.failures:
            testcase.append(f.xml_element())
        for e in self.errors:
            testcase.append(e.xml_element())
        return testcase


class Result(object):
    __slots__ = ['name', 'num_errors', 'num_failures', 'num_tests',
                 'test_case_results', 'system_out', 'system_err', 'time']

    def __init__(self, name, num_errors=0, num_failures=0, num_tests=0):
        self.name = name
        self.num_errors = num_errors
        self.num_failures = num_failures
        self.num_tests = num_tests
        self.test_case_results = []
        self.system_out = ''
        self.system_err = ''
        self.time = 0.0

    def accumulate(self, r):
        """
        Add results from r to this result
        @param r: results to aggregate with this result
        @type  r: Result
        """
        self.num_errors += r.num_errors
        self.num_failures += r.num_failures
        self.num_tests += r.num_tests
        self.time += r.time
        self.test_case_results.extend(r.test_case_results)
        if r.system_out:
            self.system_out += '\n'+r.system_out
        if r.system_err:
            self.system_err += '\n'+r.system_err

    def add_test_case_result(self, r):
        """
        Add results from a testcase to this result container
        @param r: TestCaseResult
        @type  r: TestCaseResult
        """
        self.test_case_results.append(r)

    def xml_element(self):
        """
        @return: document as unicode (UTF-8 declared) XML according to Ant JUnit spec
        """
        testsuite = ET.Element('testsuite')
        testsuite.set('tests', str(self.num_tests))
        testsuite.set('failures', str(self.num_failures))
        testsuite.set('time', str(self.time))
        testsuite.set('errors', str(self.num_errors))
        testsuite.set('name', self.name)
        for tc in self.test_case_results:
            tc.xml(testsuite)
        system_out = ET.SubElement(testsuite, 'system-out')
        system_out.text = cdata(filter_nonprintable_text(self.system_out))
        system_err = ET.SubElement(testsuite, 'system-err')
        system_err.text = cdata(filter_nonprintable_text(self.system_err))
        return ET.tostring(testsuite, encoding='utf-8', method='xml')


def _text(tag):
    return reduce(lambda x, y: x + y, [c.data for c in tag.childNodes if c.nodeType in [DomNode.TEXT_NODE, DomNode.CDATA_SECTION_NODE]], '').strip()


def _load_suite_results(test_suite_name, test_suite, result):
    nodes = [n for n in test_suite.childNodes
             if n.nodeType == DomNode.ELEMENT_NODE]
    for node in nodes:
        name = node.tagName
        if name == 'testsuite':
            # for now we flatten this hierarchy
            _load_suite_results(test_suite_name, node, result)
        elif name == 'system-out':
            if _text(node):
                system_out = '[%s] stdout' % test_suite_name + '-' * (71 - len(test_suite_name))
                system_out += '\n'+_text(node)
                result.system_out += system_out
        elif name == 'system-err':
            if _text(node):
                system_err = '[%s] stderr' % test_suite_name + '-' * (71 - len(test_suite_name))
                system_err += '\n'+_text(node)
                result.system_err += system_err
        elif name == 'testcase':
            name = node.getAttribute('name') or 'unknown'
            classname = node.getAttribute('classname') or 'unknown'

            # mangle the classname for some sense of uniformity
            # between rostest/unittest/gtest
            if '__main__.' in classname:
                classname = classname[classname.find('__main__.') + 9:]
            if classname == 'rostest.rostest.RosTest':
                classname = 'rostest'
            elif not classname.startswith(result.name):
                classname = '%s.%s' % (result.name, classname)

            time = float(node.getAttribute('time')) or 0.0
            tc_result = TestCaseResult('%s/%s' % (test_suite_name, name))
            tc_result.classname = classname
            tc_result.time = time
            result.add_test_case_result(tc_result)
            for d in [n for n in node.childNodes
                      if n.nodeType == DomNode.ELEMENT_NODE]:
                # convert 'message' attributes to text elements to keep
                # python unittest and gtest consistent
                if d.tagName == 'failure':
                    message = d.getAttribute('message') or ''
                    text = _text(d) or message
                    x = TestFailure(d.getAttribute('type') or '', text)
                    tc_result.add_failure(x)
                elif d.tagName == 'error':
                    message = d.getAttribute('message') or ''
                    text = _text(d) or message
                    x = TestError(d.getAttribute('type') or '', text)
                    tc_result.add_error(x)


# #603: unit test suites are not good about screening out illegal
# unicode characters. This little recipe I from http://boodebr.org/main/python/all-about-python-and-unicode#UNI_XML
# screens these out
try:
    char = unichr
except NameError:
    char = chr
RE_XML_ILLEGAL = '([%s-%s%s-%s%s-%s%s-%s])' + \
                 '|' + \
                 '([%s-%s][^%s-%s])|([^%s-%s][%s-%s])|([%s-%s]$)|(^[%s-%s])'
try:
    RE_XML_ILLEGAL = unicode(RE_XML_ILLEGAL)
except NameError:
    pass
RE_XML_ILLEGAL = RE_XML_ILLEGAL % \
                 (char(0x0000), char(0x0008), char(0x000b), char(0x000c),
                  char(0x000e), char(0x001f), char(0xfffe), char(0xffff),
                  char(0xd800), char(0xdbff), char(0xdc00), char(0xdfff),
                  char(0xd800), char(0xdbff), char(0xdc00), char(0xdfff),
                  char(0xd800), char(0xdbff), char(0xdc00), char(0xdfff))
_safe_xml_regex = re.compile(RE_XML_ILLEGAL)


def _read_file_safe_xml(test_file, write_back_sanitized=True):
    """
    read in file, screen out unsafe unicode characters
    """
    f = None
    try:
        # this is ugly, but the files in question that are problematic
        # do not declare unicode type.
        if not os.path.isfile(test_file):
            raise Exception('test file does not exist')
        try:
            f = codecs.open(test_file, 'r', 'utf-8')
            x = f.read()
        except Exception:
            if f is not None:
                f.close()
            f = codecs.open(test_file, 'r', 'iso8859-1')
            x = f.read()

        for match in _safe_xml_regex.finditer(x):
            x = x[:match.start()] + '?' + x[match.end():]
        x = x.encode('utf-8')
        if write_back_sanitized:
            with open(test_file, 'wb') as h:
                h.write(x)
        return x
    finally:
        if f is not None:
            f.close()


def read(test_file, test_name):
    """
    Read in the test_result file
    @param test_file: test file path
    @type  test_file: str
    @param test_name: name of test
    @type  test_name: str
    @return: test results
    @rtype: Result
    """
    try:
        xml_str = _read_file_safe_xml(test_file)
        if not xml_str.strip():
            print('WARN: test result file is empty [%s]' % (test_file))
            return Result(test_name, 0, 0, 0)
        test_suites = parseString(xml_str).getElementsByTagName('testsuite')
    except Exception as e:
        print('WARN: cannot read test result file [%s]: %s' % (test_file, str(e)))
        return Result(test_name, 0, 0, 0)
    if not test_suites:
        print('WARN: test result file [%s] contains no results' % (test_file))
        return Result(test_name, 0, 0, 0)

    results = Result(test_name, 0, 0, 0)
    for index, test_suite in enumerate(test_suites):
        # skip test suites which are already covered by a parent test suite
        if index > 0 and test_suite.parentNode in test_suites[0:index]:
            continue

        # test_suite = test_suite[0]
        vals = [test_suite.getAttribute(attr) for attr in ['errors', 'failures', 'tests']]
        vals = [v or 0 for v in vals]
        err, fail, tests = [int(val) for val in vals]

        result = Result(test_name, err, fail, tests)
        result.time = 0.0 if not len(test_suite.getAttribute('time')) else float(test_suite.getAttribute('time'))

        # Create a prefix based on the test result filename. The idea is to
        # disambiguate the case when tests of the same name are provided in
        # different .xml files.  We use the name of the parent directory
        test_file_base = os.path.basename(os.path.dirname(os.path.abspath(test_file)))
        fname = os.path.basename(test_file)
        if fname.startswith('TEST-'):
            fname = fname[5:]
        if fname.endswith('.xml'):
            fname = fname[:-4]
        test_file_base = '%s.%s' % (test_file_base, fname)
        _load_suite_results(test_file_base, test_suite, result)
        results.accumulate(result)
    return results


def read_all(filter_=[]):
    """
    Read in the test_results and aggregate into a single Result object
    @param filter_: list of packages that should be processed
    @type filter_: [str]
    @return: aggregated result
    @rtype: L{Result}
    """
    dir_ = rospkg.get_test_results_dir()
    root_result = Result('ros', 0, 0, 0)
    if not os.path.exists(dir_):
        return root_result
    for d in os.listdir(dir_):
        if filter_ and d not in filter_:
            continue
        subdir = os.path.join(dir_, d)
        if os.path.isdir(subdir):
            for filename in os.listdir(subdir):
                if filename.endswith('.xml'):
                    filename = os.path.join(subdir, filename)
                    result = read(filename, os.path.basename(subdir))
                    root_result.accumulate(result)
    return root_result


def test_failure_junit_xml(test_name, message, stdout=None, class_name='Results', testcase_name='test_ran'):
    """
    Generate JUnit XML file for a unary test suite where the test failed

    @param test_name: Name of test that failed
    @type  test_name: str
    @param message: failure message
    @type  message: str
    @param stdout: stdout data to include in report
    @type  stdout: str
    """
    testsuite = ET.Element('testsuite')
    testsuite.set('tests', '1')
    testsuite.set('failures', '1')
    testsuite.set('time', '1')
    testsuite.set('errors', '0')
    testsuite.set('name', test_name)
    testcase = ET.SubElement(testsuite, 'testcase')
    testcase.set('name', testcase_name)
    testcase.set('status', 'run')
    testcase.set('time', '1')
    testcase.set('classname', class_name)
    failure = ET.SubElement(testcase, 'failure')
    failure.set('message', message)
    failure.set('type', '')
    if stdout:
        system_out = ET.SubElement(testsuite, 'system-out')
        system_out.text = cdata(filter_nonprintable_text(stdout))
    return ET.tostring(testsuite, encoding='utf-8', method='xml')


def test_success_junit_xml(test_name, class_name='Results', testcase_name='test_ran'):
    """
    Generate JUnit XML file for a unary test suite where the test succeeded.

    @param test_name: Name of test that passed
    @type  test_name: str
    """
    testsuite = ET.Element('testsuite')
    testsuite.set('tests', '1')
    testsuite.set('failures', '0')
    testsuite.set('time', '1')
    testsuite.set('errors', '0')
    testsuite.set('name', test_name)
    testcase = ET.SubElement(testsuite, 'testcase')
    testcase.set('name', testcase_name)
    testcase.set('status', 'run')
    testcase.set('time', '1')
    testcase.set('classname', class_name)
    return ET.tostring(testsuite, encoding='utf-8', method='xml')


def print_summary(junit_results, runner_name='ROSUNIT'):
    """
    Print summary of junitxml results to stdout.
    """
    # we have two separate result objects, which can be a bit
    # confusing. 'result' counts successful _running_ of tests
    # (i.e. doesn't check for actual test success). The 'r' result
    # object contains results of the actual tests.

    buff = StringIO()
    buff.write('[%s]' % runner_name + '-' * 71 + '\n\n')
    for tc_result in junit_results.test_case_results:
        buff.write(tc_result.description)

    buff.write('\nSUMMARY\n')
    if (junit_results.num_errors + junit_results.num_failures) == 0:
        buff.write('\033[32m * RESULT: SUCCESS\033[0m\n')
    else:
        buff.write('\033[1;31m * RESULT: FAIL\033[0m\n')

    # TODO: still some issues with the numbers adding up if tests fail to launch

    # number of errors from the inner tests, plus add in count for tests
    # that didn't run properly ('result' object).
    buff.write(' * TESTS: %s\n' % junit_results.num_tests)
    num_errors = junit_results.num_errors
    if num_errors:
        buff.write('\033[1;31m * ERRORS: %s\033[0m\n' % num_errors)
    else:
        buff.write(' * ERRORS: 0\n')
    num_failures = junit_results.num_failures
    if num_failures:
        buff.write('\033[1;31m * FAILURES: %s\033[0m\n' % num_failures)
    else:
        buff.write(' * FAILURES: 0\n')

    print(buff.getvalue())
