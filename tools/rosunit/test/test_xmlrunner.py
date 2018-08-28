#!/usr/bin/env python

# This file should be run using a non-ros unit test framework such as nose using
# nosetests test_dotname.py. Alternatively, just run with python test_dotname.py.
# You will get the output from rostest as well.

# Original code for xmlrunner written by Sebastian Rittau 
# <srittau@jroger.in-berlin.de> and placed in the Public Domain.
# With contributions by Paolo Borelli.
# These tests refactored into a separate package by Edward Venator.

from __future__ import print_function
import re
import sys
import unittest
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
from rosunit.xmlrunner import XMLTestRunner

class XMLTestRunnerTest(unittest.TestCase):
    def setUp(self):
        self._stream = StringIO()

    def _try_test_run(self, test_class, expected):

        """Run the test suite against the supplied test class and compare the
        XML result against the expected XML string. Fail if the expected
        string doesn't match the actual string. All time attribute in the
        expected string should have the value "0.000". All error and failure
        messages are reduced to "Foobar".

        """

        runner = XMLTestRunner(self._stream)
        runner.run(unittest.makeSuite(test_class))

        got = self._stream.getvalue()
        # Replace all time="X.YYY" attributes by time="0.000" to enable a
        # simple string comparison.
        got = re.sub(r'time="\d+\.\d+"', 'time="0.000"', got)
        # Likewise, replace all failure and error messages by a simple "Foobar"
        # string.
        got = re.sub(r'(?s)<failure (.*?)>.*?</failure>', r'<failure \1>Foobar</failure>', got)
        got = re.sub(r'(?s)<error (.*?)>.*?</error>', r'<error \1>Foobar</error>', got)

        self.assertIn(got, expected)

    def test_no_tests(self):
        """Regression test: Check whether a test run without any tests
        matches a previous run.
        
        """
        class TestTest(unittest.TestCase):
            pass
        self._try_test_run(TestTest, ["""<testsuite errors="0" failures="0" name="unittest.suite.TestSuite" tests="0" time="0.000"><system-out>&lt;![CDATA[\n\n]]&gt;</system-out><system-err>&lt;![CDATA[\n\n]]&gt;</system-err></testsuite>"""])

    def test_success(self):
        """Regression test: Check whether a test run with a successful test
        matches a previous run.
        
        """
        class TestTest(unittest.TestCase):
            def test_foo(self):
                pass
        py2_expected = """<testsuite errors="0" failures="0" name="unittest.suite.TestSuite" tests="1" time="0.000"><testcase classname="test.test_xmlrunner.TestTest" name="test_foo" time="0.000" /><system-out>&lt;![CDATA[\n\n]]&gt;</system-out><system-err>&lt;![CDATA[\n\n]]&gt;</system-err></testsuite>"""
        py3_expected = py2_expected.replace('TestTest', 'XMLTestRunnerTest.test_success.&lt;locals&gt;.TestTest')
        self._try_test_run(TestTest, [py2_expected, py3_expected])

    def test_failure(self):
        """Regression test: Check whether a test run with a failing test
        matches a previous run.
        
        """
        class TestTest(unittest.TestCase):
            def test_foo(self):
                self.assert_(False)
        py2_expected = """<testsuite errors="0" failures="1" name="unittest.suite.TestSuite" tests="1" time="0.000"><testcase classname="test.test_xmlrunner.TestTest" name="test_foo" time="0.000"><failure type="AssertionError">Foobar</failure></testcase><system-out>&lt;![CDATA[\n\n]]&gt;</system-out><system-err>&lt;![CDATA[\n\n]]&gt;</system-err></testsuite>"""
        py3_expected = py2_expected.replace('TestTest', 'XMLTestRunnerTest.test_failure.&lt;locals&gt;.TestTest')
        self._try_test_run(TestTest, [py2_expected, py3_expected])

    def test_error(self):
        """Regression test: Check whether a test run with a erroneous test
        matches a previous run.
        
        """
        class TestTest(unittest.TestCase):
            def test_foo(self):
                raise IndexError()
        py2_expected = """<testsuite errors="1" failures="0" name="unittest.suite.TestSuite" tests="1" time="0.000"><testcase classname="test.test_xmlrunner.TestTest" name="test_foo" time="0.000"><error type="IndexError">Foobar</error></testcase><system-out>&lt;![CDATA[\n\n]]&gt;</system-out><system-err>&lt;![CDATA[\n\n]]&gt;</system-err></testsuite>"""
        py3_expected = py2_expected.replace('TestTest', 'XMLTestRunnerTest.test_error.&lt;locals&gt;.TestTest')
        self._try_test_run(TestTest, [py2_expected, py3_expected])

    def test_stdout_capture(self):
        """Regression test: Check whether a test run with output to stdout
        matches a previous run.
        
        """
        class TestTest(unittest.TestCase):
            def test_foo(self):
                print("Foo > Bar")
        py2_expected = """<testsuite errors="0" failures="0" name="unittest.suite.TestSuite" tests="1" time="0.000"><testcase classname="test.test_xmlrunner.TestTest" name="test_foo" time="0.000" /><system-out>&lt;![CDATA[\nFoo &gt; Bar\n\n]]&gt;</system-out><system-err>&lt;![CDATA[\n\n]]&gt;</system-err></testsuite>"""
        py3_expected = py2_expected.replace('TestTest', 'XMLTestRunnerTest.test_stdout_capture.&lt;locals&gt;.TestTest')
        self._try_test_run(TestTest, [py2_expected, py3_expected])

    def test_stderr_capture(self):
        """Regression test: Check whether a test run with output to stderr
        matches a previous run.
        
        """
        class TestTest(unittest.TestCase):
            def test_foo(self):
                print("Foo > Bar", file=sys.stderr)
        py2_expected = """<testsuite errors="0" failures="0" name="unittest.suite.TestSuite" tests="1" time="0.000"><testcase classname="test.test_xmlrunner.TestTest" name="test_foo" time="0.000" /><system-out>&lt;![CDATA[\n\n]]&gt;</system-out><system-err>&lt;![CDATA[\nFoo &gt; Bar\n\n]]&gt;</system-err></testsuite>"""
        py3_expected = py2_expected.replace('TestTest', 'XMLTestRunnerTest.test_stderr_capture.&lt;locals&gt;.TestTest')
        self._try_test_run(TestTest, [py2_expected, py3_expected])

    class NullStream(object):
        """A file-like object that discards everything written to it."""
        def write(self, buffer):
            pass

    def test_unittests_changing_stdout(self):
        """Check whether the XMLTestRunner recovers gracefully from unit tests
        that change stdout, but don't change it back properly.

        """
        class TestTest(unittest.TestCase):
            def test_foo(self):
                sys.stdout = XMLTestRunnerTest.NullStream()

        runner = XMLTestRunner(self._stream)
        runner.run(unittest.makeSuite(TestTest))

    def test_unittests_changing_stderr(self):
        """Check whether the XMLTestRunner recovers gracefully from unit tests
        that change stderr, but don't change it back properly.

        """
        class TestTest(unittest.TestCase):
            def test_foo(self):
                sys.stderr = XMLTestRunnerTest.NullStream()

        runner = XMLTestRunner(self._stream)
        runner.run(unittest.makeSuite(TestTest))


class XMLTestProgram(unittest.TestProgram):
    def runTests(self):
        if self.testRunner is None:
            self.testRunner = XMLTestRunner()
        unittest.TestProgram.runTests(self)

main = XMLTestProgram


if __name__ == "__main__":
    main(module=None)
