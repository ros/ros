"""
XML Test Runner for PyUnit
"""

# Written by Sebastian Rittau <srittau@jroger.in-berlin.de> and placed in
# the Public Domain. With contributions by Paolo Borelli.

from __future__ import print_function

__revision__ = "$Id$"

import codecs
import os.path
import re
import sys
import time
import traceback
import unittest
try:
    from cStringIO import StringIO
    python2 = True
except ImportError:
    from io import StringIO
    python2 = False
from xml.sax.saxutils import escape
import xml.etree.ElementTree as ET

def cdata(cdata_text):
    return '<![CDATA[\n{}\n]]>'.format(cdata_text)

class _TestInfo(object):

    """Information about a particular test.
    
    Used by _XMLTestResult.
    
    """

    def __init__(self, test, time):
        (self._class, self._method) = test.id().rsplit(".", 1)
        self._time = time
        self._error = None
        self._failure = None

    @staticmethod
    def create_success(test, time):
        """Create a _TestInfo instance for a successful test."""
        return _TestInfo(test, time)

    @staticmethod
    def create_failure(test, time, failure):
        """Create a _TestInfo instance for a failed test."""
        info = _TestInfo(test, time)
        info._failure = failure
        return info

    @staticmethod
    def create_error(test, time, error):
        """Create a _TestInfo instance for an erroneous test."""
        info = _TestInfo(test, time)
        info._error = error
        return info

    def xml(self):
        """Create an XML tag with information about this test case.

        """
        testcase = ET.Element("testcase")
        testcase.set('classname', self._class)
        testcase.set('name', self._method)
        testcase.set('time', '%.4f' % self._time)
        if self._failure != None:
            self._print_error(testcase, 'failure', self._failure)
        if self._error != None:
            self._print_error(testcase, 'error', self._error)
        return testcase

    def print_report(self, stream):
        """Print information about this test case in XML format to the
        supplied stream.

        """
        stream.write(ET.tostring(self.xml()))

    def print_report_text(self, stream):
        #stream.write('  <testcase classname="%(class)s" name="%(method)s" time="%(time).4f">' % \
        #    {
        #        "class": self._class,
        #        "method": self._method,
        #        "time": self._time,
        #    })
        stream.write('[Testcase: ' + self._method + ']')
        if self._failure != None:
            stream.write(' ... FAILURE!\n')
            self._print_error_text(stream, 'failure', self._failure)
        if self._error != None:
            stream.write(' ... ERROR!\n')            
            self._print_error_text(stream, 'error', self._error)
        if self._failure == None and self._error == None:
            stream.write(' ... ok\n')

    def _print_error(self, testcase, tagname, error):
        """
        Append an XML tag with information from a failure or error to the
        supplied testcase.
        """
        tag = ET.SubElement(testcase, tagname)
        tag.set('type', str(error[0].__name__))
        tb_stream = StringIO()
        traceback.print_tb(error[2], None, tb_stream)
        tag.text ='%s\n%s' % (str(error[1]), tb_stream.getvalue())

    def _print_error_text(self, stream, tagname, error):
        """Print information from a failure or error to the supplied stream."""
        text = escape(str(error[1]))
        stream.write('%s: %s\n' \
            % (tagname.upper(), text))
        tb_stream = StringIO()
        traceback.print_tb(error[2], None, tb_stream)
        stream.write(escape(tb_stream.getvalue()))
        stream.write('-'*80 + '\n')

class _XMLTestResult(unittest.TestResult):

    """A test result class that stores result as XML.

    Used by XMLTestRunner.

    """

    def __init__(self, classname):
        unittest.TestResult.__init__(self)
        self._test_name = classname
        self._start_time = None
        self._tests = []
        self._error = None
        self._failure = None

    def startTest(self, test):
        unittest.TestResult.startTest(self, test)
        self._error = None
        self._failure = None
        self._start_time = time.time()

    def stopTest(self, test):
        time_taken = time.time() - self._start_time
        unittest.TestResult.stopTest(self, test)
        if self._error:
            info = _TestInfo.create_error(test, time_taken, self._error)
        elif self._failure:
            info = _TestInfo.create_failure(test, time_taken, self._failure)
        else:
            info = _TestInfo.create_success(test, time_taken)
        self._tests.append(info)

    def addError(self, test, err):
        unittest.TestResult.addError(self, test, err)
        self._error = err

    def addFailure(self, test, err):
        unittest.TestResult.addFailure(self, test, err)
        self._failure = err

    def filter_nonprintable_text(self, text):
        pattern = r'[^\x09\x0A\x0D\x20-\x7E\x85\xA0-\xFF\u0100-\uD7FF\uE000-\uFDCF\uFDE0-\uFFFD]'
        if python2:
            pattern = pattern.decode('unicode_escape')
        else:
            pattern = codecs.decode(pattern, 'unicode_escape')
        invalid_chars = re.compile(pattern)

        def invalid_char_replacer(m):
            return "&#x"+('%04X' % ord(m.group(0)))+";"
        return re.sub(invalid_chars, invalid_char_replacer, str(text))

    def xml(self, time_taken, out, err):
        """
        @return XML tag representing the object
        @rtype: xml.etree.ElementTree.Element
        """
        test_suite = ET.Element('testsuite')
        test_suite.set('errors', str(len(self.errors)))
        test_suite.set('failures', str(len(self.failures)))
        test_suite.set('name', self._test_name)
        test_suite.set('tests', str(self.testsRun))
        test_suite.set('time', '%.3f' % time_taken)
        for info in self._tests:
            test_suite.append(info.xml())
        system_out = ET.SubElement(test_suite, 'system-out')
        system_out.text = cdata(self.filter_nonprintable_text(out))
        system_err = ET.SubElement(test_suite, 'system-err')
        system_err.text = cdata(self.filter_nonprintable_text(err))
        return ET.ElementTree(test_suite)

    def print_report(self, stream, time_taken, out, err):
        """Prints the XML report to the supplied stream.
        
        The time the tests took to perform as well as the captured standard
        output and standard error streams must be passed in.a

        """
        root = self.xml(time_taken, out, err).getroot()
        stream.write(ET.tostring(root, encoding='utf-8', method='xml').decode('utf-8'))

    def print_report_text(self, stream, time_taken, out, err):
        """Prints the text report to the supplied stream.
        
        The time the tests took to perform as well as the captured standard
        output and standard error streams must be passed in.a

        """
        #stream.write('<testsuite errors="%(e)d" failures="%(f)d" ' % \
        #    { "e": len(self.errors), "f": len(self.failures) })
        #stream.write('name="%(n)s" tests="%(t)d" time="%(time).3f">\n' % \
        #    {
        #        "n": self._test_name,
        #        "t": self.testsRun,
        #        "time": time_taken,
        #    })
        for info in self._tests:
            info.print_report_text(stream)


class XMLTestRunner(object):

    """A test runner that stores results in XML format compatible with JUnit.

    XMLTestRunner(stream=None) -> XML test runner

    The XML file is written to the supplied stream. If stream is None, the
    results are stored in a file called TEST-<module>.<class>.xml in the
    current working directory (if not overridden with the path property),
    where <module> and <class> are the module and class name of the test class.

    """

    def __init__(self, stream=None):
        self._stream = stream
        self._path = "."

    def run(self, test):
        """Run the given test case or test suite."""
        class_ = test.__class__
        classname = class_.__module__ + "." + class_.__name__
        if self._stream == None:
            filename = "TEST-%s.xml" % classname
            stream = file(os.path.join(self._path, filename), "w")
            stream.write('<?xml version="1.0" encoding="utf-8"?>\n')
        else:
            stream = self._stream

        result = _XMLTestResult(classname)
        start_time = time.time()

        # TODO: Python 2.5: Use the with statement
        old_stdout = sys.stdout
        old_stderr = sys.stderr
        sys.stdout = StringIO()
        sys.stderr = StringIO()

        try:
            test(result)
            try:
                out_s = sys.stdout.getvalue()
            except AttributeError:
                out_s = ""
            try:
                err_s = sys.stderr.getvalue()
            except AttributeError:
                err_s = ""
        finally:
            sys.stdout = old_stdout
            sys.stderr = old_stderr

        time_taken = time.time() - start_time
        result.print_report(stream, time_taken, out_s, err_s)

        result.print_report_text(sys.stdout, time_taken, out_s, err_s)

        return result

    def _set_path(self, path):
        self._path = path

    path = property(lambda self: self._path, _set_path, None,
            """The path where the XML files are stored.
            
            This property is ignored when the XML file is written to a file
            stream.""")
