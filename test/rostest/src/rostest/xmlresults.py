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

import os
import sys
import string
from xml.dom.minidom import parse, parseString
from xml.dom import Node as DomNode

import roslib.rosenv

## Common container for 'error' and 'failure' results
class _TestInfo(object):
    ## @param type str: type attribute from xml 
    ## @param text str: text property from xml
    def __init__(self, type, text):
        self.type = type
        self.text = text
## 'error' result container        
class TestError(_TestInfo): 
    def xml(self):
        return u'<error type="%s"><![CDATA[%s]]></error>'%(self.type, self.text)            

## 'failure' result container        
class TestFailure(_TestInfo):
    def xml(self):
        return u'<failure type="%s"><![CDATA[%s]]></failure>'%(self.type, self.text)            


## 'testcase' result container
class TestCaseResult(object):
    ## @param name str: name of testcase
    def __init__(self, name):
        self.name = name
        self.failures = []
        self.errors = []
        self.time = 0.0
        self.classname = ''
        
    ## @return bool: True if test passed
    def _passed(self):
        return not self.errors and not self.failures
    ## bool: True if test passed without errors or failures
    passed = property(_passed)
    
    ## @return str: description of testcase failure
    def _failure_description(self):
        if self.failures:
            tmpl = "[%s][FAILURE]"%self.name
            tmpl = tmpl + '-'*(80-len(tmpl))
            tmpl = tmpl+"\n%s\n"+'-'*80+"\n\n"
            return '\n'.join(tmpl%x.text for x in self.failures)
        return ''
    ## @return str: description of testcase error
    def _error_description(self):
        if self.errors:
            tmpl = "[%s][ERROR]"%self.name
            tmpl = tmpl + '-'*(80-len(tmpl))
            tmpl = tmpl+"\n%s\n"+'-'*80+"\n\n"
            return '\n'.join(tmpl%x.text for x in self.errors)
        return ''
    ## @return str: description of testcase result
    def _description(self):
        if self.passed:
            return "[%s][passed]\n"%self.name
        else:
            return self._failure_description()+\
                   self._error_description()                   
    ## str: printable description of testcase result
    description = property(_description)
    ## @param failure TestFailure
    def add_failure(self, failure):
        self.failures.append(failure)
    ## @param failure TestError        
    def add_error(self, error):
        self.errors.append(error)

    def xml(self):
        return u'  <testcase classname="%s" name="%s" time="%s">\n'%(self.classname, self.name, self.time)+\
               '\n    '.join([f.xml() for f in self.failures])+\
               '\n    '.join([e.xml() for e in self.errors])+\
               '  </testcase>'
        
class Result(object):
    __slots__ = ['name', 'num_errors', 'num_failures', 'num_tests', \
                 'test_case_results', 'system_out', 'system_err', 'time']
    def __init__(self, name, num_errors, num_failures, num_tests):
        self.name = name
        self.num_errors = num_errors
        self.num_failures = num_failures
        self.num_tests = num_tests
        self.test_case_results = []
        self.system_out = ''
        self.system_err = ''
        self.time = 0.0

    ## Add results from \a r to this result
    ## @param r Result: results to aggregate with this result
    def accumulate(self, r):
        self.num_errors += r.num_errors
        self.num_failures += r.num_failures
        self.num_tests += r.num_tests
        self.test_case_results.extend(r.test_case_results)
        if r.system_out:
            self.system_out += '\n'+r.system_out
        if r.system_err:
            self.system_err += '\n'+r.system_err

    ## Add results from a testcase to this result container
    ## @param r TestCaseResult
    def add_test_case_result(self, r):
        self.test_case_results.append(r)

    ## @return document as unicode (UTF-8 declared) XML
    def xml(self):
        return u'<?xml version="1.0" encoding="utf-8"?>'+\
               '<testsuite name="%s" tests="%s" errors="%s" failures="%s" time="%s">'%\
               (self.name, self.num_tests, self.num_errors, self.num_failures, self.time)+\
               '\n'.join([tc.xml() for tc in self.test_case_results])+\
               '  <system-out><![CDATA[%s]]></system-out>'%self.system_out+\
               '  <system-err><![CDATA[%s]]></system-err>'%self.system_err+\
               '</testsuite>'

def _text(tag):
    return reduce(lambda x, y: x + y, [c.data for c in tag.childNodes if c.nodeType in [DomNode.TEXT_NODE, DomNode.CDATA_SECTION_NODE]], "").strip()

def _load_suite_results(test_suite_name, test_suite, result):
    nodes = [n for n in test_suite.childNodes \
             if n.nodeType == DomNode.ELEMENT_NODE]
    for node in nodes:
        name = node.tagName
        if name == 'testsuite':
            # for now we flatten this hierarchy
            _load_suite_results(test_suite_name, node, result)
        elif name == 'system-out':
            if _text(node):
                system_out = "[%s] stdout"%test_suite_name + "-"*(71-len(test_suite_name))
                system_out += '\n'+_text(node)
                result.system_out += system_out
        elif name == 'system-err':
            if _text(node):
                system_err = "[%s] stderr"%test_suite_name + "-"*(71-len(test_suite_name))
                system_err += '\n'+_text(node)
                result.system_err += system_err
        elif name == 'testcase':
            name = node.getAttribute('name') or 'unknown'
            classname = node.getAttribute('classname') or 'unknown'

            # mangle the classname for some sense of uniformity
            # between rostest/unittest/gtest
            if '__main__.' in classname:
              classname = classname[classname.find('__main__.')+9:]
            if classname == 'rostest.rostest.RosTest':
              classname = 'rostest'
            elif not classname.startswith(result.name):
              classname = "%s.%s"%(result.name,classname)
              
            time = node.getAttribute('time') or 0.0
            tc_result = TestCaseResult("%s/%s"%(test_suite_name,name))
            tc_result.classname = classname
            tc_result.time = time            
            result.add_test_case_result(tc_result)
            for d in [n for n in node.childNodes \
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

## #603: unit test suites are not good about screening out illegal
## unicode characters. This little recipe I from http://boodebr.org/main/python/all-about-python-and-unicode#UNI_XML
## screens these out
import re
RE_XML_ILLEGAL = u'([\u0000-\u0008\u000b-\u000c\u000e-\u001f\ufffe-\uffff])' + \
                 u'|' + \
                 u'([%s-%s][^%s-%s])|([^%s-%s][%s-%s])|([%s-%s]$)|(^[%s-%s])' % \
                 (unichr(0xd800),unichr(0xdbff),unichr(0xdc00),unichr(0xdfff),
                  unichr(0xd800),unichr(0xdbff),unichr(0xdc00),unichr(0xdfff),
                  unichr(0xd800),unichr(0xdbff),unichr(0xdc00),unichr(0xdfff))
_safe_xml_regex = re.compile(RE_XML_ILLEGAL)

## read in file, screen out unsafe unicode characters
def _read_file_safe_xml(test_file):
    import codecs
    try:
        # this is ugly, but the files in question that are problematic
        # do not declare unicode type.
        try:
            f = codecs.open(test_file, "r", "utf-8" )
            x = f.read()
        except:
            f.close()
            f = codecs.open(test_file, "r", "iso8859-1" )
            x = f.read()        

        for match in _safe_xml_regex.finditer(x):
            x = x[:match.start()] + "?" + x[match.end():]
        return x.encode("utf-8")
    finally:
        f.close()

## Read in the test_result file
## @param test_file str: test file path
## @param test_name str: name of test                    
## @return Result test results
def read(test_file, test_name):
    try:
        xml_str = _read_file_safe_xml(test_file)
        test_suite = parseString(xml_str).getElementsByTagName('testsuite')
    except Exception, e:
        import traceback
        traceback.print_exc()
        print "WARN: cannot read test result file [%s]: %s"%(test_file, str(e))
        return Result(test_name, 0, 0, 0)
    if not test_suite:
        print "WARN: test result file [%s] contains no results"%test_file
        return Result(test_name, 0, 0, 0)
    test_suite = test_suite[0]
    vals = [test_suite.getAttribute(attr) for attr in ['errors', 'failures', 'tests']]
    vals = [v or 0 for v in vals]
    err, fail, tests = [string.atoi(val) for val in vals]

    result = Result(test_name, err, fail, tests)
    result.time = test_suite.getAttribute('time') or 0.0    

    # Create a prefix based on the test result filename. The idea is to
    # disambiguate the case when tests of the same name are provided in
    # different .xml files.  We use the name of the parent directory
    test_file_base = os.path.basename(os.path.dirname(test_file))
    fname = os.path.basename(test_file)
    if fname.startswith('TEST-'):
        fname = fname[5:]
    if fname.endswith('.xml'):
        fname = fname[:-4]
    test_file_base = "%s.%s"%(test_file_base, fname)
    _load_suite_results(test_file_base, test_suite, result)
    return result

def read_all(filter=[]):
    """
    Read in the test_results and aggregate into a single Result object
    @param filter: list of packages that should be processed
    @type filter: [str]
    @return: aggregated result
    @rtype: L{Result}
    """
    dir = roslib.rosenv.get_test_results_dir()
    root_result = Result('ros', 0, 0, 0)
    for d in os.listdir(dir):
        if filter and not d in filter:
            continue
        subdir = os.path.join(dir, d)
        if os.path.isdir(subdir):
            for file in os.listdir(subdir):
                if file.endswith('.xml'):
                    file = os.path.join(subdir, file)
                    result = read(file, os.path.basename(subdir))
                    root_result.accumulate(result)
    return root_result
