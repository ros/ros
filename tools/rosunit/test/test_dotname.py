#!/usr/bin/env python

# This file should be run using a non-ros unit test framework such as nose using
# nosetests test_dotname.py. Alternatively, just run with python test_dotname.py.
# You will get the output from rostest as well.

import unittest
import rosunit
from dotname_cases import DotnameLoadingTest, NotTestCase


class TestDotnameLoading(unittest.TestCase):

    def test_class_basic(self):
        rosunit.unitrun('test_rosunit', 'test_class_basic', DotnameLoadingTest)

    def test_class_dotname(self):
        rosunit.unitrun('test_rosunit', 'test_class_dotname', 'test.dotname_cases.DotnameLoadingTest')

    def test_method_dotname(self):
        rosunit.unitrun('test_rosunit', 'test_method_dotname', 'test.dotname_cases.DotnameLoadingTest.test_a')

    def test_suite_dotname(self):
        rosunit.unitrun('test_rosunit', 'test_suite_dotname', 'test.dotname_cases.DotnameLoadingSuite')

    def test_class_basic_nottest(self):
        # class which exists but is not a TestCase
        with self.assertRaises(SystemExit):
            rosunit.unitrun('test_rosunit', 'test_class_basic_nottest', NotTestCase)

    def test_class_dotname_nottest(self):
        # class which exists but is not a valid test
        with self.assertRaises(TypeError):
            rosunit.unitrun('test_rosunit', 'test_class_dotname_nottest', 'test.dotname_cases.NotTestCase')

    def test_class_dotname_noexist(self):
        # class which does not exist in the module
        with self.assertRaises(AttributeError):
            rosunit.unitrun('test_rosunit', 'test_class_dotname_noexist', 'test.dotname_cases.DotnameLoading')

    def test_method_dotname_noexist(self):
        # method which does not exist in the class
        with self.assertRaises(AttributeError):
            rosunit.unitrun('test_rosunit', 'test_method_dotname_noexist', 'test.dotname_cases.DotnameLoadingTest.not_method')


if __name__ == '__main__':
    unittest.main()
