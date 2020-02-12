#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

from __future__ import print_function

"""
clean_junit_xml.py is a simple script that takes all the xml-formatted
Ant JUnit XML test output in test_results and aggregates them into
test_results/_hudson. In this process, it strips any characters that
tend to cause Hudson trouble.
"""

import os
import sys

import rospkg

import rosunit.junitxml as junitxml

PKG = 'rosunit'


def prepare_dirs(output_dir_name):
    test_results_dir = rospkg.get_test_results_dir()
    print('will read test results from', test_results_dir)
    output_dir = os.path.join(test_results_dir, output_dir_name)
    if not os.path.exists(output_dir):
        print('creating directory', output_dir)
        os.makedirs(output_dir)
    return test_results_dir, output_dir


def clean_results(test_results_dir, output_dir, filter):
    """
    Read results from test_results_dir and write them into output_dir.
    """
    for d in os.listdir(test_results_dir):
        if filter and d in filter:
            continue
        print('looking at', d)
        test_dir = os.path.join(test_results_dir, d)
        if not os.path.isdir(test_dir):
            continue
        base_test_name = os.path.basename(test_dir)
        # for each test result that a package generated, read it, then
        # rewrite it to our output directory. This will invoke our
        # cleaning rules on the XML that protect the result from Hudson
        # issues.
        for file in os.listdir(test_dir):
            if file.endswith('.xml'):
                test_name = base_test_name + '.' + file[:-4]
            file = os.path.join(test_dir, file)
            try:
                result = junitxml.read(file, test_name)
                output_path = os.path.join(output_dir, '%s.xml' % test_name)
                with open(output_path, 'w') as f:
                    print('re-writing', output_path)
                    f.write(result.xml().encode('utf-8'))
            except Exception as e:
                sys.stderr.write('ignoring [%s]: %s\n' % (file, e))


def main():

    print('[clean_junit_xml]: STARTING')

    output_dir_name = '_hudson'
    test_results_dir, output_dir = prepare_dirs(output_dir_name)

    print('[clean_junit_xml]: writing aggregated test results to %s' % output_dir)

    clean_results(test_results_dir, output_dir, [output_dir_name, '.svn'])

    print('[clean_junit_xml]: FINISHED')


if __name__ == '__main__':
    main()
