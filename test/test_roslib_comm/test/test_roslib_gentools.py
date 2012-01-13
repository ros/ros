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

import os
import string 
import sys 
import unittest
import cStringIO
        
import roslib.names
import roslib.packages

TEST_CTX = 'rosgraph_msgs'

import roslib
PKG='test_roslib_comm'

class TestGentools(unittest.TestCase):

    def setUp(self):
        pass
        
    def _load_md5_tests(self, dir):
        test_dir = os.path.join(roslib.packages.get_pkg_dir(PKG), 'test', 'md5tests', dir)
        tests = {}
        for f in os.listdir(test_dir):
            path = os.path.join(test_dir, f)
            if not f.endswith('.txt'):
                continue
            name = f[:-4]
            while name and name[-1].isdigit():
                name = name[:-1]
            self.assert_(name)
            if name in tests:
                tests[name].append(path)
            else:
                tests[name] = [path]
        return tests
    
    def _compute_md5(self, f):
        from roslib.gentools import compute_md5, get_dependencies
        from roslib.msgs import load_from_string

        text = open(f, 'r').read()
        spec = load_from_string(text, package_context=TEST_CTX) 
        get_deps_dict = get_dependencies(spec, TEST_CTX, compute_files=False)
        return compute_md5(get_deps_dict)
        
    def _compute_md5_text(self, f):
        from roslib.gentools import compute_md5_text, get_dependencies
        from roslib.msgs import load_from_string

        text = open(f, 'r').read()
        spec = load_from_string(text, package_context=TEST_CTX)
        get_deps_dict = get_dependencies(spec, TEST_CTX, compute_files=False)
        return compute_md5_text(get_deps_dict, spec)

    def test_compute_md5_text(self):
        from std_msgs.msg import Header
        Header_md5 = Header._md5sum
        rg_msg_dir = os.path.join(roslib.packages.get_pkg_dir(TEST_CTX), 'msg')
        clock_msg = os.path.join(rg_msg_dir, 'Clock.msg')
        # a bit gory, but go ahead and regression test these important messages
        self.assertEquals("time clock", self._compute_md5_text(clock_msg))
        log_msg = os.path.join(rg_msg_dir, 'Log.msg')
        self.assertEquals("byte DEBUG=1\nbyte INFO=2\nbyte WARN=4\nbyte ERROR=8\nbyte FATAL=16\n%s header\nbyte level\nstring name\nstring msg\nstring file\nstring function\nuint32 line\nstring[] topics"%Header_md5, self._compute_md5_text(log_msg))

        tests = self._load_md5_tests('md5text')
        # text file #1 is the reference
        for k, files in tests.iteritems():
            print "running tests", k
            ref_file = [f for f in files if f.endswith('%s1.txt'%k)]
            if not ref_file:
                self.fail("failed to load %s"%k)
            ref_file = ref_file[0]
            ref_text = open(ref_file, 'r').read().strip()
            print "KEY", k
            files = [f for f in files if not f.endswith('%s1.txt'%k)]
            for f in files[1:]:
                f_text = self._compute_md5_text(f)
                self.assertEquals(ref_text, f_text, "failed on %s\n%s\n%s: \n[%s]\nvs.\n[%s]\n"%(k, ref_file, f, ref_text, f_text))
        
    def test_md5_equals(self):
        tests = self._load_md5_tests('same')
        for k, files in tests.iteritems():
            print "running tests", k
            md5sum = self._compute_md5(files[0])
            for f in files[1:]:
                self.assertEquals(md5sum, self._compute_md5(f), "failed on %s: \n[%s]\nvs.\n[%s]\n"%(k, self._compute_md5_text(files[0]), self._compute_md5_text(f)))
    
    def test_md5_not_equals(self):
        tests = self._load_md5_tests('different')
        for k, files in tests.iteritems():
            print "running tests", k
            md5s = set()
            md6md5sum = self._compute_md5(files[0])
            for f in files:
                md5s.add(self._compute_md5(f))
            # each md5 should be unique
            self.assertEquals(len(md5s), len(files))
