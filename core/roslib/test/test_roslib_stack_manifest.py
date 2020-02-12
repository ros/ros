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

import os
import unittest


def get_test_path():
    return os.path.abspath(os.path.dirname(__file__))


class RoslibStackManifestTest(unittest.TestCase):

    def _subtest_parse_stack_example1(self, m):
        from roslib.manifestlib import _Manifest
        self.assert_(isinstance(m, _Manifest))
        self.assertEquals('stack', m._type)
        self.assertEquals('a brief description', m.brief)
        self.assertEquals('Line 1\nLine 2', m.description.strip())
        self.assertEquals('The authors\ngo here', m.author.strip())
        self.assertEquals('Public Domain\nwith other stuff', m.license.strip())
        self.assertEquals('http://ros.org/stack/', m.url)
        self.assertEquals('http://www.willowgarage.com/files/willowgarage/robot10.jpg', m.logo)
        dpkgs = [d.stack for d in m.depends]
        self.assertEquals({'stackname', 'common'}, set(dpkgs))
        self.assertEquals([], m.rosdeps)
        self.assertEquals([], m.exports)

    def _subtest_parse_stack_version(self, m):
        self.assertEquals('1.2.3', m.version)

    def test_parse_example1_file(self):
        from roslib.stack_manifest import parse_file

        p = os.path.join(get_test_path(), 'manifest_tests', 'stack_example1.xml')
        self._subtest_parse_stack_example1(parse_file(p))

        p = os.path.join(get_test_path(), 'manifest_tests', 'stack_version.xml')
        self._subtest_parse_stack_version(parse_file(p))

    def test_parse_example1_string(self):
        from roslib.manifestlib import parse, _Manifest
        self._subtest_parse_stack_example1(parse(_Manifest('stack'), STACK_EXAMPLE1))

    def test_StackManifest(self):
        from roslib.stack_manifest import StackManifest
        m = StackManifest()
        self.assertEquals('stack', m._type)

    def test_StackManifest_str(self):
        # just make sure it doesn't crash
        from roslib.stack_manifest import parse
        str(parse(STACK_EXAMPLE1))

    def test_StackManifest_xml(self):
        from roslib.stack_manifest import parse
        m = parse(STACK_EXAMPLE1)
        self._subtest_parse_stack_example1(m)
        # verify roundtrip
        m2 = parse(m.xml())
        self._subtest_parse_stack_example1(m2)


# bad file examples should be more like the roslaunch tests where there is just 1 thing wrong
STACK_EXAMPLE1 = """<stack>
  <description brief="a brief description">Line 1
Line 2
  </description>
  <author>The authors
go here</author>
  <license>Public Domain
with other stuff</license>
  <url>http://ros.org/stack/</url>
  <logo>http://www.willowgarage.com/files/willowgarage/robot10.jpg</logo>
  <depend stack="stackname" />
  <depend stack="common"/>
</stack>"""

STACK_INVALID1 = """<stack>
  <description brief="a brief description">Line 1</description>
  <author>The authors</author>
  <license>Public Domain</license>
  <rosdep name="python" />
</stack>"""

STACK_INVALID2 = """<stack>
  <description brief="a brief description">Line 1</description>
  <author>The authors</author>
  <license>Public Domain</license>
  <export>
    <cpp cflags="-I${prefix}/include" lflags="-L${prefix}/lib -lros"/>
    <cpp os="osx" cflags="-I${prefix}/include" lflags="-L${prefix}/lib -lrosthread -framework CoreServices"/>
  </export>
</stack>"""
