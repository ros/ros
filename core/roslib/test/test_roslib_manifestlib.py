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
import struct
import sys
import unittest

import roslib

class RoslibManifestlibTest(unittest.TestCase):
  
  def test_ManifestException(self):
    from roslib.manifestlib import ManifestException
    self.assert_(isinstance(ManifestException(), Exception))

  def test_Platform(self):
    from roslib.manifestlib import Platform, ManifestException
    for bad in [None, '']:
      try:
        Platform(bad, '1')
        self.fail("should have failed on [%s]"%bad)
      except ValueError: pass
      try:
        Platform('ubuntu', bad)
        self.fail("should have failed on [%s]"%bad)
      except ValueError: pass
    
    p = Platform('ubuntu', '8.04')
    self.assertEquals('ubuntu 8.04', str(p))
    self.assertEquals('ubuntu 8.04', repr(p))

    self.assertEquals('<platform os="ubuntu" version="8.04"/>',p.xml())
    self.assertEquals(p, Platform('ubuntu', '8.04'))
    self.assertEquals(p, Platform('ubuntu', '8.04', notes=None))
    self.assertNotEquals(p, Platform('ubuntu', '8.04', 'some notes'))
    self.assertNotEquals(p, 'foo')
    self.assertNotEquals(p, 1)

    # note: probably actually "osx"
    p = Platform('OS X', '10.6', 'macports')
    self.assertEquals('OS X 10.6', str(p))
    self.assertEquals('OS X 10.6', repr(p))

    self.assertEquals('<platform os="OS X" version="10.6" notes="macports"/>',p.xml())
    self.assertEquals(p, p)
    self.assertEquals(p, Platform('OS X', '10.6', 'macports'))
    self.assertNotEquals(p, Platform('OS X', '10.6'))
    self.assertNotEquals(p, 'foo')
    self.assertNotEquals(p, 1)
    

  def test_Depend(self):
    from roslib.manifestlib import Depend, StackDepend, ManifestException
    for bad in [None, '']:
      try:
        Depend(bad)
        self.fail("should have failed on [%s]"%bad)
      except ValueError: pass
    
    d = Depend('roslib')
    self.assertEquals('roslib', str(d))
    self.assertEquals('roslib', repr(d))    

    self.assertEquals('<depend package="roslib" />',d.xml())
    self.assertEquals(d, Depend('roslib'))
    self.assertNotEquals(d, StackDepend('roslib'))
    self.assertNotEquals(d, Depend('roslib2'))
    self.assertNotEquals(d, 1)
    
  def test_StackDepend(self):
    from roslib.manifestlib import Depend, StackDepend, ManifestException
    for bad in [None, '']:
      try:
        StackDepend(bad)
        self.fail("should have failed on [%s]"%bad)
      except ValueError: pass
    
    d = StackDepend('common')
    self.assertEquals('common', str(d))
    self.assertEquals('common', repr(d))    

    self.assertEquals('<depend stack="common" />',d.xml())
    self.assertEquals(d, StackDepend('common'))
    self.assertNotEquals(d, Depend('common'))
    self.assertNotEquals(d, StackDepend('common2'))
    self.assertNotEquals(d, 1)

  def test_ROSDep(self):
    from roslib.manifestlib import ROSDep, ManifestException
    for bad in [None, '']:
      try:
        rd = ROSDep(bad)
        self.fail("should have failed on [%s]"%bad)
      except ValueError: pass
    
    rd = ROSDep('python')
    self.assertEquals('<rosdep name="python" />',rd.xml())
    
  def test_VersionControl(self):
    from roslib.manifestlib import VersionControl, ManifestException
    ros_svn = 'https://ros.svn.sf.net/svnroot'
    
    bad = [
      (None, ros_svn),
      ]
    for type_, url in bad:
      try:
        VersionControl(type_,url)
        self.fail("should have failed on [%s] [%s]"%(type_, url))
      except ValueError: pass
      
    tests = [
      ('svn', ros_svn, '<versioncontrol type="svn" url="%s" />'%ros_svn),
      ('cvs', None, '<versioncontrol type="cvs" />'),
      ]
    for type_, url, xml in tests:
      vc = VersionControl(type_, url)
      self.assertEquals(type_, vc.type)
      self.assertEquals(url, vc.url)      
      self.assertEquals(xml, vc.xml())

  def _subtest_parse_example1(self, m):
    from roslib.manifestlib import _Manifest
    self.assert_(isinstance(m, _Manifest))
    self.assertEquals("a brief description", m.brief)
    self.assertEquals("Line 1\nLine 2", m.description.strip())
    self.assertEquals("The authors\ngo here", m.author.strip())    
    self.assertEquals("Public Domain\nwith other stuff", m.license.strip())
    self.assertEquals("http://pr.willowgarage.com/package/", m.url)
    self.assertEquals("http://www.willowgarage.com/files/willowgarage/robot10.jpg", m.logo)
    dpkgs = [d.package for d in m.depends]
    self.assertEquals(set(['pkgname', 'common']), set(dpkgs))
    rdpkgs = [d.name for d in m.rosdeps]
    self.assertEquals(set(['python', 'bar', 'baz']), set(rdpkgs))
    for p in m.platforms:
      if p.os == 'ubuntu':
        self.assertEquals("8.04", p.version)
        self.assertEquals('', p.notes)        
      elif p.os == 'OS X':
        self.assertEquals("10.6", p.version)
        self.assertEquals("macports", p.notes)        
      else:
        self.fail("unknown platform "+str(p))

  def _subtest_parse_stack_example1(self, m):
    from roslib.manifestlib import _Manifest
    self.assert_(isinstance(m, _Manifest))
    self.assertEquals('stack', m._type)
    self.assertEquals("a brief description", m.brief)
    self.assertEquals("Line 1\nLine 2", m.description.strip())
    self.assertEquals("The authors\ngo here", m.author.strip())    
    self.assertEquals("Public Domain\nwith other stuff", m.license.strip())
    self.assertEquals("http://ros.org/stack/", m.url)
    self.assertEquals("http://www.willowgarage.com/files/willowgarage/robot10.jpg", m.logo)
    dpkgs = [d.stack for d in m.depends]
    self.assertEquals(set(['stackname', 'common']), set(dpkgs))
    self.assertEquals([], m.rosdeps)
    self.assertEquals([], m.exports)    

  def _subtest_parse_stack_version(self, m):
    self.assertEquals("1.2.3", m.version)

  def test_parse_example1_file(self):
    from roslib.manifestlib import parse_file, _Manifest
    p = os.path.join(get_test_path(), 'manifest_tests', 'example1.xml')
    self._subtest_parse_example1(parse_file(_Manifest(), p))
    
    p = os.path.join(get_test_path(), 'manifest_tests', 'stack_example1.xml')
    self._subtest_parse_stack_example1(parse_file(_Manifest('stack'), p))

    p = os.path.join(get_test_path(), 'manifest_tests', 'stack_version.xml')
    self._subtest_parse_stack_version(parse_file(_Manifest('stack'), p))

  def test_parse_example1_string(self):
    from roslib.manifestlib import parse, _Manifest
    self._subtest_parse_example1(parse(_Manifest(), EXAMPLE1))
    self._subtest_parse_stack_example1(parse(_Manifest('stack'), STACK_EXAMPLE1))
    
  def test__Manifest(self):
    from roslib.manifestlib import _Manifest
    m = _Manifest()
    # check defaults
    self.assertEquals('package', m._type)
    m = _Manifest('stack')
    self.assertEquals('stack', m._type)    
    
  def test_Manifest_str(self):
    # just make sure it doesn't crash
    from roslib.manifestlib import parse, _Manifest
    str(parse(_Manifest(), EXAMPLE1))
    
  def test_Manifest_xml(self):
    from roslib.manifestlib import parse, _Manifest
    m = _Manifest()
    parse(m, EXAMPLE1)
    self._subtest_parse_example1(m)
    # verify roundtrip
    m2 = _Manifest()
    parse(m2, m.xml())
    self._subtest_parse_example1(m2)
    
  # bad file examples should be more like the roslaunch tests where there is just 1 thing wrong
  def test_parse_bad_file(self):
    from roslib.manifestlib import parse_file, _Manifest, ManifestException
    base_p = os.path.join(get_test_path(), 'manifest_tests')
    m = _Manifest()
    for b in ['bad1.xml', 'bad2.xml', 'bad3.xml']:
      p = os.path.join(base_p, b)
      try:
        parse_file(m, p)
        self.fail("parse should have failed on bad manifest")
      except ManifestException, e:
        print str(e)
        self.assert_(b in str(e), "file name should be in error message [%s]"%(str(e)))
    
EXAMPLE1 = """<package>
  <description brief="a brief description">Line 1
Line 2
  </description>
  <author>The authors
go here</author>
  <license>Public Domain
with other stuff</license>
  <url>http://pr.willowgarage.com/package/</url>
  <logo>http://www.willowgarage.com/files/willowgarage/robot10.jpg</logo>
  <depend package="pkgname" />
  <depend package="common"/>
  <export>
    <cpp cflags="-I${prefix}/include" lflags="-L${prefix}/lib -lros"/>
    <cpp os="osx" cflags="-I${prefix}/include" lflags="-L${prefix}/lib -lrosthread -framework CoreServices"/>
  </export>
  <rosdep name="python" />
  <rosdep name="bar" />
  <rosdep name="baz" />
  <platform os="ubuntu" version="8.04" />
  <platform os="OS X" version="10.6" notes="macports" />
  <rosbuild2> 
    <depend thirdparty="thisshouldbeokay"/> 
  </rosbuild2>
</package>"""

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


def get_test_path():
    return os.path.abspath(os.path.dirname(__file__))
