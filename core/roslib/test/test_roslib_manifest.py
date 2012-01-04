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
import sys
import unittest

import roslib

def get_test_path():
    return os.path.abspath(os.path.dirname(__file__))

class RoslibManifestTest(unittest.TestCase):
  
  def test_ManifestException(self):
    from roslib.manifest import ManifestException
    self.assert_(isinstance(ManifestException(), Exception))

  def test_Depend(self):
    from roslib.manifestlib import Depend, ManifestException
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
    self.assertNotEquals(d, Depend('roslib2'))
    self.assertNotEquals(d, 1)

  def test_ROSDep(self):
    from roslib.manifest import ROSDep, ManifestException
    for bad in [None, '']:
      try:
        rd = ROSDep(bad)
        self.fail("should have failed on [%s]"%bad)
      except ValueError: pass
    
    rd = ROSDep('python')
    self.assertEquals('<rosdep name="python" />',rd.xml())
    
  def test_VersionControl(self):
    from roslib.manifest import VersionControl, ManifestException
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
    from roslib.manifest import Manifest
    self.assert_(isinstance(m, Manifest))
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
    
  def test_parse_example1_file(self):
    from roslib.manifest import parse_file, Manifest
    p = os.path.join(get_test_path(), 'manifest_tests', 'example1.xml')
    self._subtest_parse_example1(parse_file(p))

  def test_parse_example1_string(self):
    from roslib.manifest import parse, Manifest
    self._subtest_parse_example1(parse(EXAMPLE1))

  def test_Manifest_str(self):
    # just make sure it doesn't crash
    from roslib.manifest import parse
    str(parse(EXAMPLE1))
    
  def test_Manifest_xml(self):
    from roslib.manifest import parse
    m = parse(EXAMPLE1)
    self._subtest_parse_example1(m)
    # verify roundtrip
    m2 = parse(m.xml())
    self._subtest_parse_example1(m2)
    
    
  def test_parse_bad_file(self):
    from roslib.manifest import parse_file, Manifest
    # have to import from ManifestException due to weirdness when run in --cov mode
    from roslib.manifestlib import ManifestException
    base_p = os.path.join(get_test_path(), 'manifest_tests')
    for b in ['bad1.xml', 'bad2.xml', 'bad3.xml']:
      p = os.path.join(base_p, b)
      try:
        parse_file(p)
        self.fail("parse should have failed on bad manifest")
      except ManifestException, e:
        print str(e)
        self.assert_(b in str(e), "file name should be in error message: %s"%(str(e)))
    
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
  <rosbuild2> 
    <depend thirdparty="thisshouldbeokay"/> 
  </rosbuild2>
</package>"""
