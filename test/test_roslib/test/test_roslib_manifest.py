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
import roslib; roslib.load_manifest('test_roslib')

import os
import struct
import sys
import unittest

import rostest

class RoslibManifestTest(unittest.TestCase):
  
  def test_ManifestException(self):
    from roslib.manifest import ManifestException
    self.assert_(isinstance(ManifestException(), Exception))

  def test_Depend(self):
    from roslib.manifest import Depend, ManifestException
    for bad in [None, '', 1]:
      try:
        Depend(bad)
        self.fail("should have failed on [%s]"%bad)
      except ManifestException: pass
    
    d = Depend('roslib')
    self.assertEquals('roslib', str(d))
    self.assertEquals('roslib', repr(d))    

    self.assertEquals('<depend package="roslib" />',d.xml())
    self.assertEquals(d, Depend('roslib'))
    self.assertNotEquals(d, Depend('roslib2'))
    self.assertNotEquals(d, 1)

  def test_ROSDep(self):
    from roslib.manifest import ROSDep, ManifestException
    for bad in [None, '', 1]:
      try:
        rd = ROSDep(bad)
        self.fail("should have failed on [%s]"%bad)
      except ManifestException: pass
    
    rd = ROSDep('python')
    self.assertEquals('<rosdep name="python" />',rd.xml())
    
  def test_SysDepend(self):
    # deprecated tag
    from roslib.manifest import SysDepend, ManifestException

    bad = [
      (None, 'ubuntu', '8.04-hardy'),
      ]
    for package, os_, version in bad:
      try:
        SysDepend(package, os_, version)
        self.fail("should have failed")
      except ManifestException: pass
      
    # package, os, version
    # os and version are optional
    tests = [
      ('pack', 'ubuntu', '8.04-hardy', '<sysdepend package="pack" os="ubuntu" version="8.04-hardy" />'),
      ('pack', 'ubuntu', None, '<sysdepend package="pack" os="ubuntu" />'),
      ('pack', None, '8.04-hardy', '<sysdepend package="pack" version="8.04-hardy" />'),
      ('pack', None, None, '<sysdepend package="pack" />'),
      ]
    for package, os_, version, xml in tests:
      sd = SysDepend(package, os_, version)
      self.assertEquals(package, sd.package)
      self.assertEquals(os_, sd.os)
      self.assertEquals(version, sd.version)
      self.assertEquals(xml, sd.xml())

      self.assertEquals(sd, SysDepend(package, os_, version))
      self.assertNotEquals(sd, SysDepend(package+'foo', os_, version))
      if os_:
        self.assertNotEquals(sd, SysDepend(package, None, version))
        self.assertNotEquals(sd, SysDepend(package, os_+'foo', version))        
      else:
        self.assertNotEquals(sd, SysDepend(package, 'foo', version))        
      if version:
        self.assertNotEquals(sd, SysDepend(package, os_, version+'foo'))
        self.assertNotEquals(sd, SysDepend(package, os_, None))        
      else:
        self.assertNotEquals(sd, SysDepend(package, os_, 'foo'))
      

  def test_VersionControl(self):
    from roslib.manifest import VersionControl, ManifestException
    ros_svn = 'https://ros.svn.sf.net/svnroot'
    
    bad = [
      (None, ros_svn),
      ('svn', 1),
      ]
    for type_, url in bad:
      try:
        VersionControl(type_,url)
        self.fail("should have failed on [%s] [%s]"%(type_, url))
      except ManifestException: pass
      
    tests = [
      ('svn', ros_svn, '<versioncontrol type="svn" url="%s" />'%ros_svn),
      ('cvs', None, '<versioncontrol type="cvs" />'),
      ]
    for type_, url, xml in tests:
      vc = VersionControl(type_, url)
      self.assertEquals(type_, vc.type)
      self.assertEquals(url, vc.url)      
      self.assertEquals(xml, vc.xml())

  def test_parse_example1(self):
    from roslib.manifest import parse, Manifest
    m = parse(EXAMPLE1)
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
    
    from roslib.manifest import SysDepend
    self.assert_(SysDepend('antigravity', 'ubuntu', '8.04-hardy') in m.sysdepends)
    self.assert_(SysDepend('antigravity', 'centos', '5.2') in m.sysdepends)
    self.assert_(SysDepend('mac-antigravity', 'macports', None) in m.sysdepends)
                
    
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
  <sysdepend version="8.04-hardy" package="antigravity" os="ubuntu" />
  <sysdepend package="mac-antigravity" os="macports" />
  <sysdepend version="5.2" os="centos" package="antigravity" />
  <rosdep name="python" />
  <rosdep name="bar" />
  <rosdep name="baz" />
</package>"""
      
    
    
if __name__ == '__main__':
  rostest.unitrun('test_roslib', 'test_manifest', RoslibManifestTest, coverage_packages=['roslib.manifest'])

