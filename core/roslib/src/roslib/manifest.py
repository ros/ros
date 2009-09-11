#! /usr/bin/env python
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
# Revision $Id: manifest.py 3357 2009-01-13 07:13:05Z jfaustwg $
# $Author: jfaustwg $
"""Python parser for rospack manifest.xml files"""
## Python parser for rospack manifest.xml files
## See: http://pr.willowgarage.com/wiki/ROS/Package_Structure

import sys
import os
import getopt
import itertools
import re
import xml.dom
import xml.dom.minidom as dom
from xml.dom.minidom import parse, parseString

import roslib.exceptions
import roslib.packages
import roslib.rosenv

MANIFEST_FILE = 'manifest.xml'

EXAMPLE = """<package>
  <description brief="a brief description">A much longer description goes here.<br />
    Here is some more info.<br />
    <strong>XHTML is legal</strong>
  </description>
  <author>Ken Conley/kwc@willowgarage.com, Eric Berger/berger@willowgarage.com, Morgan Quigley/mquigley@cs.stanford.edu</author>
  <license>Public Domain</license>
  <url>http://pr.willowgarage.com/package/</url>
  <logo>http://www.willowgarage.com/files/willowgarage/robot10.jpg</logo>
  <depend package="pkgname" />
  <depend package="common"/>
  <export>
    <cpp cflags="-I${prefix}/include" lflags="-L${prefix}/lib -lros"/>
    <cpp os="osx" cflags="-I${prefix}/include" lflags="-L${prefix}/lib -lrosthre
ad -framework CoreServices"/>
  </export>
  <rosdep name="python" />
</package>"""

REQUIRED = ['author', 'license']
ALLOWXHTML = ['description']
OPTIONAL = ['logo', 'url', 'brief', 'description', 'status', 'notes']

class ManifestException(roslib.exceptions.ROSLibException): pass

# TODO: this is all needlessly complicated in and indirect. Now that
# we are more commited to our manifest spec, this can be more direct
# (and unit tested)

def _check_optional(name, allowXHTML=False):
    def check(n, filename):
        n = n.getElementsByTagName(name)    
        if len(n) > 1:
            raise Exception("Invalid manifest file: must have a single '%s' element"%n)
        if n:
            if allowXHTML:
                return ''.join([x.toxml() for x in n[0].childNodes])
            return _get_text(n[0].childNodes).strip()
    return check

def _check_required(name, allowXHTML=False):
    def check(n, filename):
        n = n.getElementsByTagName(name)
        if not n:
            print >> sys.stderr, "Invalid manifest file[%s]: missing required '%s' element"%(filename, name)
            return ''
        if len(n) != 1:
            raise Exception("Invalid manifest file: must have only one '%s' element"%name)
        if allowXHTML:
            return ''.join([x.toxml() for x in n[0].childNodes])
        return _get_text(n[0].childNodes).strip()
    return check

def _check_depends(name):
    def check(n, filename):
        depends = [e.attributes for e in n.getElementsByTagName(name)]
        packages = [d['package'].value for d in depends]
        return [Depend(p) for p in packages]
    return check

def _check_rosdeps(name):
    def check(n, filename):
        rosdeps = [e.attributes for e in n.getElementsByTagName(name)]
        names = [d['name'].value for d in rosdeps]
        return [ROSDep(n) for n in names]
    return check

def _check_sysdepends(name):
    def check(n, filename):
        ret_val = []
        for e in n.getElementsByTagName(name):
            ret_val.append(SysDepend(e.attributes['package'].value, e.getAttribute('os'), e.getAttribute('version')))
        return ret_val
    return check

def _attrs(node):
    attrs = {}
    for k in node.attributes.keys(): 
        attrs[k] = node.attributes.get(k).value
    return attrs
    
def _check_exports(name):
    def check(n, filename):
        ret_val = []
        for e in n.getElementsByTagName(name):
            elements = [c for c in e.childNodes if c.nodeType == c.ELEMENT_NODE]
            ret_val.extend([Export(t.tagName, _attrs(t), _get_text(t.childNodes)) for t in elements])
        return ret_val 
    return check

def _check_versioncontrol(name):
    def check(n, filename):
        e = n.getElementsByTagName(name)
        if not e:
            return None
        # note: 'url' isn't actually required, but as we only support type=svn it implicitly is for now
        return VersionControl(e[0].attributes['type'].value, e[0].attributes['url'].value)
    return check

def _check(name):
    if name == 'depend':
        return _check_depends('depend')
    elif name == 'sysdepend':
        return _check_sysdepends('sysdepend')
    elif name == 'export':
        return _check_exports('export')
    elif name == 'versioncontrol':
        return _check_versioncontrol('versioncontrol')
    elif name == 'rosdep':
        return _check_rosdeps('rosdep')
    elif name in REQUIRED:
        if name in ALLOWXHTML:
            return _check_required(name, True)
        return _check_required(name)            
    elif name in OPTIONAL:
        if name in ALLOWXHTML:
            return _check_optional(name, True)
        return _check_optional(name)
    
## Manifest 'export' tag
class Export(object):
    ## @param self
    ## @param tag str: name of the XML tag
    ## @param attrs dict: dictionary of XML attributes for this export tag
    ## @param str str: string value contained by tag, if any
    def __init__(self, tag, attrs, str):
        self.tag = tag
        self.attrs = attrs
        self.str = str
    ## @return str: value of attribute or None if attribute not set
    def get(self, attr):
        return self.attrs.get(attr, None)
    def xml(self):
        attrs = ' '.join([' %s="%s"'%(k,v) for k,v in self.attrs.iteritems()])
        if self.str:
            return '<%s%s>%s</%s>'%(self.tag, attrs, self.str, self.tag)
        else:
            return '<%s%s />'%(self.tag, attrs)
        
## Manifest 'depend' tag
class Depend(object):
    __slots__ = ['package']
    def __init__(self, package):
        if not package or not isinstance(package, basestring):
            raise ManifestException("bad 'package' attribute")
        self.package = package
    def __str__(self):
        return self.package
    def __repr__(self):
        return self.package
    def __eq__(self, obj):
        if not isinstance(obj, Depend):
            return False
        return self.package == obj.package 
    def xml(self):
        return '<depend package="%s" />'%self.package
        
## object representation of a ROS manifest 'sysdepend' tag.
## os and version are optional attributes, though recommended where possible.
class SysDepend(object):
    __slots__ = ['package','os', 'version']

    ## ctor.
    ## @param self
    ## @param package str: package name
    ## @param os str: (optional) os name. Empty string is equivalent to None
    ## @param version str: (optional) version string. Empty string is equivalent to None
    def __init__(self, package, os, version):
        if not package or not isinstance(package, basestring):
            raise ManifestException("bad 'package' attribute: %s"%package)
        self.package = package
        self.os = os or None
        self.version = version or None
        
    def __eq__(self, obj):
        if not isinstance(obj, SysDepend):
            return False
        return self.package == obj.package and self.os == obj.os and self.version == obj.version
    
    ## @return str: XML representation of <sysdepend/> tag
    def xml(self):
        os = version = ""
        if self.os:
            os = ' os="%s"'%self.os
        if self.version:
            version = ' version="%s"'%self.version
        return '<sysdepend package="%s"%s%s />'%(self.package, os, version) 

## object representation of a ROS manifest 'rosdep' tag
class ROSDep(object):
    __slots__ = ['name',]

    ## ctor.
    ## @param self
    ## @param name str: dependency name
    def __init__(self, name):
        if not name or not isinstance(name, basestring):
            raise ManifestException("bad 'name' attribute")
        self.name = name
    ## get XML representation of rosdep
    ## @return str: XML representation of <rosdep/> tag
    def xml(self):
        return '<rosdep name="%s" />'%self.name

## object representation of a ROS manifest 'versioncontrol' tag
class VersionControl(object):
    __slots__ = ['type', 'url']
    def __init__(self, type_, url):
        if not type_ or not isinstance(type_, basestring):
            raise ManifestException("bad 'type' attribute")
        if not url is None and not isinstance(url, basestring):
            raise ManifestException("bad 'url' attribute")
        self.type = type_
        self.url = url
    def xml(self):
        if self.url:
            return '<versioncontrol type="%s" url="%s" />'%(self.type, self.url)
        else:
            return '<versioncontrol type="%s" />'%self.type
    
## object representation of a ROS manifest file
class Manifest(object):
    __slots__ = ['description', 'brief', \
                 'author', 'license', 'license_url', 'url', \
                 'depends', 'sysdepends', 'rosdeps',\
                 'logo', 'exports',\
                 'versioncontrol', 'status', 'notes']
    def __init__(self):
        self.description = self.brief = self.author = \
                           self.license = self.license_url = \
                           self.url = self.logo = self.status = self.notes = ''
        self.depends = []
        self.rosdeps = []
        self.sysdepends = []
        self.exports = []
        
    def __str__(self):
        return self.xml()
    ## Get exports that match the specified tag and attribute, e.g. 'python', 'path'
    def get_export(self, tag, attr):
        return [e.get(attr) for e in self.exports if e.tag == tag if e.get(attr) is not None]
    ## Render the Manifest as XML
    def xml(self):
        if not self.brief:
            desc = "  <description>%s</description>"%self.description
        else:
            desc = '  <description brief="%s">%s</description>'%(self.brief, self.description) 
        author  = "  <author>%s</author>"%self.author
        if self.license_url:
            license = '  <license url="%s">%s</license>'%(self.license_url, self.license)
        else:
            license = "  <license>%s</license>"%self.license
        versioncontrol = url = logo = exports = ""
        if self.url:
            url     = "  <url>%s</url>"%self.url
        if self.logo:
            logo    = "  <logo>%s</logo>"%self.logo
        depends = '\n'.join(["  %s"%d.xml() for d in self.depends])
        sysdepends = '\n'.join(["  %s"%sd.xml() for sd in self.sysdepends])
        rosdeps = '\n'.join(["  %s"%rd.xml() for rd in self.rosdeps])
        if self.exports:
            exports = '  <export>\n' + '\n'.join(["  %s"%e.xml() for e in self.exports]) + '  </export>'
        if self.versioncontrol:
            versioncontrol = "  %s"%self.versioncontrol.xml()
        if self.status or self.notes:
            review = '  <review status="%s" notes="%s" />'%(self.status, self.notes)
        fields = filter(lambda x: x, [desc, author, license, review, url, logo, depends, sysdepends, exports, versioncontrol])
        return "<package>\n" + "\n".join(fields) + "\n</package>"


## @param package_dir str: path to package directory
## @param environ dict: environment dictionary
## @param required bool: require that the directory exist
## @return str: path to manifest file of package
## @throws InvalidROSPkgException if required is True and manifest file cannot be located
def _manifest_file_by_dir(package_dir, required=True, environ=os.environ):
    try:
        p = os.path.join(package_dir, MANIFEST_FILE)
        if not required and not os.path.exists(p):
            return p
        if not os.path.isfile(p):
            raise roslib.packages.InvalidROSPkgException("""
Package '%(package_dir)s' is improperly configured: no manifest file is present.
"""%locals())
        return p
    except roslib.packages.InvalidROSPkgException, e:
        if required:
            raise

## @param package str: package name
## @param environ dict: environment dictionary
## @param required bool: require that the directory exist
## @return str: path to manifest file of package
## @throws InvalidROSPkgException if required is True and manifest file cannot be located
def manifest_file(package, required=True, environ=os.environ):
    # ros_root needs to be determined from the environment or else
    # everything breaks when trying to launch nodes via ssh where the
    # path isn't setup correctly.
    pkg_dir = roslib.packages.get_pkg_dir(package, required, ros_root=environ[roslib.rosenv.ROS_ROOT]) 
    return _manifest_file_by_dir(pkg_dir, required, environ)
        
## @internal
def _get_all_dependencies(m, depends, env=os.environ):
    newdepends = []
    for d in m.depends:
        if not d in depends and not d in newdepends:
            newdepends.append(d)
    depends.extend(newdepends)
    for d in newdepends:
        dep_manifest_file = roslib.resources.manifest_file(d.package, True, env)
        _get_all_dependencies(parse_file(dep_manifest_file), depends, env)
    return depends
    
## @internal
def get_all_dependencies(m, env=os.environ):
    return _get_all_dependencies(m, [], env)

## @internal
## DOM utility routine for getting contents of text nodes
def _get_text(nodes):
    return "".join([n.data for n in nodes if n.nodeType == n.TEXT_NODE])

## Parse manifest.xml file
## @param file str: manifest.xml file path
## @return Manifest
def parse_file(file):
    if not file:
        raise Exception("Missing manifest file argument")
    if not os.path.isfile(file):
        raise Exception("Invalid/non-existent manifest file: %s"%file)
    f = open(file, 'r')
    try:
        text = f.read()
    finally:
        f.close()
    return parse(text, file)

## Parse manifest.xml string contents
## @param string str: manifest.xml contents
## @return Manifest
def parse(string, filename='string'):
    try:
        d = dom.parseString(string)
    except Exception, e:
        raise Exception("Invalid manifest file [%s]: %s"%(filename, e))
    p = d.getElementsByTagName("package")
    if len(p) != 1:
        raise Exception("Invalid manifest file [%s]: must have a single 'package' element"%filename)
    p = p[0]
    m = Manifest()
    m.description = _check('description')(p, filename)
    m.brief = ''
    try:
        tag = p.getElementsByTagName('description')[0]
        m.brief = tag.getAttribute('brief') or ''
    except:
        pass #manifest is missing required 'description' tag
    m.depends = _check('depend')(p, filename)
    m.sysdepends = _check('sysdepend')(p, filename)    
    m.rosdeps = _check('rosdep')(p, filename)    
    m.exports = _check('export')(p, filename)
    m.versioncontrol = _check('versioncontrol')(p,filename)
    m.license = _check('license')(p, filename)
    m.license_url = ''
    try:
        tag = p.getElementsByTagName('license')[0]
        m.license_url = tag.getAttribute('url') or ''
    except:
        pass #manifest is missing required 'license' tag
  
    m.status='unreviewed'
    try:
        tag = p.getElementsByTagName('review')[0]
        m.status=tag.getAttribute('status') or ''
    except:
        pass #manifest is missing optional 'review status' tag

    m.notes=''
    try:
        tag = p.getElementsByTagName('review')[0]
        m.notes=tag.getAttribute('notes') or ''
    except:
        pass #manifest is missing optional 'review notes' tag

    m.author = _check('author')(p, filename)
    m.url = _check('url')(p, filename)
    m.logo = _check('logo')(p, filename)
    return m

def _usage(stdout, progname):
    print >> stdout,  "%(progname)s manifest-file"%vars()

def _manifest_main(argv, stdout, environ):
    optlist, args = getopt.getopt(argv[1:], "h?", ["help", "test"])
    optlist = [opt[0] for opt in optlist]
    manifest = None
    #order is important due to len(args) check vs. '--test' option
    if "--test" in optlist:
        manifest = parse(EXAMPLE)
    elif filter(lambda x: x in ["-h","-?","--help"], optlist) or \
             len(args) != 1:
        _usage(stdout, argv[0])
        return
    else:
        f = open(args[0], 'r')
        try:
            text = f.read()
            manifest = parse(text)
        finally:
            f.close()
    print >> stdout, manifest
    
if __name__ == "__main__":
    _manifest_main(sys.argv, sys.stdout, os.environ)
