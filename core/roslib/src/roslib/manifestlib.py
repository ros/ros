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
# Revision $Id$
# $Author$

"""
Internal library for processing 'manifest' files, i.e. manifest.xml and stack.xml.
For external code apis, see L{roslib.manifest} and L{roslib.stack_manifest}.
"""

import sys
import os
import xml.dom
import xml.dom.minidom as dom

import roslib.exceptions

# stack.xml and manifest.xml have the same internal tags right now
REQUIRED = ['author', 'license']
ALLOWXHTML = ['description']
OPTIONAL = ['logo', 'url', 'brief', 'description', 'status', 'notes', 'depend', 'rosdep', 'export', 'review']
VALID = REQUIRED + OPTIONAL

class ManifestException(roslib.exceptions.ROSLibException): pass

# TODO: this is all needlessly complicated in and indirect. Now that
# we are more commited to our manifest spec, this can be more direct
# (and unit tested)

def check_optional(name, allowXHTML=False):
    """
    Validator for optional elements.
    @raise ManifestException: if validation fails
    """
    def check(n, filename):
        n = n.getElementsByTagName(name)    
        if len(n) > 1:
            raise ManifestException("Invalid manifest file: must have a single '%s' element"%name)
        if n:
            if allowXHTML:
                return ''.join([x.toxml() for x in n[0].childNodes])
            return _get_text(n[0].childNodes).strip()
    return check

def check_required(name, allowXHTML=False):
    """
    Validator for required elements.
    @raise ManifestException: if validation fails
    """
    def check(n, filename):
        n = n.getElementsByTagName(name)
        if not n:
            print >> sys.stderr, "Invalid manifest file[%s]: missing required '%s' element"%(filename, name)
            return ''
        if len(n) != 1:
            raise ManifestException("Invalid manifest file: must have only one '%s' element"%name)
        if allowXHTML:
            return ''.join([x.toxml() for x in n[0].childNodes])
        return _get_text(n[0].childNodes).strip()
    return check

def check_depends(name):
    """
    Validator for manifest depends.
    @raise ManifestException: if validation fails
    """
    def check(n, filename):
        depends = [e.attributes for e in n.getElementsByTagName(name)]
        packages = [d['package'].value for d in depends]
        return [Depend(p) for p in packages]
    return check

def check_stack_depends(name):
    """
    Validator for stack depends.
    @raise ManifestException: if validation fails
    """
    def check(n, filename):
        depends = [e.attributes for e in n.getElementsByTagName(name)]
        packages = [d['stack'].value for d in depends]
        return [StackDepend(p) for p in packages]
    return check

def check_rosdeps(name):
    """
    Validator for stack rosdeps.    
    @raise ManifestException: if validation fails
    """
    def check(n, filename):
        rosdeps = [e.attributes for e in n.getElementsByTagName(name)]
        names = [d['name'].value for d in rosdeps]
        return [ROSDep(n) for n in names]
    return check

def _attrs(node):
    attrs = {}
    for k in node.attributes.keys(): 
        attrs[k] = node.attributes.get(k).value
    return attrs
    
def check_exports(name):
    def check(n, filename):
        ret_val = []
        for e in n.getElementsByTagName(name):
            elements = [c for c in e.childNodes if c.nodeType == c.ELEMENT_NODE]
            ret_val.extend([Export(t.tagName, _attrs(t), _get_text(t.childNodes)) for t in elements])
        return ret_val 
    return check

def check_versioncontrol(name):
    def check(n, filename):
        e = n.getElementsByTagName(name)
        if not e:
            return None
        # note: 'url' isn't actually required, but as we only support type=svn it implicitly is for now
        return VersionControl(e[0].attributes['type'].value, e[0].attributes['url'].value)
    return check

def check(name):
    if name == 'depend':
        return check_depends('depend')
    elif name == 'export':
        return check_exports('export')
    elif name == 'versioncontrol':
        return check_versioncontrol('versioncontrol')
    elif name == 'rosdep':
        return check_rosdeps('rosdep')
    elif name in REQUIRED:
        if name in ALLOWXHTML:
            return check_required(name, True)
        return check_required(name)            
    elif name in OPTIONAL:
        if name in ALLOWXHTML:
            return check_optional(name, True)
        return check_optional(name)
    
class Export(object):
    """
    Manifest 'export' tag
    """
    
    def __init__(self, tag, attrs, str):
        """
        Create new export instance.
        @param tag: name of the XML tag
        @type  tag: str
        @param attrs: dictionary of XML attributes for this export tag
        @type  attrs: dict
        @param str: string value contained by tag, if any
        @type  str: str
        """
        self.tag = tag
        self.attrs = attrs
        self.str = str

    def get(self, attr):
        """
        @return: value of attribute or None if attribute not set
        @rtype:  str
        """
        return self.attrs.get(attr, None)
    def xml(self):
        """
        @return: export instance represented as manifest XML
        @rtype: str
        """        
        attrs = ' '.join([' %s="%s"'%(k,v) for k,v in self.attrs.iteritems()])
        if self.str:
            return '<%s%s>%s</%s>'%(self.tag, attrs, self.str, self.tag)
        else:
            return '<%s%s />'%(self.tag, attrs)
        
class Depend(object):
    """
    Manifest 'depend' tag
    """
    __slots__ = ['package']

    def __init__(self, package):
        """
        Create new depend instance.
        @param package: package name. must be non-empty
        @type  package: str
        """
        if not package or not isinstance(package, basestring):
            raise ValueError("bad 'package' attribute")
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
        """
        @return: depend instance represented as manifest XML
        @rtype: str
        """
        return '<depend package="%s" />'%self.package
        
class StackDepend(object):
    """
    Stack Manifest 'depend' tag
    """
    __slots__ = ['stack']

    def __init__(self, stack):
        """
        @param stack: stack name. must be non-empty
        @type  stack: str
        """
        if not stack or not isinstance(stack, basestring):
            raise ValueError("bad 'stack' attribute")
        self.stack = stack
    def __str__(self):
        return self.stack
    def __repr__(self):
        return self.stack
    def __eq__(self, obj):
        if not isinstance(obj, StackDepend):
            return False
        return self.stack == obj.stack 
    def xml(self):
        """
        @return: stack depend instance represented as stack manifest XML
        @rtype: str
        """        
        return '<depend stack="%s" />'%self.stack

class ROSDep(object):
    """
    Manifest 'rosdep' tag    
    """
    __slots__ = ['name',]

    def __init__(self, name):
        """
        Create new rosdep instance.
        @param name: dependency name. Must be non-empty.
        @type  name: str
        """
        if not name or not isinstance(name, basestring):
            raise ValueError("bad 'name' attribute")
        self.name = name
    def xml(self):
        """
        @return: rosdep instance represented as manifest XML
        @rtype: str
        """        
        return '<rosdep name="%s" />'%self.name

class VersionControl(object):
    """
    Manifest 'versioncontrol' tag
    """
    __slots__ = ['type', 'url']

    def __init__(self, type_, url):
        """
        @param type_: version control type (e.g. 'svn'). must be non empty
        @type  type_: str
        @param url: URL associated with version control. must be non empty
        @type  url: str
        """
        if not type_ or not isinstance(type_, basestring):
            raise ValueError("bad 'type' attribute")
        if not url is None and not isinstance(url, basestring):
            raise ValueError("bad 'url' attribute")
        self.type = type_
        self.url = url
    def xml(self):
        """
        @return: versioncontrol instance represented as manifest XML
        @rtype: str
        """        
        if self.url:
            return '<versioncontrol type="%s" url="%s" />'%(self.type, self.url)
        else:
            return '<versioncontrol type="%s" />'%self.type
    
class _Manifest(object):
    """
    Object representation of a ROS manifest file
    """
    __slots__ = ['description', 'brief', \
                 'author', 'license', 'license_url', 'url', \
                 'depends', 'rosdeps',\
                 'logo', 'exports',\
                 'versioncontrol', 'status', 'notes',\
                 'unknown_tags',\
                 '_type']
    def __init__(self, _type='package'):
        self.description = self.brief = self.author = \
                           self.license = self.license_url = \
                           self.url = self.logo = self.status = self.notes = ''
        self.depends = []
        self.rosdeps = []
        self.exports = []
        self._type = _type
        
        # store unrecognized tags during parsing
        self.unknown_tags = []
        
    def __str__(self):
        return self.xml()
    def get_export(self, tag, attr):
        """
        @return: exports that match the specified tag and attribute, e.g. 'python', 'path'
        @rtype: [L{Export}]
        """
        return [e.get(attr) for e in self.exports if e.tag == tag if e.get(attr) is not None]
    def xml(self):
        """
        @return: Manifest instance as ROS XML manifest
        @rtype: str
        """
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
        rosdeps = '\n'.join(["  %s"%rd.xml() for rd in self.rosdeps])
        if self.exports:
            exports = '  <export>\n' + '\n'.join(["  %s"%e.xml() for e in self.exports]) + '  </export>'
        if self.versioncontrol:
            versioncontrol = "  %s"%self.versioncontrol.xml()
        if self.status or self.notes:
            review = '  <review status="%s" notes="%s" />'%(self.status, self.notes)
        fields = filter(lambda x: x, [desc, author, license, review, url, logo, depends, rosdeps, exports, versioncontrol])
        return "<%s>\n"%self._type + "\n".join(fields) + "\n</%s>"%self._type

def _get_text(nodes):
    """
    DOM utility routine for getting contents of text nodes
    """
    return "".join([n.data for n in nodes if n.nodeType == n.TEXT_NODE])

def parse_file(m, file):
    """
    Parse manifest file (package, stack)
    @param m: field to populate
    @type  m: L{_Manifest}
    @param file: manifest.xml file path
    @type  file: str
    @return: return m, populated with parsed fields
    @rtype: L{_Manifest}
    """
    if not file:
        raise ValueError("Missing manifest file argument")
    if not os.path.isfile(file):
        raise ValueError("Invalid/non-existent manifest file: %s"%file)
    f = open(file, 'r')
    try:
        text = f.read()
    finally:
        f.close()
    try:
        return parse(m, text, file)
    except ManifestException, e:
        raise ManifestException("Invalid manifest file [%s]: %s"%(os.path.abspath(file), e))

def parse(m, string, filename='string'):
    """
    Parse manifest.xml string contents
    @param string: manifest.xml contents
    @type  string: str
    @param m: field to populate
    @type  m: L{_Manifest}
    @return: return m, populated with parsed fields
    @rtype: L{_Manifest}
    """
    try:
        d = dom.parseString(string)
    except Exception, e:
        raise ManifestException("invalid XML: %s"%e)
    p = d.getElementsByTagName(m._type)
    if len(p) != 1:
        raise ManifestException("manifest must have a single '%s' element"%m._type)
    p = p[0]
    m.description = check('description')(p, filename)
    m.brief = ''
    try:
        tag = p.getElementsByTagName('description')[0]
        m.brief = tag.getAttribute('brief') or ''
    except:
        # means that 'description' tag is missing
        pass
    #TODO: figure out how to multiplex
    if m._type == 'package':
        m.depends = check_depends('depend')(p, filename)
    elif m._type == 'stack':
        m.depends = check_stack_depends('depend')(p, filename)
    elif m._type == 'app':
        # not implemented yet
        pass
    m.rosdeps = check('rosdep')(p, filename)    
    m.exports = check('export')(p, filename)
    m.versioncontrol = check('versioncontrol')(p,filename)
    m.license = check('license')(p, filename)
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

    m.author = check('author')(p, filename)
    m.url = check('url')(p, filename)
    m.logo = check('logo')(p, filename)

    # do some validation on what we just parsed
    if m._type == 'stack':
        if m.exports:
            raise ManifestException("stack manifests are not allowed to have exports")
        if m.rosdeps:
            raise ManifestException("stack manifests are not allowed to have rosdeps") 

    # store unrecognized tags
    m.unknown_tags = [e for e in p.childNodes if e.nodeType == e.ELEMENT_NODE and e.tagName not in VALID]
    return m
