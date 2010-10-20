# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

"""
Library for process rosdistro files.

New in ROS C-Turtle
"""

from __future__ import with_statement

import os
import sys
import yaml
import urllib2
import re
import string

import warnings
warnings.warn("roslib.distro is deprecated. Please use rosdistro package instead", DeprecationWarning, stacklevel=2)

class DistroException(Exception): pass

def distro_version(version_val):
    """Parse distro version value, converting SVN revision to version value if necessary"""
    version_val = str(version_val)
    m = re.search('\$Revision:\s*([0-9]*)\s*\$', version_val)
    if m is not None:
        version_val = 'r'+m.group(1)

    # Check that is a valid version string
    valid = string.ascii_letters + string.digits + '.+~'
    if False in (c in valid for c in version_val):
        raise DistroException("Version string %s not valid"%self.version)
    return version_val

def expand_rule(rule, stack_name, stack_ver, release_name, revision=None):
    s = rule.replace('$STACK_NAME', stack_name)
    s =    s.replace('$STACK_VERSION', stack_ver)
    s =    s.replace('$RELEASE_NAME', release_name)
    if s.find('$REVISION') > 0 and not revision:
        raise DistroException("revision specified but not supplied by build_release")
    elif revision:
        s = s.replace('$REVISION', revision)
    return s

def get_variants(distro, stack_name):
    """
    Retrieve names of variants that stack is present in. This operates
    on the raw distro dictionary document.
    
    @param distro: rosdistro document
    @type  distro: dict
    """
    if stack_name == 'ROS':
        stack_name = 'ros'

    retval = []
    variants = distro.get('variants', {})
    
    for variant_d in variants:
        try:
            variant = variant_d.keys()[0]
            variant_props = variant_d[variant]
            if stack_name in variant_props['stacks']:
                retval.append(variant)
            elif 'extends' in variant_props and variant_props['extends'] in retval:
                retval.append(variant)                
        except:
            pass
    return retval

# TODO: integrate with Distro
def get_rules(distro, stack_name):
    """
    Retrieve rules from distro for specified stack This operates on
    the raw distro dictionary document.

    @param distro: rosdistro document
    @type  distro: dict
    @param stack_name: name of stack to get rules for
    @type  stack_name: str
    """
    if stack_name == 'ROS':
        stack_name = 'ros'
    # there are three tiers of dictionaries that we look in for uri rules
    rules_d = [distro.get('stacks', {}),
               distro.get('stacks', {}).get(stack_name, {})]
    rules_d = [d for d in rules_d if d]
    # load the '_rules' from the dictionaries, in order
    props = {}
    for d in rules_d:
        if type(d) == dict:
            props.update(d.get('_rules', {}))

    if not props:
        raise Exception("cannot load _rules")
    return props
        
def load_distro_stacks(distro_doc, stack_names, release_name=None, version=None):
    """
    @param distro_doc: dictionary form of rosdistro file
    @type distro_doc: dict
    @param stack_names: names of stacks to load
    @type  stack_names: [str]
    @param release_name: (optional) name of distro release to override distro_doc spec.
    @type  release_name: str
    @param version: (optional) distro version to override distro_doc spec.
    @type  version: str
    @return: dictionary of stack names to DistroStack instances
    @rtype: {str : DistroStack}
    @raise DistroException: if distro_doc format is invalid
    """

    # load stacks and expand out uri rules
    stacks = {}
    # we pass these in mostly for small performance reasons, as well as testing
    if version is None:
        version = distro_version(distro_doc.get('version', '0'))        
    if release_name is None:
        release_name = distro_doc['release']

    try:
        stack_props = distro_doc['stacks']
    except KeyError:
        raise DistroException("distro is missing required 'stacks' key")
    for stack_name in stack_names:
        # ignore private keys like _rules
        if stack_name[0] == '_':
            continue

        try:
            stack_version = stack_props[stack_name]['version']
        except KeyError:
            raise DistroException("stack [%s] is missing version key"%stack_name)

        rules = get_rules(distro_doc, stack_name)
        stacks[stack_name] = DistroStack(stack_name, rules, stack_version, release_name, version)
    return stacks

class DistroStack(object):
    """Stores information about a stack release"""

    def __init__(self, stack_name, rules, stack_version, release_name, release_version):
        self.name = stack_name
        self.version = stack_version

        #debian-specific stuff
        #TODO: move to rosdeb
        try:
            self.debian_version = debianize_version(stack_version, release_version)
            self.debian_name = debianize_name("ros-%s-%s"%(release_name,stack_name))
        except DistroException:
            # ignore on non debian systems. This really belongs in an extension
            self.debian_version = self.debian_name = None

        #rosdistro key
        self.dev_svn = expand_rule(rules['dev-svn'], stack_name, stack_version, release_name)
        self.distro_svn = expand_rule(rules['distro-svn'], stack_name, stack_version, release_name)
        self.release_svn = expand_rule(rules['release-svn'], stack_name, stack_version, release_name)
        # for password-protected repos
        self.user_svn = rules.get('user-svn', None)
        self.pass_svn = rules.get('pass-svn', None)
    
        if 'source-tarball' in rules:
            self.source_tarball = expand_rule(rules['source-tarball'], stack_name, stack_version, release_name)
        else:
            self.source_tarball = None

    def __eq__(self, other):
        if not isinstance(other, DistroStack):
            return False
        return self.name == other.name and \
            self.version == other.version and \
            self.debian_version == other.debian_version and \
            self.debian_name == other.debian_name and \
            self.dev_svn == other.dev_svn and \
            self.distro_svn == other.distro_svn and \
            self.release_svn == other.release_svn and \
            self.user_svn == other.user_svn and \
            self.pass_svn == other.pass_svn and \
            self.source_tarball == other.source_tarball

class Variant(object):
    """
    A variant defines a specific set of stacks ("metapackage", in Debian
    parlance). For example, "base", "pr2". These variants can extend
    another variant.
    """

    def __init__(self, variant_name, source_uri, variants_props):
        """
        @param variant_name: name of variant to load from distro file
        @type  variant_name: str
        @param source_uri: source URI of distro file
        @type  source_uri: str
        @param variants_props: dictionary mapping variant names to the rosdistro map for that variant
        """
        self.name = variant_name
        self.source_uri = source_uri

        # save the properties for our particular variant
        try:
            props = variants_props[variant_name]
        except:
            raise DistroException("distro does not define a '%s' variant"%variant_name)

        # load in variant properties from distro spec
        if not 'stacks' in props:
            raise DistroException("variant properties must define 'stacks':\n%s"%props[n])
        self.stack_names = list(props['stacks'])

        # check to see if we extend another distro, in which case we prepend their props
        if 'extends' in props:
            self.parent = props['extends']
            self.parent_uri = props.get('extends-uri', source_uri)
            parent_variant = Variant(self.parent, self.parent_uri, variants_props)
            self.stack_names = parent_variant.stack_names + self.stack_names
        self.props = props
      
class Distro(object):
    """
    Store information in a rosdistro file.
    """
    
    def __init__(self, source_uri):
        """
        @param source_uri: source URI of distro file, or path to distro file
        """
        # initialize members
        self.source_uri = source_uri

        self.ros = None
        self.stacks = {} # {str: DistroStack}
        self.stack_names = [] 
        self.variants = {}
        self.distro_props = None

        try:
            # load rosdistro file
            if os.path.isfile(source_uri):
                with open(source_uri) as f:
                    y = yaml.load(f.read())
            else:
                y = yaml.load(urllib2.urlopen(source_uri))
            self.distro_props = y
  
            stack_props = y['stacks']
            self.stack_names = [x for x in stack_props.keys() if not x[0] == '_']
            self.version = distro_version(y.get('version', '0'))
            self.release_name = y['release']
  
            variants = {}
            for props in y['variants']:
                if len(props.keys()) != 1:
                    raise DistroException("invalid variant spec: %s"%props)
                n = props.keys()[0]
                variants[n] = props[n]
  
        except KeyError, e:
            raise DistroException("this program assumes the yaml has a '%s' map"%(str(e)))

        # load variants
        for v in variants.iterkeys():
            self.variants[v] = Variant(v, source_uri, variants)

        if not 'ros' in stack_props:
            raise DistroException("this program assumes that ros is in your variant")

        self.stacks = load_distro_stacks(self.distro_props, self.stack_names, release_name=self.release_name, version=self.version)
        self.ros = self.stacks.get('ros', None)

    
################################################################################
# DEBIAN-SPECIFIC STUFF
# TODO: move to rosdeb

_ubuntu_map = { '10.10': 'mighty', '10.04': 'lucid', '9.10': 'karmic', '9.04': 'jaunty', '8.10': 'intrepid', '8.04': 'hardy'}
def ubuntu_release():
    """
    WARNING: this can only be called on an Ubuntu system
    """
    if not os.path.isfile('/etc/issue'):
        raise DistroException("this is not an ubuntu system")        
    f = open('/etc/issue')
    for s in f:
        if s.startswith('Ubuntu'):
            v = s.split()[1]
            v = '.'.join(v.split('.')[:2])
        try:
            return _ubuntu_map[v]
        except KeyError:
            raise DistroException("unrecognized ubuntu version %s" % v)
    raise DistroException("could not parse ubuntu release version")

def debianize_name(name):
    """
    Convert ROS stack name to debian conventions (dashes, not underscores)
    """
    return name.replace('_', '-')

def debianize_version(stack_version, distro_version, ubuntu_rel=None):
    """
    WARNING: this can only be called on an Ubuntu system
    """
    if ubuntu_rel is None:
        ubuntu_rel = ubuntu_release()
    return stack_version+'-'+distro_version+'~%s'%ubuntu_rel
