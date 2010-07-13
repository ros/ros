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

from __future__ import with_statement

import os
import sys
from subprocess import Popen, PIPE

import roslib.packages
import roslib.rosenv
import roslib.manifest 
import roslib.rospack 
import roslib.stacks
import roslib.stack_manifest

class RosdocContext(object):
    def __init__(self, name, docdir, package_filters=None, path_filters=None):
        self.name = name
        self.package_filters = package_filters
        self.path_filters = []
        if path_filters:
            for p in path_filters.split(os.pathsep):
                if not p:
                    continue
                if not p.endswith(os.sep):
                    p = p + os.sep
                self.path_filters.append(p)
        self.docdir = docdir

        # these will be initialized in init()
        self.rosroot = self.ros_package_path = None
        self.packages = {}
        self.stacks = {}        
        self.external_docs = {}
        self.manifests = {}
        self.stack_manifests = {}
        # advanced per-package config
        self.rd_configs = {}                

        self.template_dir = None

    def has_builder(self, package, builder):
        """
        @return: True if package is configured to use builder. NOTE:
        if there is no config, package is assumed to define a doxygen
        builder
        @rtype: bool
        """
        rd_config = self.rd_configs.get(package, None)
        if not rd_config:
            return builder == 'doxygen'
        try:
            return len([d for d in rd_config if d['builder'] == builder]) > 0
        except KeyError:
            print >> sys.stderr, "config file for [%s] is invalid, missing required 'builder' key"%package
        
    def should_document(self, package):
        """
        @return: True if package should be documented
        @rtype: bool
        """
        if not package in self.packages:
            return False
        # package filters override all 
        if self.package_filters:
            return package in self.package_filters
        # don't document if not in path filters
        if self.path_filters:
            package_path = self.packages[package]
            if not [p for p in self.path_filters if package_path.startswith(p)]:
                return False
        # TODO: don't document if not in requested stack
        return True

    def init(self):
        self.rosroot = roslib.rosenv.get_ros_root(required=True)
        self.ros_package_path = roslib.rosenv.get_ros_package_path(required=False) or ''

        rosdoc_dir = roslib.packages.get_pkg_dir('rosdoc')
        self.template_dir = os.path.join(rosdoc_dir, 'templates')

        # use 'rospack list' to locate all packages and store their paths in a dictionary
        rospack_list = roslib.rospack.rospackexec(['list']).split('\n')
        rospack_list = [x.split(' ') for x in rospack_list if ' ' in x]

        # I'm still debating whether or not to immediately filter
        # these. The problem is that a package that is within the
        # filter may reference packages outside that filter. I'm not
        # sure if this is an issue or not.
        packages = self.packages
        for package, path in rospack_list:
            packages[package] = path

        self.doc_packages = [p for p in packages if self.should_document(p)]
        self._crawl_deps()
        
    def _crawl_deps(self):
        """
        Crawl manifest.xml dependencies
        """
        external_docs = self.external_docs
        manifests = self.manifests
        rd_configs = self.rd_configs

        stacks = self.stacks = {}

        # keep track of packages with invalid manifests so we can unregister them
        bad = []
        for package, path in self.packages.iteritems():

            # find stacks to document on demand
            if self.should_document(package):
                stack = roslib.stacks.stack_of(package) or ''
                if stack and stack not in stacks:
                    #print "adding stack [%s] to documentation"%stack
                    try:
                        p = roslib.stacks.get_stack_dir(stack)
                        stacks[stack] = p
                    except:
                        print >> sys.stderr, "cannot locate directory of stack [%s]"%stack
                
            f = os.path.join(path, roslib.manifest.MANIFEST_FILE)
            try:
                manifests[package] = m = roslib.manifest.parse_file(f)

                #NOTE: the behavior is undefined if the users uses
                #both config and export properties directly

                # #1650 for backwards compatibility, we ready the old
                # 'doxymaker' tag, which is deprecated
                #  - this is a loop but we only accept one value
                for e in m.get_export('doxymaker', 'external'):
                    external_docs[package] = e
                for e in m.get_export('rosdoc', 'external'):
                    external_docs[package] = e
                    
                # load in any external config files
                for e in m.get_export('rosdoc', 'config'):
                    import yaml
                    try:
                        e = e.replace('${prefix}', path)
                        config_p = os.path.join(path, e)
                        with open(config_p, 'r') as config_f:
                            rd_configs[package] = yaml.load(config_f)
                    except Exception, e:
                        print >> sys.stderr, "ERROR: unable to load rosdoc config file [%s]: %s"%(config_p, str(e))
                    

            except:
                print >> sys.stderr, "WARN: Package '%s' does not have a valid manifest.xml file, manifest information will not be included in docs"%package
                bad.append(package)

        for b in bad:
            if b in self.packages:
                del self.packages[b]
        stack_manifests = self.stack_manifests
        for stack, path in stacks.iteritems():

            f = os.path.join(path, roslib.stack_manifest.STACK_FILE)
            try:
                stack_manifests[stack] = roslib.stack_manifest.parse_file(f)
            except:
                import traceback
                traceback.print_exc()
                print >> sys.stderr, "WARN: Package '%s' does not have a valid manifest.xml file, manifest information will not be included in docs"%package
                

def compute_relative(src, target):
    s1, s2 = [p.split(os.sep) for p in [src, target]]
    #filter out empties
    s1, s2 = filter(lambda x: x, s1), filter(lambda x: x, s2)
    i = 0
    while i < min(len(s1), len(s2)):
        if s1[i] != s2[i]:
            break
        i+=1
    rel = ['..' for d in s1[i:]] + s2[i:]
    return os.sep.join(rel)

def html_path(package, docdir):
    return os.path.join(docdir, package, 'html')


################################################################################
# TEMPLATE ROUTINES

_TEMPLATES_DIR = 'templates'

def load_tmpl(filename):
    filename = os.path.join(roslib.packages.get_pkg_dir('rosdoc'), _TEMPLATES_DIR, filename)
    if not os.path.isfile(filename):
        print >> sys.stderr, "Cannot locate template file '%s'"%filename
        sys.exit(1)
    f = open(filename, 'r')
    try:
        str = f.read()
        if not str:
            print >> sys.stderr, "Template file '%s' is empty"%filename
            sys.exit(1)
        return str
    finally:
        f.close()

def li_package_links(ctx, package, packages, docdir, package_htmldir=None):
    """
    @param package: current package
    @type  package: str
    @param packages: list of packages to generate 'li' html links to
    @type  packages: [str]
    """
    # package_htmldir can be overridden by rosdoc config
    package_htmldir = package_htmldir or html_path(package, docdir)
    
    # don't link to packages that aren't documentable
    documented_packages = [p for p in packages if ctx.should_document(p)]
    undocumented_packages = [p for p in packages if not ctx.should_document(p)]
    
    documented = '\n'.join(['  <li><a href="%s/index.html">%s</a></li>'%\
                                (compute_relative(package_htmldir, html_path(p, docdir)), p) for p in documented_packages])
    undocumented = '\n'.join(['  <li>%s</li>'%p for p in undocumented_packages])
    return documented + undocumented + "\n</ul>"
            

def instantiate_template(tmpl, vars):
    for k, v in vars.iteritems():
        try:
            tmpl = tmpl.replace(k, v)
        except:
            # soft fail
            traceback.print_exc()
    return tmpl


################################################################################
# ROS package utilities

def generate_package_tree(ctx):
    """
    Generate a tree representation of the available packages
    """
    remaining_packages = set(ctx.doc_packages)
    
    # step 1: assign stack packages
    stacks = roslib.stacks.list_stacks()
    # - fill out tree structure for packages in stacks
    packagetree = {}
    found_packages = set()
    for s in stacks:
        packagetree[s] = d = {}
        packages = roslib.stacks.packages_of(s)
        for p in packages:
            d[p] = {}
            found_packages.add(p)
    remaining_packages = set(ctx.doc_packages) - found_packages

    # step 2: walk remaining packages
    def _gt_visitor(base, d, dirs, files):
        if os.path.isfile(os.path.join(d, 'manifest.xml')):
            package = os.path.basename(d)
            if package in remaining_packages:
                visited.append(package)
                rel = d[len(base):]
                treepath = [t for t in rel.split(os.sep) if t]
                node = packagetree
                for t in treepath:
                    if not node.has_key(t):
                        node[t] = {}
                    node = node[t]
                return True
            else:
                # ignore 
                pass
        elif os.path.isfile(os.path.join(d, 'rospack_nosubdirs')):
            return True
        return False

    packages = ctx.packages
    rosroot = ctx.rosroot
    ros_package_path = ctx.ros_package_path
    
    visited = []
    paths = [rosroot] + ros_package_path.split(os.pathsep)
    paths = [p for p in paths if p]
    for p in paths:
        for d, dirs, files in os.walk(p, topdown=True):
            if _gt_visitor(p, d, dirs, files):
                del dirs[:] #don't descend further

    return packagetree

