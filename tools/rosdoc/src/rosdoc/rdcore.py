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
# Revision $Id: doxyutil.py 3727 2009-02-06 22:42:26Z sfkwc $

import os
import sys
from subprocess import Popen, PIPE

import roslib.packages
import roslib.rosenv
import roslib.manifest 
import roslib.scriptutil as scriptutil

class RosdocContext(object):
    def __init__(self, docdir, filters):
        self.filters = filters
        self.docdir = docdir

        # these will be initialized in init()
        self.rosroot = self.ros_package_path = None
        self.packages = {}
        self.external_docs = {}
        self.manifests = {}

        self.template_dir = None

    def init(self):
        self.rosroot = roslib.rosenv.get_ros_root(True)
        self.ros_package_path = roslib.rosenv.get_ros_package_path(False) or ''

        rosdoc_dir = roslib.packages.get_pkg_dir('rosdoc')
        self.template_dir = os.path.join(rosdoc_dir, 'templates')

        # use 'rospack list' to locate all packages and store their paths in a dictionary
        rospack_list = scriptutil.rospackexec(['list']).split('\n')
        rospack_list = [x.split(' ') for x in rospack_list if ' ' in x]

        packages = self.packages
        for package, path in rospack_list:
            packages[package] = path
            
        self._crawl_deps()
        
    ## Crawl manifest.xml dependencies
    def _crawl_deps(self):
        external_docs = self.external_docs
        manifests = self.manifests
        
        for package, path in self.packages.iteritems():
            f = os.path.join(path, roslib.manifest.MANIFEST_FILE)
            try:
                manifests[package] = m = roslib.manifest.parse_file(f)
                # this is a loop but we only accept one value
                for e in m.get_export('doxymaker', 'external'):
                    external_docs[package] = e
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

## @param package str: current package
## @param packages [str]: list of packages to generate 'li' html links to
def li_package_links(package, packages, docdir):
    curr_path = html_path(package, docdir)
    return '\n'.join(['  <li><a href="%s/index.html">%s</a></li>'%\
                      (compute_relative(curr_path, html_path(p, docdir)), p) for p in packages])+"\n</ul>"
            

def instantiate_template(tmpl, vars):
    for k, v in vars.iteritems():
        tmpl = tmpl.replace(k, v)
    return tmpl


################################################################################
# ROS package utilities

## generate a tree representation of the available packages
def generate_package_tree(ctx):
    def _gt_visitor(base, d, dirs, files):
        if os.path.isfile(os.path.join(d, 'manifest.xml')):
            package = os.path.basename(d)
            if package in ctx.packages:
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
                print "Rogue package", d
        elif os.path.isfile(os.path.join(d, 'rospack_nosubdirs')):
            return True
        return False

    packages = ctx.packages
    rosroot = ctx.rosroot
    ros_package_path = ctx.ros_package_path
    
    packagetree = {}
    visited = []
    paths = [rosroot] + ros_package_path.split(os.pathsep)
    paths = [p for p in paths if p]
    for p in paths:
        for d, dirs, files in os.walk(p, topdown=True):
            if _gt_visitor(p, d, dirs, files):
                del dirs[:] #don't descend further

    # do some sanity checks on the code above to make sure our package lists agrees
    problempackages = set(visited) ^ set(ctx.packages.keys())
    if problempackages:
        print "WARNING: doxymaker and rospack list disagree about the existence of the following packages:"
        for p in problempackages:
            print " * %s"%p
    return packagetree

