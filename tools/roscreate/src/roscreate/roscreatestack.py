#!/usr/bin/env python
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
#
# Revision $Id$

from __future__ import with_statement
import roslib; roslib.load_manifest('roscreate')

NAME='roscreate-stack'

import os
import sys
import roslib.manifest
import roslib.packages
import roslib.stacks

from roscreate.core import read_template, author_name

def get_templates():
    templates = {}
    templates['stack.xml'] = read_template('stack.tmpl')
    templates['CMakeLists.txt'] = read_template('CMakeLists.stack.tmpl')
    templates['Makefile'] = read_template('Makefile.stack.tmpl')
    return templates

def instantiate_template(template, name, version, stack, brief, description, author, depends, licenses):
    return template%locals()

## @param stack str: name of stack
## @param stack_dir str: path to stack
## @param author str: name of stack maintainer
## @param depends dict {str: [str]}: map of stack name to packages that use that stack
## @param licenses set(str): list of licenses present in stack
def create_stack(stack, stack_dir, author, depends, licenses, show_deps):
    depends = ''.join(['  <depend stack="%s"/> <!-- %s --> \n'%(s, ', '.join(set(pkgs))) for s, pkgs in depends.iteritems()])
    if show_deps:
      print depends
      return
    licenses = ','.join(licenses)

    p = os.path.abspath(stack_dir)
    if not os.path.exists(p):
        print "Creating stack directory", p
        os.makedirs(p)

    templates = get_templates()
    for filename, template in templates.iteritems():
        contents = instantiate_template(template, stack, '0.1.0', stack, stack, stack, author, depends, licenses)
        p = os.path.abspath(os.path.join(stack_dir, filename))
        if not os.path.exists(filename) or filename == 'stack.xml':
            print "Creating stack file", p
            with open(p, 'w') as f:
                f.write(contents)
    print "\nPlease edit %s/stack.xml to finish creating your stack"%stack

def compute_stack_depends_and_licenses(stack, packages):
    pkg_depends = []
    licenses = []
    for pkg in packages:
        m = roslib.manifest.parse_file(roslib.manifest.manifest_file(pkg))
        pkg_depends.extend([d.package for d in m.depends])
        licenses.append(m.license)
        
    stack_depends = {}
    for pkg in pkg_depends:
        st = roslib.stacks.stack_of(pkg)
        if not st:
            print >> sys.stderr, "WARNING: stack depends on [%s], which is not in a stack"%pkg
            continue 
        if st == stack:
            continue
        if not st in stack_depends:
            stack_depends[st] = []            
        stack_depends[st].append(pkg)
    return stack_depends, set(licenses)

def roscreatestack_main():
    from optparse import OptionParser    
    parser = OptionParser(usage="usage: %prog <path-to-stack>", prog=NAME)
    parser.add_option("--show-deps", 
                      dest="show_deps", default=False,
                      action="store_true",
                      help="show stack dependencies, instead of generating stack.xml")
    options, args = parser.parse_args()
    if not args:
        parser.error("you must specify the path to a stack")
    stack_dir = args[0]
    stack = os.path.basename(os.path.abspath(stack_dir))

    # check whether or not stack directory exists
    try:
        if os.path.exists(stack_dir):
            packages = roslib.packages.list_pkgs(pkg_dirs=[os.path.abspath(stack_dir)])
            depends, licenses = compute_stack_depends_and_licenses(stack, packages)
        else:
            depends = dict()
            licenses = ['BSD']
    except roslib.packages.InvalidROSPkgException, e:
        print >> sys.stderr, str(e)
        sys.exit(1)

    if not options.show_deps:
      # Check for existing stack.xml
      stack_xml_path = os.path.join(stack_dir, 'stack.xml')
      if os.path.exists(stack_xml_path):
        import shutil
        stack_xml_path_bak = os.path.join(stack_dir, 'stack.xml.bak')
        print 'Backing up existing stack.xml to %s'%stack_xml_path_bak
        shutil.copyfile(stack_xml_path, stack_xml_path_bak)
  
    create_stack(stack, stack_dir, author_name(), depends, licenses, options.show_deps)

if __name__ == "__main__":
    roscreatestack_main()
