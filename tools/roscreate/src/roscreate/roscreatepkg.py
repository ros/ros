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

from __future__ import print_function

NAME='roscreate-pkg'

import os
import sys

import roslib.names

from roscreate.core import read_template, author_name
from rospkg import on_ros_path, RosPack, ResourceNotFound

def get_templates():
    templates = {}
    templates['CMakeLists.txt'] = read_template('CMakeLists.tmpl')
    templates['manifest.xml'] = read_template('manifest.tmpl')
    templates['mainpage.dox'] = read_template('mainpage.tmpl')
    templates['Makefile'] = read_template('Makefile.tmpl')
    return templates

def instantiate_template(template, package, brief, description, author, depends):
    return template%locals()

def create_package(package, author, depends, uses_roscpp=False, uses_rospy=False):
    p = os.path.abspath(package)
    if os.path.exists(p):
        print("%s already exists, aborting"%p, file=sys.stderr)
        sys.exit(1)

    os.makedirs(p)
    print("Created package directory", p)
        
    if uses_roscpp:
        # create package/include/package and package/src for roscpp code
        cpp_path = os.path.join(p, 'include', package)
        try:        
            os.makedirs(cpp_path)
            print("Created include directory", cpp_path)
            cpp_path = os.path.join(p, 'src')
            os.makedirs(cpp_path)
            print("Created cpp source directory", cpp_path)
        except:
            # file exists
            pass
    if uses_rospy:
        # create package/src/ for python files
        py_path = os.path.join(p, 'src')
        try:
            os.makedirs(py_path)
            print("Created python source directory", py_path)
        except:
            # file exists
            pass
        
    templates = get_templates()
    for filename, template in templates.iteritems():
        contents = instantiate_template(template, package, package, package, author, depends)
        p = os.path.abspath(os.path.join(package, filename))
        with open(p, 'w') as f:
            f.write(contents.encode('utf-8'))
            print("Created package file", p)
    print("\nPlease edit %s/manifest.xml and mainpage.dox to finish creating your package"%package)

def roscreatepkg_main():
    from optparse import OptionParser    
    parser = OptionParser(usage="usage: %prog <package-name> [dependencies...]", prog=NAME)
    options, args = parser.parse_args()
    if not args:
        parser.error("you must specify a package name and optionally also list package dependencies")
    package = args[0]

    if not roslib.names.is_legal_resource_base_name(package):
        parser.error("illegal package name: %s\nNames must start with a letter and contain only alphanumeric characters\nand underscores."%package)

    # validate dependencies and turn into XML
    depends = args[1:]
    uses_roscpp = 'roscpp' in depends
    uses_rospy = 'rospy' in depends

    rospack = RosPack()
    for d in depends:
        try:
            rospack.get_path(d)
        except ResourceNotFound:
            print("ERROR: dependency [%s] cannot be found"%d, file=sys.stderr)
            sys.exit(1)

    depends = u''.join([u'  <depend package="%s"/>\n'%d for d in depends])

    if not on_ros_path(os.getcwd()):
        print('!'*80+"\nWARNING: current working directory is not on ROS_PACKAGE_PATH!\nPlease update your ROS_PACKAGE_PATH environment variable.\n"+'!'*80, file=sys.stderr)
    if type(package) == str:
        package = package.decode('utf-8')
    create_package(package, author_name(), depends, uses_roscpp=uses_roscpp, uses_rospy=uses_rospy)
