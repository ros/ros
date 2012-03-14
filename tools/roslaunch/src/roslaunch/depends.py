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

"""
Utility module of roslaunch that extracts dependency information from
roslaunch files, including calculating missing package dependencies.
"""

from __future__ import print_function

import os
import sys

from xml.dom.minidom import parse
from xml.dom import Node as DomNode

import rospkg

from .substitution_args import resolve_args

NAME="roslaunch-deps"

class RoslaunchDepsException(Exception):
    """
    Base exception of roslaunch.depends errors.
    """
    pass

class RoslaunchDeps(object):
    """
    Represents dependencies of a roslaunch file. 
    """
    def __init__(self, nodes=None, includes=None, pkgs=None):
        if nodes == None:
            nodes = []
        if includes == None:
            includes = []
        if pkgs == None:
            pkgs = []
        self.nodes = nodes
        self.includes = includes
        self.pkgs = pkgs
        
    def __eq__(self, other):
        if not isinstance(other, RoslaunchDeps):
            return False
        return set(self.nodes) == set(other.nodes) and \
               set(self.includes) == set(other.includes) and \
               set(self.pkgs) == set(other.pkgs)

    def __repr__(self):
        return "nodes: %s\nincludes: %s\npkgs: %s"%(str(self.nodes), str(self.includes), str(self.pkgs))

    def __str__(self):
        return "nodes: %s\nincludes: %s\npkgs: %s"%(str(self.nodes), str(self.includes), str(self.pkgs))

def _parse_launch(tags, launch_file, file_deps, verbose):
    dir_path = os.path.dirname(os.path.abspath(launch_file))
    launch_file_pkg = rospkg.get_package_name(dir_path)
            
    # process group, include, node, and test tags from launch file
    for tag in [t for t in tags if t.nodeType == DomNode.ELEMENT_NODE]:

        if tag.tagName == 'group':
            
            #descend group tags as they can contain node tags
            _parse_launch(tag.childNodes, launch_file, file_deps, verbose)

        elif tag.tagName == 'include':
            try:
                sub_launch_file = resolve_args(tag.attributes['file'].value)
            except KeyError as e:
                raise RoslaunchDepsException("Cannot load roslaunch <%s> tag: missing required attribute %s.\nXML is %s"%(tag.tagName, str(e), tag.toxml()))
                
            if verbose:
                print("processing included launch %s"%sub_launch_file)

            # determine package dependency for included file
            sub_pkg = rospkg.get_package_name(os.path.dirname(os.path.abspath(sub_launch_file)))
            if sub_pkg is None:
                print("ERROR: cannot determine package for [%s]"%sub_launch_file, file=sys.stderr)
            
            if sub_launch_file not in file_deps[launch_file].includes:
                file_deps[launch_file].includes.append(sub_launch_file)
            if launch_file_pkg != sub_pkg:            
                file_deps[launch_file].pkgs.append(sub_pkg)
            
            # recurse
            file_deps[sub_launch_file] = RoslaunchDeps()
            try:
                dom = parse(sub_launch_file).getElementsByTagName('launch')
                if not len(dom):
                    print("ERROR: %s is not a valid roslaunch file"%sub_launch_file, file=sys.stderr)
                else:
                    launch_tag = dom[0]
                    _parse_launch(launch_tag.childNodes, sub_launch_file, file_deps, verbose)
            except IOError as e:
                raise RoslaunchDepsException("Cannot load roslaunch include '%s' in '%s'"%(sub_launch_file, launch_file))

        elif tag.tagName in ['node', 'test']:
            try:
                pkg, type = [resolve_args(tag.attributes[a].value) for a in ['pkg', 'type']]
            except KeyError as e:
                raise RoslaunchDepsException("Cannot load roslaunch <%s> tag: missing required attribute %s.\nXML is %s"%(tag.tagName, str(e), tag.toxml()))
            if (pkg, type) not in file_deps[launch_file].nodes:
                file_deps[launch_file].nodes.append((pkg, type))
            # we actually want to include the package itself if that's referenced
            #if launch_file_pkg != pkg:
            if pkg not in file_deps[launch_file].pkgs:
                file_deps[launch_file].pkgs.append(pkg)
            
def parse_launch(launch_file, file_deps, verbose):
    if verbose:
        print("processing launch %s"%launch_file)

    try:
        dom = parse(launch_file).getElementsByTagName('launch')
    except:
        raise RoslaunchDepsException("invalid XML in %s"%(launch_file))
    if not len(dom):
        raise RoslaunchDepsException("invalid XML in %s"%(launch_file))

    file_deps[launch_file] = RoslaunchDeps()
    launch_tag = dom[0]
    _parse_launch(launch_tag.childNodes, launch_file, file_deps, verbose)

def rl_file_deps(file_deps, launch_file, verbose=False):
    """
    Generate file_deps file dependency info for the specified
    roslaunch file and its dependencies.
    @param file_deps: dictionary mapping roslaunch filenames to
        roslaunch dependency information. This dictionary will be
        updated with dependency information.
    @type  file_deps: { str : RoslaunchDeps } 
    @param verbose: if True, print verbose output
    @type  verbose: bool
    @param launch_file: name of roslaunch file
    @type  launch_file: str
    """
    parse_launch(launch_file, file_deps, verbose)
    
def fullusage():
    name = NAME
    print("""Usage:
\t%(name)s [options] <file-or-package>
"""%locals())

def print_deps(base_pkg, file_deps, verbose):
    pkgs = []

    # for verbose output we print extra source information
    if verbose:
        for f, deps in file_deps.iteritems():
            for p, t in deps.nodes:
                print("%s [%s/%s]"%(p, p, t))

            pkg = rospkg.get_package_name(os.path.dirname(os.path.abspath(f)))
            if pkg is None: #cannot determine package
                print("ERROR: cannot determine package for [%s]"%pkg, file=sys.stderr)
            else:
                print("%s [%s]"%(pkg, f))
        print('-'*80)

    # print out list of package dependencies
    pkgs = [] 
    for deps in file_deps.itervalues():
        pkgs.extend(deps.pkgs)
    # print space-separated to be friendly to rosmake
    print(' '.join([p for p in set(pkgs)]))

def calculate_missing(base_pkg, missing, file_deps):
    """
    Calculate missing package dependencies in the manifest. This is
    mainly used as a subroutine of roslaunch_deps().
    
    @param base_pkg: name of package where initial walk begins (unused).
    @type  base_pkg: str
    @param missing: dictionary mapping package names to set of missing package dependencies.
    @type  missing: { str: set(str) }
    @param file_deps: dictionary mapping launch file names to RoslaunchDeps of each file
    @type  file_deps: { str: RoslaunchDeps}
    @return: missing (see parameter)
    @rtype: { str: set(str) }
    """
    rospack = rospkg.RosPack()
    for launch_file in file_deps.iterkeys():
        pkg = rospkg.get_package_name(os.path.dirname(os.path.abspath(launch_file)))

        if pkg is None: #cannot determine package
            print("ERROR: cannot determine package for [%s]"%pkg, file=sys.stderr)
            continue
        m = rospack.get_manifest(pkg)
        d_pkgs = set([d.name for d in m.depends])
        # make sure we don't count ourselves as a dep
        d_pkgs.add(pkg)

        diff = list(set(file_deps[launch_file].pkgs) - d_pkgs)
        if not pkg in missing:
            missing[pkg] = set()
        missing[pkg].update(diff)
    return missing
        
    
def roslaunch_deps(files, verbose=False):
    """
    @param packages: list of packages to check
    @type  packages: [str]
    @param files [str]: list of roslaunch files to check. Must be in
      same package.
    @type  files: [str]
    @return: base_pkg, file_deps, missing.
      base_pkg is the package of all files
      file_deps is a { filename : RoslaunchDeps } dictionary of
        roslaunch dependency information to update, indexed by roslaunch
        file name.
       missing is a { package : [packages] } dictionary of missing
        manifest dependencies, indexed by package.
    @rtype: str, dict, dict
    """
    file_deps = {}
    missing = {}
    base_pkg = None

    for launch_file in files:
        if not os.path.exists(launch_file):
            raise RoslaunchDepsException("roslaunch file [%s] does not exist"%launch_file)

        pkg = rospkg.get_package_name(os.path.dirname(os.path.abspath(launch_file)))
        if base_pkg and pkg != base_pkg:
            raise RoslaunchDepsException("roslaunch files must be in the same package: %s vs. %s"%(base_pkg, pkg))
        base_pkg = pkg
        rl_file_deps(file_deps, launch_file, verbose)

    calculate_missing(base_pkg, missing, file_deps)
    return base_pkg, file_deps, missing            
    
def roslaunch_deps_main(argv=sys.argv):
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog [options] <file(s)...>", prog=NAME)
    parser.add_option("--verbose", "-v", action="store_true",
                      dest="verbose", default=False, 
                      help="Verbose output")
    parser.add_option("--warn", "-w", action="store_true",
                      dest="warn", default=False, 
                      help="Warn about missing manifest dependencies")
        
    (options, args) = parser.parse_args(argv[1:])
    if not args:
        parser.error('please specify a launch file')

    files = [arg for arg in args if os.path.exists(arg)]
    packages = [arg for arg in args if not os.path.exists(arg)]
    if packages:
        parser.error("cannot locate %s"%','.join(packages))
    try:
        base_pkg, file_deps, missing = roslaunch_deps(files, verbose=options.verbose)
    except RoslaunchDepsException as e:
        print(sys.stderr, str(e))
        sys.exit(1)

    if options.warn:
        print("Dependencies:")
        
    print_deps(base_pkg, file_deps, options.verbose)
    
    if options.warn:
        print('\nMissing declarations:')
        for pkg, miss in missing.iteritems():
            if miss:
                print("%s/manifest.xml:"%pkg)
                for m in miss:
                    print('  <depend package="%s" />'%m)
    
