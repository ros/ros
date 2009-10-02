#! /usr/bin/env python

# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import with_statement

import os
import re
import distutils.version
import sys, string
import subprocess
import getopt
import roslib
import roslib.scriptutil
import roslib.rosenv

from math import sqrt
from optparse import OptionParser

## Build the dictionary of dependencies for a list of packages
## @param names [str]: list of package names to target. If empty, all packages will be targetted.
## @param target1 bool: if True, only target direct dependencies for packages listed in \a names
def build_dictionary(names, exclude, target1=False):
    pkgs = set()
    ## If no arguments list all packages
    if not names:
        pkgs = roslib.packages.list_pkgs()
        
    ## Get all dependencies and depends of target packages
    else:
        for name in names:
            if target1:
                pkgs.update(roslib.scriptutil.rospack_depends_1(name))
                pkgs.update(roslib.scriptutil.rospack_depends_on_1(name))
            else:
                pkgs.update(roslib.scriptutil.rospack_depends(name))
                pkgs.update(roslib.scriptutil.rospack_depends_on(name))

        pkgs.update(names)
    pkgs = [p for p in pkgs if p not in exclude]
    ## Build the dictionary of dependencies
    dict = {}
    for pkg in pkgs:
        dict[pkg] = [d for d in roslib.scriptutil.rospack_depends_1(pkg) if not d in exclude]
    return dict

## Get all the dependencies of dependent packages (deduplicated)
def get_child_deps(d, name):
    accum = set()
    for dep in d[name]:
        accum.update(get_deps(d, dep))
    return accum
                     
## Get all dependencies of this package recursively (deduplicated)
def get_deps(d, name):
    accum = set()
    try:
        for dep in d[name]:
            accum.add(dep)
            accum.update(get_deps(d, dep))
    except KeyError:
        pass #print "bad Key" # don't accumulate deps for a package not in the tree
    return accum

## Load color data from file
def get_color(filename):
    color_dictionary = {}
    try:
        with open(filename,'r') as color_file:
            for line in color_file:
                try:
                    k, v = line.split()
                    color_dictionary[k] = v
                except:
                    print "ERROR: badly formatted color line: %s"%line
    except IOError:
        print "Couldn't open color file: \"%s\""%filename
    return color_dictionary
        
license_map = {
    "bsd": "green",
    "zlib": "green",
    "apache": "green",
    "free": "green",
    "bsl1.0": "green",
    "lgpl": "dodgerblue",
    "mit": "dodgerblue",
    "gpl": "orange",
    "research-only": "red",
    }

## Get colors from license status
def get_license_color():
    color_map = {}
    for pkg in roslib.packages.list_pkgs():
        license = roslib.manifest.parse_file(roslib.manifest.manifest_file(pkg)).license.lower()
        color_map[pkg] = license_map.get(license, 'purple')
        if not license in license_map:
            print "Unknown: ", pkg, license
    return color_map

status_map = {
    "doc reviewed": "green",
    "api cleared": "dodgerblue",
    "api conditionally cleared": "orange",
    "proposal cleared": "pink",
    "unreviewed": "red",
    "experimental": "yellow",
    "3rdparty": "white",
    "3rdparty doc reviewed": "green",
    "na": "gray85",
    "test": "gray85",
    "deprecated": "magenta",        
    "ROS_BUILD_BLACKLIST": "black"
}

## Get the colors from package review status
def get_status_color():
    color_dict = {}
    notes_dict = {}
    for pkg in roslib.packages.list_pkgs():
        f = roslib.manifest.manifest_file(pkg)
        status = roslib.manifest.parse_file(f).status.lower()
        notes_dict[pkg] = roslib.manifest.parse_file(f).notes
        color_dict[pkg] = status_map.get(status, 'purple')
        # show blacklisting
        if os.path.exists(os.path.join(os.path.dirname(f), "ROS_BUILD_BLACKLIST")):
            print f, "is blacklisted"
            color_dict[pkg] = 'black'
            notes_dict[pkg] +=" ROS_BUILD_BLACKLIST"
        # show ROS_NOBUILD flag
        if os.path.exists(os.path.join(os.path.dirname(f), "ROS_NOBUILD")):
            print f, "is installed"
            notes_dict[pkg] +=" ROS_NOBUILD"
    return color_dict, notes_dict

rosmakeall_color_map = { "rosmakeall-testfailures.txt": "orange", "rosmakeall-buildfailures.txt": "red"}

def get_rosmakeall_color():
    path = roslib.rosenv.get_ros_root()
    color_dict = {}
    for f in rosmakeall_color_map:
        c = rosmakeall_color_map[f]
        try:
            with open(path+"/"+f) as input:
                for l in input:
                    color_dict[l.strip()] = c
        except IOError:
            print >> sys.stderr, "Couldn't find %s"%f
            sys.exit(1)
    return color_dict

def get_depth(pkg_dict, pkg, depth):
    for pkg_dep in pkg_dict[pkg]:
    	depth = max(depth, get_depth(pkg_dict, pkg_dep, depth + 1))
    return depth

def build_rank(pkg_dict):
    rank = {}
    for pkg in pkg_dict:
    	depth = get_depth(pkg_dict, pkg, 0)
	if depth in rank:
	   rank[depth].append(pkg)
        else:
	   rank[depth] = [pkg]

    return rank

def classify_packages(pkg_dict):
    classes = {}
    stacks = []
    found_stack = ''
    stack_dict = {}
    try:
        # Check version, make postscript if too old to make pdf
        args = ["rosstack", "list"]
        vstr, verr = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
        stack_entries = vstr.split('\n')
        stacks = []
        stack_paths = {}
        for entry in stack_entries:
            if  len(entry.split()) == 2:
                n, p = entry.split()
                stacks.append(n)
                stack_paths[n] = p
            elif entry.strip():
                print >> sys.stderr, "rosstack list returned wrong number of arguments %d not the expected 2: %s"%len(entry.split(), entry)
        #print stacks, stack_paths
        #sys.exit(-1)

        for pkg in pkg_dict:
            pkg_path = os.path.realpath(roslib.packages.get_pkg_dir(pkg))

            found_stack = "Unreleased"
            for stack in stacks:
                realpath_stack = os.path.realpath(stack_paths[stack])+"/" # slash needed to prevent partial stack name matches aka ros matching ros-pkg
                if pkg_path.startswith(realpath_stack):
                    #print "matching %s with %s"%(pkg_path, realpath_stack)
                    found_stack = stack
                    break

            # record the path to the stack
            if found_stack == "Unreleased":
                stack_dict["Unreleased"] = "Not Contained in a stack"
            else:
                #print "adding package %s to stack %s with path %s"%(pkg, found_stack, stack_paths[found_stack])
                stack_dict[found_stack] = stack_paths[found_stack]

            # Record the stack on the list of classes
            if found_stack in classes:
               classes[found_stack].append (pkg)
            else:
               classes[found_stack] = [pkg]
    except subprocess.CalledProcessError:
        print >> sys.stderr, "failed to call [rosstack contents %s]"%stack_string
    return (classes, stack_dict)

def vdmain():
    parser = OptionParser(usage="usage: %prog [options]", prog='rxdeps')
    parser.add_option("-r", "--rosmake", dest="rosmake", default=False,
                      action="store_true", help="color by rosmakeall results")
    parser.add_option("-s", "--review", dest="review_status", default=False,
                      action="store_true", help="color by review status")
    parser.add_option("--size", dest="size_by_deps", default=False,
                      action="store_true", help="whether to size by number of depends on 1")
    parser.add_option("-l", "--license", dest="license", default=False,
                      action="store_true", help="color by license type")
    parser.add_option("-v", dest="verbose", default=False,
                      action="store_true", help="don't deduplicate dependencies")
    parser.add_option("--one", dest="target1", default=False,
                      action="store_true", help="only draw one away from target")
    parser.add_option("-q", "--quiet", dest="quiet", default=False,
                      action="store_true", help="quiet (remove links to common packages)")
    parser.add_option("--hide_single", dest="hide", default=False,
                      action="store_true", help="hide (remove packages without links)")
    parser.add_option("--rank", dest="rank", default=False,
                      action="store_true", help="layer by maximum dependency depth")
    parser.add_option("--cluster", dest="cluster", default=False,
                      action="store_true", help="group by main subdir")
    parser.add_option("--color", metavar="FILENAME",
                      dest="color_file", default=None, 
                      type="string", help="use color file") 
    parser.add_option("-o", metavar="FILENAME",
                      dest="output_filename", default=None, 
                      type="string", help="output_filename") 
    parser.add_option("--target", metavar="PKG_NAMES_LIST,COMMA_SEPERATED",
                      dest="targets", default='', 
                      type="string", help="target packages")
    parser.add_option("--stack", metavar="STACK_NAMES_LIST,COMMA_SEPERATED",
                      dest="stacks", default='', 
                      type="string", help="target stacks")
    parser.add_option("--exclude", metavar="PKG_NAMES_LIST,COMMA_SEPERATED",
                      dest="exclude", default='', 
                      type="string", help="exclude packages")
    parser.add_option("--message", metavar="full_message_name",
                      dest="message", default='', 
                      type="string", help="target_message")


    options, args = parser.parse_args()
    if options.exclude:
        exclude = options.exclude.split(',')
    else:
        exclude = []
    if options.targets:
        targets = options.targets.split(',')
    else:
        targets = []
    if options.stacks:
        stacks = options.stacks.split(',')
    else:
        stacks = []
    if options.quiet:
        exclude.extend(['roscpp','std_msgs', 'rospy', 'std_srvs', 'robot_msgs', 'robot_srvs', 'rosconsole', 'tf'])
        
    color_sources = [x for x in [options.review_status, options.license, options.rosmake, options.color_file] if x]
    notes_dict = {}
    color_palate = None
    if len(color_sources) != 1:
        color_dict = {}
    if options.color_file:
        color_dict = get_color(options.color_file)
    elif options.review_status:
        print "Coloring by package review status"
        color_dict, notes_dict = get_status_color()
        color_palate = status_map
    elif options.license:
        print "Coloring by package licensing"
        color_dict = get_license_color()
        color_palate = license_map
    elif options.rosmake:
        print "Coloring by rosmakeall results"
        color_dict = get_rosmakeall_color()
        color_palate = rosmakeall_color_map

    if options.output_filename:
        output_filename = os.path.realpath(options.output_filename)
    else:
        output_filename = "deps.pdf"
        
    pkg_dictionary = build_dictionary(targets, exclude, target1=options.target1)

    print "Writing"
    with open('deps.gv', 'w') as outfile:
        outfile.write( """digraph ros {
  //size="11,8";
  size="100,40";
  node [color=gainsboro, style=filled];
  ranksep = 2.0;
""")
        colors = ['red', 'blue', 'green', 'yellow', 'gold', 'orange','chocolate', 'gray44', 'deeppink', 'black']
	classes = []
        if options.cluster:
            classes, base_dict = classify_packages(pkg_dictionary)
            bases = base_dict.keys()
            values = list(set(base_dict.values()))
            i = 1
        # create the legend
        if color_palate:
            outfile.write(' subgraph cluster_legend { style=bold; color=black; label = "Color Legend"; ')
            for type in color_palate:
                text_color="black"
                bg_color = color_palate[type]
                if bg_color == "black":
                    text_color = "white"  
                outfile.write('"%s" [color="%s", fontcolor="%s"];\n '%(type, bg_color, text_color))
            outfile.write('}\n')
        for cl in classes:
            if options.cluster:
                base_color = colors[values.index(base_dict[cl])%len(colors)]
                outfile.write(' subgraph cluster_%d { style=bold; color=%s; label = "%s \\n (%s)"; '%(i,base_color, cl, base_dict[cl]))
                i = i + 1
                for pkg in classes[cl]:
                    outfile.write(' "%s" ;'%pkg)
                outfile.write('}\n')
                
        for pkg, deps in pkg_dictionary.iteritems():
            node_args = []
            ##Coloring
            color = color_dict.get(pkg, 'gainsboro')
            node_args.append("color=%s"%color)
            if color == 'black':
                node_args.append("fontcolor=white")
                
            ## Shape
            if pkg in exclude:
                node_args.append("shape=box")

            # Size
            if options.size_by_deps:
                num_deps = len(roslib.scriptutil.rospack_depends_on(pkg))
                node_args.append("width=%s,height=%s"%(.75 + .1 * sqrt(num_deps), .5 + .1* sqrt(num_deps)))

            ##Perimeter 
            if pkg in targets:
                node_args.append('peripheries=6, style=bold')
            elif pkg in exclude: 
                node_args.append('peripheries=3, style=dashed')

            if len(notes_dict.get(pkg, '')) > 0:
              node_args.append('label="%s\\n(%s)"' % (pkg, notes_dict[pkg]))

            if options.hide:
               if len(roslib.scriptutil.rospack_depends_on(pkg)) == 0 and len(roslib.scriptutil.rospack_depends(pkg)) == 0: #TODO: This is pretty slow
                  print "Hiding unattached package %s"%pkg
                  continue

            outfile.write('  "%s" [%s];\n'%(pkg, ', '.join(node_args)))

            ## Edges
            for dep in deps:
                if not options.verbose and dep in get_child_deps(pkg_dictionary, pkg): 
                    continue
                if dep in pkg_dictionary: #Draw edges to all dependencies
                    outfile.write( '  "%s" -> "%s";\n' % (pkg, dep))
#	outfile.write('}\n')
        ## rank
	if options.rank:	
	    rank_dictionary = build_dictionary([], [])
	    rank = build_rank(rank_dictionary)
            for key in rank:
                outfile.write('{ rank = same;')    
	        for pkg_rank in rank[key]:
                    if pkg_rank in pkg_dictionary:
                        outfile.write(' "%s" ;'%pkg_rank)
                outfile.write('}\n')
        outfile.write( '}\n')
    try:
        # Check version, make postscript if too old to make pdf
        args = ["dot", "-V"]
        vstr = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[1]
        v = distutils.version.StrictVersion('2.16')
        r = re.compile(".*version ([^ ]*).*")
        print vstr
        m = r.search(vstr)
        if not m or not m.group(1):
          print 'Warning: failed to determine your version of dot.  Assuming v2.16'
        else:
          version = distutils.version.StrictVersion(m.group(1))
          print 'Detected dot version %s' % (version)
        if version > distutils.version.StrictVersion('2.8'):
          subprocess.check_call(["dot", "-Tpdf", "deps.gv", "-o", output_filename])
          print "%s generated"%output_filename
        else:
          orig = output_filename
          if output_filename.lower().endswith('.pdf'):
            output_filename = output_filename[:-3]+'ps'
          else:
            print "ERROR", output_filename
          subprocess.check_call(["dot", "-Tps2", "deps.gv", "-o", output_filename])
          print "%s generated"%output_filename
          
          print "Trying to create %s..."%orig
          subprocess.check_call(["ps2pdf", output_filename, orig])
          print "%s generated"%orig
    except subprocess.CalledProcessError:
        print >> sys.stderr, "failed to generate %s"%output_filename

if __name__ == '__main__':
    vdmain()
