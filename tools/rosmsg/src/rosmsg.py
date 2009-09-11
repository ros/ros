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
import os
import platform
import sys
import socket
import threading
import time
import subprocess

import roslib
import roslib.names
import roslib.msgs
import roslib.packages
import roslib.srvs

from optparse import OptionParser

class ROSMsgException(Exception): pass

def succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSMsgException("remote call failed: %s"%msg)
    return val


def rosmsg_users_package_search(mode, type_, package):
    result = set() #using a set for deduplication
    mode_str = "srv"
    if mode == roslib.msgs.EXT:
        mode_str = "msg"

    msg_pkg, msg_name = type_.split('/')

    # Get the full path to the using package
    p = roslib.packages.get_pkg_dir(package)


    # Find the msg/srv files
    # Leave the heavy lifting to find and grep.
    if mode == roslib.msgs.EXT:
        command = []
        if msg_pkg == package:
            command = ["find", p,"-regextype", "posix-egrep",  "-regex", ".*\.(msg|srv)", "!", "-regex", ".*build.*", "!", "-regex", ".*\.svn.*", "!", "-regex", ".*~", "-exec", "grep", "-lE", "(" + type_ + "|" + msg_name + ")", "{}", ";"]
        else:
            command = ["find", p,"-regextype", "posix-egrep",  "-regex", ".*\.(msg|srv)", "!", "-regex", ".*build.*", "!", "-regex", ".*\.svn.*", "!", "-regex", ".*~", "-exec", "grep", "-lE", type_, "{}", ";"]


        #print ' '.join(command)
        msgfiles = subprocess.Popen(command, stdout=subprocess.PIPE).communicate()[0].strip().split()
        #print cppfiles
        # Dump out what we found
        for f in msgfiles:
            if len(f.strip()) != 0:
                result.add( f )
                #print f[len(p)-len(d):]

    # Find the C/C++ files in this package that #include the msg/srv
    # header.  Leave the heavy lifting to find and grep.
    command = ["find", p,"-regextype", "posix-egrep",  "-regex", ".*\.(cpp|h|hh|cc|hpp|c)", "!", "-regex", ".*build.*", "!", "-regex", ".*svn.*", "!", "-regex", ".*~", "-exec", "grep", "-lE", " *#include *(\"|<)" + type_ + ".h", "{}", ";"]
    #print ' '.join(command)
    cppfiles = subprocess.Popen(command, stdout=subprocess.PIPE).communicate()[0].strip().split()
    #print cppfiles
    # Dump out what we found
    for f in cppfiles:
        if len(f.strip()) != 0:
            result.add( f )
            #print f[len(p)-len(d):]

    #print type_list[0], " and ", type_list[1]
    # Capture all of the form import package_name.msg.message_name
    command = ["find", p,"-regextype", "posix-egrep",  "-regex", ".*(\.py|)", "!", "-regex", ".*build.*", "!", "-regex", ".*\.svn.*", "-exec", "grep", "-IlE", "import " +msg_pkg+"\."+mode_str+"\."+ msg_name, "{}", ";"]
    #print ' '.join(command)
    pyfiles =  subprocess.Popen(command, stdout=subprocess.PIPE).communicate()[0].strip().split('\n')
    for f in pyfiles:
        if len(f.strip()) != 0:
            result.add(f)

    # Capture all of the form "from package_name.msg import *|message_name  with message name used
    command = ["find", p,"-regextype", "posix-egrep",  "-regex", ".*(\.py|)", "!", "-regex", ".*build.*", "!", "-regex", ".*\.svn.*", "!", "-regex", ".*~", "-exec", "grep", "-IlE", " *from.*" + msg_pkg + "\."+mode_str+".*import (\*|" + msg_name +")", "{}",";"]
    #print ' '.join(command)
    pyfiles =  subprocess.Popen(command, stdout=subprocess.PIPE).communicate()[0].strip().split('\n')


    for f in pyfiles:
        if len(f.strip()) != 0:
            # Make sure that this message is used for the above test could return without using the message if importing *
            #print "looking for", msg, "in", f
            command = ["grep", "-l", msg_name, f]
            #print ' '.join(command)
            present  =  subprocess.Popen(command, stdout=subprocess.PIPE).communicate()[0].strip().split('\n')
            if len(present[0]) > 0:
                #print present, len(present[0])
                result.add(f)
            
    return list(result) #return as list


def rosmsg_users(mode, type_):
    print 'Files using %s:' % (type_)
    package, base_type = roslib.names.package_resource_name(type_)
    # Find the direct users of the package; they're the only ones who
    # should be able to use this message
    deps = subprocess.Popen(["rospack", "depends-on1", package], stdout=subprocess.PIPE).communicate()[0].strip().split()
    # Check the messages own package too
    msg_pkg, msg_name = type_.split('/')
    deps.append(msg_pkg)

    correct_dependencies = []
    for d in deps:
        files = rosmsg_users_package_search(mode, type_, d)
        correct_dependencies.extend(files)

    print "Useages directly depended upon:" + type_
    for f in correct_dependencies:
        print f

    deps = subprocess.Popen(["rospack", "depends-on", package], stdout=subprocess.PIPE).communicate()[0].strip().split()
    incorrect_dependencies = []
    for d in deps:
        files = rosmsg_users_package_search(mode, type_, d)
        incorrect_dependencies.extend(files)
    print "Usages indirectly depended upon:"
    for f in incorrect_dependencies:
        if f not in correct_dependencies:
            print f

## Get .srv file for \a type_ as text
## @param type_ str: service type
## @param raw bool: if True, include comments and whitespace (default False)
## @return str: text of .srv file
def get_srv_text(type_, raw=False):
    package, base_type = roslib.names.package_resource_name(type_)
    roslib.msgs.load_package_dependencies(package)
    roslib.msgs.load_package(package)
    f = roslib.srvs.srv_file(package, base_type)
    name, spec = roslib.srvs.load_from_file(f)
    if raw:
        return spec.text
    else:
        return str(spec.request)+'---\n'+str(spec.response)

## Get .msg file for \a type_ as text
## @param type_ str: message type
## @param raw bool: if True, include comments and whitespace (default False)
## @return str: text of .msg file
def get_msg_text(type_, raw=False):
    package, base_type = roslib.names.package_resource_name(type_)
    roslib.msgs.load_package_dependencies(package)
    roslib.msgs.load_package(package)
    spec = roslib.msgs.get_registered(type_)
    if raw:
        return spec.text
    else:
        return str(spec)
    
## Prints contents of msg/srv file
## @param mode str: roslib.srvs.EXT or roslib.msgs.EXT
def rosmsg_debug(mode, type_, raw=False):
    if mode == roslib.srvs.EXT:
        print get_srv_text(type_, raw=raw)
    elif mode == roslib.msgs.EXT:
        print get_msg_text(type_, raw=raw)
    else:
        raise ROSMsgException("invalid mode: %s"%mode)
    
## Prints msg/srvs contained in \a package
## @param mode str: roslib.srvs.EXT or roslib.msgs.EXT
## @param package str: package name
def rosmsg_pkg_debug(mode, package):
    if mode == roslib.msgs.EXT:
        print "Messages in [%s]:\n"%package
        print '\n'.join([roslib.names.resource_name(package, t) for t in roslib.msgs.list_msg_types(package, False)])
    elif mode == roslib.srvs.EXT:
        print "Services in [%s]:\n"%package
        print '\n'.join([roslib.names.resource_name(package, t) for t in roslib.srvs.list_srv_types(package, False)])

## iterator for packages that contain messages/services
## @param mode str: roslib.msgs.EXT or roslib.srvs.EXT
def iterate_packages(mode):
    if mode == roslib.msgs.EXT:
        subdir = roslib.packages.MSG_DIR
    else:
        subdir = roslib.packages.SRV_DIR

    pkgs = roslib.packages.list_pkgs()
    for p in pkgs:
        dir = roslib.packages.get_pkg_subdir(p, subdir, False)
        if dir and os.path.isdir(dir):
            yield p
    
## print all packages that contain messages/services
## @param mode str: roslib.msgs.EXT or roslib.srvs.EXT
def rosmsg_all(mode):
    if mode == roslib.msgs.EXT:
        print "Packages with messages:\n"        
    else:
        print "Packages with services:\n"                
    for p in iterate_packages(mode):
        print p
    print ''

## iterator for all packages that contain a message matching \a base_type
## @param base_type str: message base type to match, e.g. 'String' would match std_msgs/String
def rosmsg_search(mode, base_type):
    if mode == roslib.msgs.EXT:
        res_file = roslib.msgs.msg_file
    else:
        res_file = roslib.srvs.srv_file
    for p in iterate_packages(mode):
        if os.path.isfile(res_file(p, base_type)):
            yield roslib.names.resource_name(p, base_type)

def _stdin_arg(parser, full):
    options, args = parser.parse_args(sys.argv[2:])    
    # read in args from stdin pipe if not present
    if not args:
        arg = None
        while not arg:
            arg = sys.stdin.readline().strip()
        return options, arg
    else:
        if len(args) > 1:
            parser.error("you may only specify one input package or %s"%full)
        return options, args[0]
    
def rosmsg_cmd_show(mode, full):
    parser = OptionParser(usage="usage: ros%s show [options] <%s>"%(mode[1:], full))
    parser.add_option("-r", "--raw",
                      dest="raw", default=False,action="store_true",
                      help="show raw message text, including comments")
    options, arg = _stdin_arg(parser, full)

    if '/' in arg: #package specified
        rosmsg_debug(mode, arg, options.raw)
    else:
        for found in rosmsg_search(mode, arg):
            print "[%s]:"%found
            rosmsg_debug(mode, found, options.raw)

def rosmsg_cmd_users(mode, full):
    parser = OptionParser(usage="usage: ros%s users <%s>"%(mode[1:], full))
    options, arg = _stdin_arg(parser, full)
    rosmsg_users(mode, arg)
    
def rosmsg_cmd_package(mode, full):
    parser = OptionParser(usage="usage: ros%s package <package>"%mode[1:])
    options, arg = _stdin_arg(parser, full)
    rosmsg_pkg_debug(mode, arg)    
    
def rosmsg_cmd_packages(mode, full):
    parser = OptionParser(usage="usage: ros%s packages"%mode[1:])
    options, args = parser.parse_args(sys.argv[2:])
    rosmsg_all(mode)    

def fullusage(cmd):
    print """Commands:
\t%(cmd)s show\tShow message description
\t%(cmd)s users\tFind files that use message
\t%(cmd)s package\tList messages in a package
\t%(cmd)s packages\tList packages that contain messages

Type %(cmd)s <command> -h for more detailed usage
"""%locals()
    sys.exit(os.EX_USAGE)
    
## rosmsg can interact with either ros messages or ros services. the \a mode
## param indicates which
## @param mode str: roslib.msgs.EXT or roslib.srvs.EXT
def rosmsgmain(mode=roslib.msgs.EXT):
    if len(sys.argv) == 1:
        fullusage('ros'+mode[1:])
    if mode == roslib.msgs.EXT:
        ext, full = mode, "message type"
    else:
        ext, full = mode, "service type"
        
    try:
        command = sys.argv[1]
        if command == 'users':
            rosmsg_cmd_users(ext, full)
        elif command == 'show':
            rosmsg_cmd_show(ext, full)
        elif command == 'package':
            rosmsg_cmd_package(ext, full)
        elif command == 'packages':
            rosmsg_cmd_packages(ext, full)
        else:
            fullusage('ros'+mode[1:])
    except KeyError, e:
        print >> sys.stderr, "Invalid message: %s"%e
    except roslib.packages.InvalidROSPkgException, e:
        print >> sys.stderr, "Invalid package: '%s'"%e
    except ROSMsgException, e:
        print >> sys.stderr, str(e)
    except KeyboardInterrupt:
        pass
