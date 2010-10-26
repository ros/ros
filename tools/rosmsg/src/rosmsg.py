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

"""
Implements rosmsg/rossrv command-line tools.

The code API of the rosmsg module is unstable. Much of the
functionality of rosmsg/rossrv is implemented using the roslib.msgs
and roslib.srvs libraries and can be found there instead.
"""

import os
import platform
import sys
import socket
import threading
import time
import subprocess

import roslib
import roslib.genpy
import roslib.gentools
import roslib.message
import roslib.msgs
import roslib.names
import roslib.packages
import roslib.srvs
import rosbag

from optparse import OptionParser

class ROSMsgException(Exception): pass

import warnings
def deprecated(func):
    """This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emmitted
    when the function is used."""
    def newFunc(*args, **kwargs):
        warnings.warn("Call to deprecated function %s." % func.__name__,
                      category=DeprecationWarning, stacklevel=2)
        return func(*args, **kwargs)
    newFunc.__name__ = func.__name__
    newFunc.__doc__ = func.__doc__
    newFunc.__dict__.update(func.__dict__)
    return newFunc

def succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSMsgException("remote call failed: %s"%msg)
    return val

def make_find_command(path):
  if os.uname()[0] == 'Darwin':
    return ["find", "-E", path]
  else:
    return ["find", path, "-regextype", "posix-egrep"]


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
            command = make_find_command(p)
            command += ["-regex", ".*\.(msg|srv)", "!", "-regex", ".*build.*", "!", "-regex", ".*\.svn.*", "!", "-regex", ".*~", "-exec", "grep", "-lE", "(" + type_ + "|" + msg_name + ")", "{}", ";"]
        else:
            command = make_find_command(p)
            command += ["-regex", ".*\.(msg|srv)", "!", "-regex", ".*build.*", "!", "-regex", ".*\.svn.*", "!", "-regex", ".*~", "-exec", "grep", "-lE", type_, "{}", ";"]


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
    command = make_find_command(p)
    command += ["-regex", ".*\.(cpp|h|hh|cc|hpp|c)", "!", "-regex", ".*build.*", "!", "-regex", ".*svn.*", "!", "-regex", ".*~", "-exec", "grep", "-lE", " *#include *(\"|<)" + type_ + ".h", "{}", ";"]
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
    command = make_find_command(p)
    # The following regex doesn't work on OS X, and also doesn't appear to
    # do anything.
    #command += ["-regex", ".*(\.py|)"]
    command += ["!", "-regex", ".*build.*", "!", "-regex", ".*\.svn.*", "-exec", "grep", "-IlE", "import " +msg_pkg+"\."+mode_str+"\."+ msg_name, "{}", ";"]
    #print ' '.join(command)
    pyfiles =  subprocess.Popen(command, stdout=subprocess.PIPE).communicate()[0].strip().split('\n')
    for f in pyfiles:
        if len(f.strip()) != 0:
            result.add(f)

    # Capture all of the form "from package_name.msg import *|message_name  with message name used
    command = make_find_command(p)
    # The following regex doesn't work on OS X, and also doesn't appear to
    # do anything.
    #command += ["-regex", ".*(\.py|)"]
    command += ["!", "-regex", ".*build.*", "!", "-regex", ".*\.svn.*", "!", "-regex", ".*~", "-exec", "grep", "-IlE", " *from.*" + msg_pkg + "\."+mode_str+".*import (\*|" + msg_name +")", "{}",";"]
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
    msg_pkg, msg_name = roslib.names.package_resource_name(type_)
    # Find the direct users of the package; they're the only ones who
    # should be able to use this message
    if not msg_pkg:
        print >> sys.stderr, "Please specify the package and type for [%s]"%type_
        sys.exit(os.EX_USAGE)

    print 'Files using %s:' % (type_)
        
    deps = subprocess.Popen(["rospack", "depends-on1", msg_pkg], stdout=subprocess.PIPE).communicate()[0].strip().split()
    # Check the messages own package too
    deps.append(msg_pkg)

    correct_dependencies = []
    for d in deps:
        files = rosmsg_users_package_search(mode, type_, d)
        correct_dependencies.extend(files)

    print "Usages directly depended upon:" + type_
    for f in correct_dependencies:
        print f

    deps = subprocess.Popen(["rospack", "depends-on", msg_pkg], stdout=subprocess.PIPE).communicate()[0].strip().split()
    incorrect_dependencies = []
    for d in deps:
        files = rosmsg_users_package_search(mode, type_, d)
        incorrect_dependencies.extend(files)
    print "Usages indirectly depended upon:"
    for f in incorrect_dependencies:
        if f not in correct_dependencies:
            print f

def get_srv_text(type_, raw=False):
    """
    Get .srv file for type_ as text
    @param type_: service type
    @type  type_: str
    @param raw: if True, include comments and whitespace (default False)
    @type  raw: bool
    @return: text of .srv file
    @rtype: str
    @raise ROSMsgException: if type_ is unknown
    """
    package, base_type = roslib.names.package_resource_name(type_)
    roslib.msgs.load_package_dependencies(package, load_recursive=True)
    roslib.msgs.load_package(package)
    f = roslib.srvs.srv_file(package, base_type)
    if not os.path.isfile(f):
        raise ROSMsgException("Unknown srv type: %s"%type_)
    name, spec = roslib.srvs.load_from_file(f, package)
    if raw:
        return spec.text
    else:
        return str(spec.request)+'---\n'+str(spec.response)

def get_msg_text(type_, raw=False, full_text=None):
    """
    Get .msg file for type_ as text
    @param type_: message type
    @type  type_: str
    @param raw: if True, include comments and whitespace (default False)
    @type  raw: bool
    @param full_text: if not None, contains full text of message definition
    @type  full_text: str
    @return: text of .msg file
    @rtype: str
    @raise ROSMsgException: if type_ is unknown
    """
    package, base_type = roslib.names.package_resource_name(type_)
    
    if not full_text:
        roslib.msgs.load_package_dependencies(package, load_recursive=True)
        roslib.msgs.load_package(package)
        try:
            spec = roslib.msgs.get_registered(type_)
        except KeyError:
            raise ROSMsgException("Unknown msg type: %s"%type_)        

        if raw:
            text = spec.text
        else:
            text = str(spec)
    else:
        splits = full_text.split('\n'+'='*80+'\n')
        core_msg = splits[0]
        deps_msgs = splits[1:]

        specs = { type_: roslib.msgs.load_from_string(core_msg, package) }
        for dep_msg in deps_msgs:
            dep_type, dep_spec = roslib.genpy._generate_dynamic_specs(specs, dep_msg)
            specs[dep_type] = dep_spec
        
        for t, spec in specs.iteritems():
            roslib.msgs.register(t, spec)       
        spec = specs[type_]
        if raw:
            text = spec.text
        else:
            text = str(spec)

    return text

def rosmsg_debug(mode, type_, raw=False):
    """
    Prints contents of msg/srv file
    @param mode: roslib.srvs.EXT or roslib.msgs.EXT
    @type  mode: str
    """
    if mode == roslib.srvs.EXT:
        print get_srv_text(type_, raw=raw)
    elif mode == roslib.msgs.EXT:
        print get_msg_text(type_, raw=raw)
    else:
        raise ROSMsgException("invalid mode: %s"%mode)
    
def list_srvs(package):
    """
    List srvs contained in package
    @param package: package name
    @type  package: str
    @return: list of srvs in package
    @rtype: [str]
    """
    return list_types(package, mode=roslib.srvs.EXT)

def list_msgs(package):
    """
    List msgs contained in package
    @param package: package name
    @type  package: str
    @return: list of msgs in package
    @rtype: [str]
    """
    return list_types(package)
    
# DEPRECATED
@deprecated
def rosmsg_list_package(mode, package):
    return list_types(package, mode=mode)

def list_types(package, mode=roslib.msgs.EXT):
    """
    Lists msg/srvs contained in package
    @param package: package name
    @type  package: str
    @param mode: roslib.srvs.EXT or roslib.msgs.EXT. Defaults to msgs.
    @type  mode: str
    @return: list of msgs/srv in package
    @rtype: [str]
    """
    if mode == roslib.msgs.EXT:
        return [roslib.names.resource_name(package, t) for t in roslib.msgs.list_msg_types(package, False)]
    elif mode == roslib.srvs.EXT:
        return [roslib.names.resource_name(package, t) for t in roslib.srvs.list_srv_types(package, False)]
    else:
        raise ValueError('mode')

def iterate_packages(mode):
    """
    Iterator for packages that contain messages/services
    @param mode: roslib.msgs.EXT or roslib.srvs.EXT
    @type  mode: str
    """
    if mode == roslib.msgs.EXT:
        subdir = roslib.packages.MSG_DIR
    elif mode == roslib.srvs.EXT:
        subdir = roslib.packages.SRV_DIR
    else:
        raise ValueError('mode')

    pkgs = roslib.packages.list_pkgs()
    for p in pkgs:
        dir = roslib.packages.get_pkg_subdir(p, subdir, False)
        if dir and os.path.isdir(dir):
            yield p
    
@deprecated
def rosmsg_list_packages(mode):
    """
    Use list_packages
    """
    return list_packages(mode=mode)

def list_packages(mode=roslib.msgs.EXT):
    """
    List all packages that contain messages/services. This is a convenience
    function of iterate_packages
    @param mode: roslib.msgs.EXT or roslib.srvs.EXT. Defaults to msgs
    @type  mode: str
    @return: list of packages that contain messages/services (depending on mode)
    @rtype: [str]
    """
    return [p for p in iterate_packages(mode)]

## iterator for all packages that contain a message matching base_type
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
            parser.error("you may only specify one %s"%full)
        return options, args[0]
    
def rosmsg_cmd_show(mode, full):
    cmd = "ros%s"%(mode[1:])
    parser = OptionParser(usage="usage: %s show [options] <%s>"%(cmd, full))
    parser.add_option("-r", "--raw",
                      dest="raw", default=False,action="store_true",
                      help="show raw message text, including comments")
    parser.add_option("-b", "--bag",
                      dest="bag", default=None,
                      help="show message from .bag file", metavar="BAGFILE")
    options, arg = _stdin_arg(parser, full)
    if arg.endswith(mode):
        arg = arg[:-(len(mode))]

    # try to catch the user specifying code-style types and error
    if '::' in arg:
        parser.error(cmd+" does not understand C++-style namespaces (i.e. '::').\nPlease refer to msg/srv types as 'package_name/Type'.")
    elif '.' in arg:
        parser.error("invalid message type '%s'.\nPlease refer to msg/srv types as 'package_name/Type'." % arg)
    if options.bag:
        bag_file = options.bag
        if not os.path.exists(bag_file):
            raise ROSMsgException("ERROR: bag file [%s] does not exist"%bag_file)
        for topic, msg, t in rosbag.Bag(bag_file).read_messages(raw=True):
            datatype, _, _, _, pytype = msg
            if datatype == arg:
                print get_msg_text(datatype, options.raw, pytype._full_text)
                break
    else:
        if '/' in arg: #package specified
            rosmsg_debug(mode, arg, options.raw)
        else:
            for found in rosmsg_search(mode, arg):
                print "[%s]:"%found
                rosmsg_debug(mode, found, options.raw)

def rosmsg_md5(mode, type_):
    package, base_type = roslib.names.package_resource_name(type_)
    roslib.msgs.load_package_dependencies(package, load_recursive=True)
    roslib.msgs.load_package(package)
    if mode == roslib.msgs.EXT:
        f = roslib.msgs.msg_file(package, base_type)
        name, spec = roslib.msgs.load_from_file(f, package)
    else:
        f = roslib.srvs.srv_file(package, base_type)
        name, spec = roslib.srvs.load_from_file(f, package)
    gendeps_dict = roslib.gentools.get_dependencies(spec, package, compute_files=False)
    return roslib.gentools.compute_md5(gendeps_dict)

## soft-fail routine to check against generated md5sum to provide a sanity check. Only checks the Python md5
def _rosmsg_md5_check(mode, type_, gendeps_md5):
    try:
        msg_class = roslib.message.get_message_class(type_)
        if msg_class is not None and msg_class._md5sum != gendeps_md5:
            print >> sys.stderr, "WARN: md5sum for compiled [%s] appears to differ from %s:\n\t%s vs %s"%(type_, mode, msg_class._md5sum, gendeps_md5)
    except ImportError: pass
    
def rosmsg_cmd_md5(mode, full):
    parser = OptionParser(usage="usage: ros%s md5 <%s>"%(mode[1:], full))
    options, arg = _stdin_arg(parser, full)

    if '/' in arg: #package specified
        try:
            md5 = rosmsg_md5(mode, arg)
            print md5
            _rosmsg_md5_check(mode, arg, md5)
        except IOError:
            print >> sys.stderr, "Cannot locate [%s]"%arg
    else:
        matches = [m for m in rosmsg_search(mode, arg)]
        for found in matches:
            try:
                md5 = rosmsg_md5(mode, found)
                print "[%s]: %s"%(found, md5)
                _rosmsg_md5_check(mode, found, md5)
            except IOError:
                print >> sys.stderr, "Cannot locate [%s]"%found
        if not matches:
            print >> sys.stderr, "No messages matching the name [%s]"%arg
                
def rosmsg_cmd_users(mode, full):
    parser = OptionParser(usage="usage: ros%s users <%s>"%(mode[1:], full))
    options, arg = _stdin_arg(parser, full)
    rosmsg_users(mode, arg)
    
def rosmsg_cmd_package(mode, full):
    parser = OptionParser(usage="usage: ros%s package <package>"%mode[1:])
    parser.add_option("-s",
                      dest="single_line", default=False,action="store_true",
                      help="list all msgs on a single line")
    options, arg = _stdin_arg(parser, full)
    if options.single_line:    
        print ' '.join(list_types(arg,mode=mode))        
    else:
        print '\n'.join(list_types(arg, mode=mode))
    
def rosmsg_cmd_packages(mode, full):
    parser = OptionParser(usage="usage: ros%s packages"%mode[1:])
    parser.add_option("-s",
                      dest="single_line", default=False,action="store_true",
                      help="list all packages on a single line")
    options, args = parser.parse_args(sys.argv[2:])
    if options.single_line:
        print ' '.join([p for p in iterate_packages(mode)])
    else:
        print '\n'.join([p for p in iterate_packages(mode)])

def fullusage(cmd):
    """
    @param cmd: command name
    @type  cmd: str
    @return: usage text for cmd
    @rtype: str
    """
    return """Commands:
\t%(cmd)s show\tShow message description
\t%(cmd)s users\tFind files that use message
\t%(cmd)s md5\tDisplay message md5sum
\t%(cmd)s package\tList messages in a package
\t%(cmd)s packages\tList packages that contain messages

Type %(cmd)s <command> -h for more detailed usage
"""%locals()
    
def rosmsgmain(mode=roslib.msgs.EXT):
    """
    Main entry point for command-line tools (rosmsg/rossrv).
    
    rosmsg can interact with either ros messages or ros services. The mode
    param indicates which
    @param mode: roslib.msgs.EXT or roslib.srvs.EXT
    @type  mode: str
    """
    if len(sys.argv) == 1:
        print fullusage('ros'+mode[1:])
        sys.exit(0)        
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
        elif command == 'md5':
            rosmsg_cmd_md5(ext, full)
        else:
            print fullusage('ros'+mode[1:])
            sys.exit(0)
    except KeyError, e:
        print >> sys.stderr, "Unknown message type: %s"%e
        sys.exit(os.EX_USAGE)
    except roslib.packages.InvalidROSPkgException, e:
        print >> sys.stderr, "Invalid package: '%s'"%e
        sys.exit(os.EX_USAGE)        
    except ValueError, e:
        print >> sys.stderr, "Invalid type: '%s'"%e
        sys.exit(os.EX_USAGE)          
    except ROSMsgException, e:
        print >> sys.stderr, str(e)
        sys.exit(1)        
    except KeyboardInterrupt:
        pass
