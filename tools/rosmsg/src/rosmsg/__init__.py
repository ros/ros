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

from __future__ import print_function

import os
import platform
import sys
import subprocess

import rospkg
import genmsg

import roslib

import roslib.gentools
import roslib.message
import roslib.msgs
import roslib.srvs
import rosbag

from optparse import OptionParser

MODE_MSG = '.msg'
MODE_SRV = '.srv'

class ROSMsgException(Exception): pass

def succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSMsgException("remote call failed: %s"%msg)
    return val

def make_find_command(path):
    if os.uname()[0] in ['Darwin', 'FreeBSD']:
        return ["find", "-E", path]
    else:
        return ["find", path, "-regextype", "posix-egrep"]

def rosmsg_users_package_search(mode, type_, package):
    result = set() #using a set for deduplication
    mode_str = "srv"
    if mode == MODE_MSG:
        mode_str = "msg"

    msg_pkg, msg_name = type_.split('/')

    # Get the full path to the using package
    rospack = rospkg.RosPack()
    p = rospack.get_path(package)

    # Find the msg/srv files
    # Leave the heavy lifting to find and grep.
    if mode == MODE_MSG:
        command = []
        if msg_pkg == package:
            command = make_find_command(p)
            command += ["-regex", ".*\.(msg|srv)", "!", "-regex", ".*build.*", "!", "-regex", ".*\.svn.*", "!", "-regex", ".*~", "-exec", "grep", "-lE", "(" + type_ + "|" + msg_name + ")", "{}", ";"]
        else:
            command = make_find_command(p)
            command += ["-regex", ".*\.(msg|srv)", "!", "-regex", ".*build.*", "!", "-regex", ".*\.svn.*", "!", "-regex", ".*~", "-exec", "grep", "-lE", type_, "{}", ";"]


        msgfiles = subprocess.Popen(command, stdout=subprocess.PIPE).communicate()[0].strip().split()
        # Dump out what we found
        for f in msgfiles:
            if len(f.strip()) != 0:
                result.add( f )

    # Find the C/C++ files in this package that #include the msg/srv
    # header.  Leave the heavy lifting to find and grep.
    command = make_find_command(p)
    command += ["-regex", ".*\.(cpp|h|hh|cc|hpp|c)", "!", "-regex", ".*build.*", "!", "-regex", ".*svn.*", "!", "-regex", ".*~", "-exec", "grep", "-lE", " *#include *(\"|<)" + type_ + ".h", "{}", ";"]
    cppfiles = subprocess.Popen(command, stdout=subprocess.PIPE).communicate()[0].strip().split()
    # Dump out what we found
    for f in cppfiles:
        if len(f.strip()) != 0:
            result.add( f )

    # Capture all of the form import package_name.msg.message_name
    command = make_find_command(p)
    # The following regex doesn't work on OS X, and also doesn't appear to
    # do anything.
    #command += ["-regex", ".*(\.py|)"]
    command += ["!", "-regex", ".*build.*", "!", "-regex", ".*\.svn.*", "-exec", "grep", "-IlE", "import " +msg_pkg+"\."+mode_str+"\."+ msg_name, "{}", ";"]
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
    pyfiles =  subprocess.Popen(command, stdout=subprocess.PIPE).communicate()[0].strip().split('\n')


    for f in pyfiles:
        if len(f.strip()) != 0:
            # Make sure that this message is used for the above test could return without using the message if importing *
            command = ["grep", "-l", msg_name, f]
            present  =  subprocess.Popen(command, stdout=subprocess.PIPE).communicate()[0].strip().split('\n')
            if len(present[0]) > 0:
                result.add(f)
            
    return list(result) #return as list


def rosmsg_users(mode, type_):
    msg_pkg, msg_name = genmsg.package_resource_name(type_)
    # Find the direct users of the package; they're the only ones who
    # should be able to use this message
    if not msg_pkg:
        print("Please specify the package and type for [%s]"%type_, file=sys.stderr)
        sys.exit(os.EX_USAGE)

    print('Files using %s:' % (type_))
        
    deps = subprocess.Popen(["rospack", "depends-on1", msg_pkg], stdout=subprocess.PIPE).communicate()[0].strip().split()
    # Check the messages own package too
    deps.append(msg_pkg)

    correct_dependencies = []
    for d in deps:
        files = rosmsg_users_package_search(mode, type_, d)
        correct_dependencies.extend(files)

    print("Usages directly depended upon:" + type_)
    for f in correct_dependencies:
        print(f)

    deps = subprocess.Popen(["rospack", "depends-on", msg_pkg], stdout=subprocess.PIPE).communicate()[0].strip().split()
    incorrect_dependencies = []
    for d in deps:
        files = rosmsg_users_package_search(mode, type_, d)
        incorrect_dependencies.extend(files)
    print("Usages indirectly depended upon:")
    for f in incorrect_dependencies:
        if f not in correct_dependencies:
            print(f)

from cStringIO import StringIO
def spec_to_str(msg_context, spec, buff=None, indent=''):
    """
    Convert spec into a string representation. Helper routine for MsgSpec.
    :param indent: internal use only, ``str``
    :param buff: internal use only, ``StringIO``
    :returns: string representation of spec, ``str``
    """
    if buff is None:
        buff = StringIO()
    for c in spec.constants:
        buff.write("%s%s %s=%s\n"%(indent, c.type, c.name, c.val_text))
    for type_, name in zip(spec.types, spec.names):
        buff.write("%s%s %s\n"%(indent, type_, name))
        base_type = genmsg.msgs.bare_msg_type(type_)
        if not base_type in genmsg.msgs.BUILTIN_TYPES:
            subspec = msg_context.get_registered(base_type)
            spec_to_str(msg_context, subspec, buff, indent + '  ')
    return buff.getvalue()

def get_srv_text(type_, raw=False):
    """
    Get .srv file for type_ as text
    :param type_: service type, ``str``
    :param raw: if True, include comments and whitespace (default False), ``bool``
    :returns: text of .srv file, ``str``
    @raise ROSMsgException: if type_ is unknown
    """
    rospack = rospkg.RosPack()
    srv_search_path = {}
    msg_search_path = {}
    for p in rospack.list():
        path = rospack.get_path(p)
        msg_search_path[p] = os.path.join(path, 'msg')
        srv_search_path[p] = os.path.join(path, 'srv')
        
    #TODO: cache context somewhere
    context = genmsg.MsgContext.create_default()
    try:
        spec = genmsg.load_srv_by_type(context, type_, srv_search_path)
        genmsg.load_depends(context, spec, msg_search_path)
    except Exception as e:
        raise ROSMsgException("Unknown srv type [%s]: %s"%(type_, e))
    
    if raw:
        return spec.text
    else:
        return spec_to_str(context, spec.request)+'---\n'+spec_to_str(context, spec.response)

def get_msg_text(type_, raw=False):
    """
    Get .msg file for type_ as text
    :param type_: message type, ``str``
    :param raw: if True, include comments and whitespace (default False), ``bool``
    :returns: text of .msg file, ``str``
    :raises :exc:`ROSMsgException` If type_ is unknown
    """
    rospack = rospkg.RosPack()
    search_path = {}
    for p in rospack.list():
        search_path[p] = os.path.join(rospack.get_path(p), 'msg')

    context = genmsg.MsgContext.create_default()
    try:
        spec = genmsg.load_msg_by_type(context, type_, search_path)
        genmsg.load_depends(context, spec, search_path)
    except Exception as e:
        raise ROSMsgException("Unable to load msg [%s]: %s"%(type_, e))
    
    if raw:
        return spec.text
    else:
        return spec_to_str(context, spec)

def rosmsg_debug(mode, type_, raw=False):
    """
    Prints contents of msg/srv file
    :param mode: MODE_MSG or MODE_SRV, ``str``
    """
    if mode == MODE_SRV:
        print(get_srv_text(type_, raw=raw))
    elif mode == MODE_MSG:
        print(get_msg_text(type_, raw=raw))
    else:
        raise ROSMsgException("Invalid mode for debug: %s"%mode)
    
def list_srvs(package):
    """
    List srvs contained in package
    :param package: package name, ``str``
    :returns: list of srvs in package, ``[str]``
    """
    return list_types(package, mode=MODE_SRV)

def list_msgs(package):
    """
    List msgs contained in package
    :param package: package name, ``str``
    :returns: list of msgs in package, ``[str]``
    """
    return list_types(package)
    
def list_types(package, mode=MODE_MSG):
    """
    Lists msg/srvs contained in package
    :param package: package name, ``str``
    :param mode: MODE_MSG or MODE_SRV. Defaults to msgs, ``str``
    :returns: list of msgs/srv in package, ``[str]``
    """
    rospack = rospkg.RosPack()
    if mode == MODE_MSG:
        return [genmsg.resource_name(package, t) for t in _list_types(rospack, package, 'msg', '.msg')]
    elif mode == MODE_SRV:
        return [genmsg.resource_name(package, t) for t in _list_types(rospack, package, 'srv', '.srv')]
    else:
        raise ValueError('Unknown mode for list_types: %s'%mode)

def _msg_filter(ext):
    def mfilter(f):
        """
        Predicate for filtering directory list. matches message files
        :param f: filename, ``str``
        """
        return os.path.isfile(f) and f.endswith(ext)
    return mfilter

def _list_types(rospack, package, subdir, ext):
    """
    List all messages in the specified package
    :param package str: name of package to search
    :param include_depends bool: if True, will also list messages in package dependencies
    :returns [str]: message type names
    """
    path = os.path.join(rospack.get_path(package), subdir)
    types = _list_resources(package, path, _msg_filter(ext))
    return [x[:-len(ext)] for x in types]

def _list_resources(package, path, rfilter=os.path.isfile):
    """
    List resources in a package directory within a particular
    subdirectory. This is useful for listing messages, services, etc...
    :param rfilter: resource filter function that returns true if filename is the desired resource type, ``fn(filename)->bool``
    """
    resources = []
    if os.path.isdir(path):
        resources = [f for f in os.listdir(path) if rfilter(os.path.join(path, f))]
    else:
        resources = []
    return resources

def iterate_packages(mode):
    """
    Iterator for packages that contain messages/services
    :param mode: .msg or .srv, ``str``
    """
    if mode == MODE_MSG:
        subdir = 'msg'
    elif mode == MODE_SRV:
        subdir = 'srv'
    else:
        raise ValueError('Unknown mode for iterate_packages: %s'%mode)

    rospack = rospkg.RosPack()
    pkgs = rospack.list()
    for p in pkgs:
        try:
            d = os.path.join(rospack.get_path(p), subdir)
            if os.path.isdir(d):
                yield p
        except rospkg.ResourceNotFound:
            # race condition, ignore
            pass
    
def list_packages(mode=MODE_MSG):
    """
    List all packages that contain messages/services. This is a convenience
    function of iterate_packages
    :param mode: MODE_MSG or MODE_SRV. Defaults to msgs, ``str``
    :returns: list of packages that contain messages/services (depending on mode), ``[str]``
    """
    return [p for p in iterate_packages(mode)]

def rosmsg_search(mode, base_type):
    """
    Iterator for all packages that contain a message matching base_type

    :param base_type: message base type to match, e.g. 'String' would match std_msgs/String, ``str``
    """
    if mode == MODE_MSG:
        res_file = roslib.msgs.msg_file
    else:
        res_file = roslib.srvs.srv_file
    for p in iterate_packages(mode):
        if os.path.isfile(res_file(p, base_type)):
            yield genmsg.resource_name(p, base_type)

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
                print(get_msg_text(datatype, options.raw, pytype._full_text))
                break
    else:
        if '/' in arg: #package specified
            rosmsg_debug(mode, arg, options.raw)
        else:
            for found in rosmsg_search(mode, arg):
                print("[%s]:"%found)
                rosmsg_debug(mode, found, options.raw)

def rosmsg_md5(mode, type_):
    package, base_type = genmsg.package_resource_name(type_)
    roslib.msgs.load_package_dependencies(package, load_recursive=True)
    roslib.msgs.load_package(package)
    if mode == MODE_MSG:
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
            print("WARN: md5sum for compiled [%s] appears to differ from %s:\n\t%s vs %s"%(type_, mode, msg_class._md5sum, gendeps_md5), file=sys.stderr)
    except ImportError: pass
    
def rosmsg_cmd_md5(mode, full):
    parser = OptionParser(usage="usage: ros%s md5 <%s>"%(mode[1:], full))
    options, arg = _stdin_arg(parser, full)

    if '/' in arg: #package specified
        try:
            md5 = rosmsg_md5(mode, arg)
            print(md5)
            _rosmsg_md5_check(mode, arg, md5)
        except IOError:
            print("Cannot locate [%s]"%arg, file=sys.stderr)
    else:
        matches = [m for m in rosmsg_search(mode, arg)]
        for found in matches:
            try:
                md5 = rosmsg_md5(mode, found)
                print("[%s]: %s"%(found, md5))
                _rosmsg_md5_check(mode, found, md5)
            except IOError:
                print("Cannot locate [%s]"%found, file=sys.stderr)
        if not matches:
            print("No messages matching the name [%s]"%arg, file=sys.stderr)
                
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
        print(' '.join(list_types(arg,mode=mode)))
    else:
        print('\n'.join(list_types(arg, mode=mode)))
    
def rosmsg_cmd_packages(mode, full):
    parser = OptionParser(usage="usage: ros%s packages"%mode[1:])
    parser.add_option("-s",
                      dest="single_line", default=False,action="store_true",
                      help="list all packages on a single line")
    options, args = parser.parse_args(sys.argv[2:])
    if options.single_line:
        print(' '.join([p for p in iterate_packages(mode)]))
    else:
        print('\n'.join([p for p in iterate_packages(mode)]))

def fullusage(cmd):
    """
    :param cmd: command name, ``str``
    :returns: usage text for cmd, ``str``
    """
    return """Commands:
\t%(cmd)s show\tShow message description
\t%(cmd)s users\tFind files that use message
\t%(cmd)s md5\tDisplay message md5sum
\t%(cmd)s package\tList messages in a package
\t%(cmd)s packages\tList packages that contain messages

Type %(cmd)s <command> -h for more detailed usage
"""%locals()
    
def rosmsgmain(mode=MODE_MSG):
    """
    Main entry point for command-line tools (rosmsg/rossrv).
    
    rosmsg can interact with either ros messages or ros services. The mode
    param indicates which
    :param mode: MODE_MSG or MODE_SRV, ``str``
    """
    try:
        if mode == MODE_MSG:
            ext, full = mode, "message type"
        elif mode == MODE_SRV:
            ext, full = mode, "service type"
        else:
            raise ROSMsgException("Invalid mode: %s"%mode)
        if len(sys.argv) == 1:
            print(fullusage('ros'+mode[1:]))
            sys.exit(0)

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
            print(fullusage('ros'+mode[1:]))
            sys.exit(os.EX_USAGE)
    except KeyError as e:
        print("Unknown message type: %s"%e, file=sys.stderr)
        sys.exit(os.EX_USAGE)
    except rospkg.ResourceNotFound as e:
        print("Invalid package: %s"%e, file=sys.stderr)
        sys.exit(os.EX_USAGE)        
    except ValueError as e:
        print("Invalid type: '%s'"%e, file=sys.stderr)
        sys.exit(os.EX_USAGE)          
    except ROSMsgException as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)        
    except KeyboardInterrupt:
        pass
