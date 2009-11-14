#!/usr/bin/env python
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
# Revision $Id: rosparam 1641 2008-07-28 21:39:33Z sfkwc $

"""
Implementation of the rosparam as well as a library for modifying the
state of the ROS Parameter Server using YAML files.
"""

NAME = 'rosparam'
import roslib; roslib.load_manifest('rosparam')

## namespace key. Use of this in a YAML document specifies the
## namespace of all the params.  NOTE: phasing out most use of this
## key. It's still useful in corner cases, but most of its
## functionality can be achieved with command-line arguments.
NS = '_ns'

import base64
import math
import os
import re
import sys
import socket
import traceback
import xmlrpclib

from optparse import OptionParser

import yaml

from roslib.names import ns_join, get_ros_namespace, make_caller_id, make_global_ns, GLOBALNS
from roslib.scriptutil import get_param_server, script_resolve_name

class ROSParamException(Exception):
    """
    rosparam base exception type
    """
    pass
class ROSParamIOException(ROSParamException):
    """
    Exception for communication-based (i/o) errors.
    """
    pass

# pyyaml customizations for binary and angle data

def represent_xml_binary(loader, data):
    """
    Adds a pyyaml serializer to handle xmlrpclib.Binary objects
    """
    data = base64.b64encode(data.data)
    return loader.represent_scalar(u'tag:yaml.org,2002:binary', data, style='|')

def construct_yaml_binary(loader, node):
    """
    Overrides pyaml's constructor for binary data. Wraps binary data in
    xmlrpclib.Binary container instead of straight string
    representation.
    """
    return xmlrpclib.Binary(loader.construct_yaml_binary(node))
        
# register the (de)serializers with pyyaml
yaml.add_representer(xmlrpclib.Binary,represent_xml_binary)
yaml.add_constructor(u'tag:yaml.org,2002:binary', construct_yaml_binary)

def construct_angle_radians(loader, node):
    """
    python-yaml utility for converting rad(num) into float value
    """
    value = loader.construct_scalar(node).strip()
    exprvalue = value.replace('pi', 'math.pi')
    if exprvalue.startswith("rad("):
        exprvalue = exprvalue[4:-1]
    try:
        return float(eval(exprvalue))
    except SyntaxError, e:
        raise ROSParamException("invalid radian expression: %s"%value)

def construct_angle_degrees(loader, node):
    """
    python-yaml utility for converting deg(num) into float value
    """
    value = loader.construct_scalar(node)
    exprvalue = value
    if exprvalue.startswith("deg("):
        exprvalue = exprvalue.strip()[4:-1]
    try:
        return float(exprvalue) * math.pi / 180.0
    except ValueError:
        raise ROSParamException("invalid degree value: %s"%value)


# utilities

def _succeed(args):
    """
    Utility routine for checking ROS XMLRPC API call
    @return: value field from ROS xmlrpc call
    @rtype: XmlRpcLegalValue
    @raise ROSParamException: if call to ROS xmlrpc API did not succeed
    """
    code, msg, val = args
    if code != 1:
        raise ROSParamException(msg)
    return val

def _get_caller_id():
    """
    @return: caller ID for rosparam ROS client calls
    @rtype: str
    """
    return make_caller_id('rosparam-%s'%os.getpid())

def print_params(params, ns):
    """
    Print contents of param dictionary to screen
    """
    if type(params) == dict:
        for k, v in params.iteritems():
            if type(v) == dict:
                print_params(v, ns_join(ns, k))
            else:
                print "%s=%s"%(ns_join(ns, k), v)
    else:
        print params
    
# yaml processing

def load_file(filename, default_namespace=None, verbose=False):
    """
    Load the YAML document from the specified file
    
    @param filename: name of filename
    @type  filename: str
    @param default_namespace: namespace to load filename into
    @type  default_namespace: str
    @return [(dict, str)...]: list of parameter dictionary and
    corresponding namespaces for each YAML document in the file
    @raise ROSParamException: if unable to load contents of filename
    """
    if not os.path.isfile(filename):
        raise ROSParamException("file [%s] does not exist"%filename)
    if verbose:
        print "reading parameters from [%s]"%filename
    f = open(filename, 'r')
    try:
        return load_str(f.read(), filename, default_namespace=default_namespace, verbose=verbose)
    finally:
        f.close()
        
def load_str(str, filename, default_namespace=None, verbose=False):
    """
    Load the YAML document as a string
    
    @param filename: name of filename, only used for debugging    
    @type  filename: str
    @param default_namespace: namespace to load filename into
    @type  default_namespace: str
    @param str: YAML text
    @type  str: str
    @return: list of parameter dictionary and
        corresponding namespaces for each YAML document in the file
    @rtype: [(dict, str)...]
    """
    paramlist = []
    default_namespace = default_namespace or get_ros_namespace()
    for doc in yaml.load_all(str):
        if NS in doc:
            ns = ns_join(default_namespace, doc.get(NS, None))
            if verbose:
                print "reading parameters into namespace [%s]"%ns
            del doc[NS]
        else:
            ns = default_namespace
        paramlist.append((doc, ns))
    return paramlist


# DUMP/GET

def get_param(param):
    """
    Download a parameter from Parameter Server

    @param param: parameter name to retrieve from parameter
        server. If param is a parameter namespace, entire parameter
        subtree will be downloaded.
    @type  param: str
    """
    try:
        return _succeed(get_param_server().getParam(_get_caller_id(), param))
    except socket.error:
        raise ROSParamIOException("Unable to communicate with master!")
    
# #698
def _pretty_print(value, indent=''):
    """
    Pretty print get value
    @param value: value to print
    @param indent: indent level, used for recursive calls
    @type  indent: str
    """
    keys = value.keys()
    keys.sort()
    for k in keys:
        v = value[k]
        if type(v) == dict:
            print "%s%s:"%(indent, k)
            _pretty_print(v, indent+'  ')
        elif type(v) == str:
            if '\n' in v:
                print indent+'%s: |'%k
                for l in v.split('\n'):
                    print indent+'  '+l
            else:
                print "%s%s: %s"%(indent, k, v)
        else:
            dump = yaml.dump(v)
            # #1617
            # newer versions of python-yaml append the '...' document end
            # syntax.  as YAML functions fine w/o it, and as it is
            # confusing to users who are just getting a single scalar, we
            # strip it
            if dump.endswith('\n...\n'):
                dump = dump[:-5]
            
            sys.stdout.write("%s%s: %s"%(indent, k, dump))
            
def _rosparam_cmd_get_param(param, pretty=False, verbose=False):
    """
    Download a parameter tree and print to screen
    @param param: parameter name to retrieve from Parameter
        Server. If param is a parameter namespace, entire parameter
        subtree will be downloaded.
    @type  param: str
    """
    # yaml.dump has a \n at the end, so use stdout.write instead of print
    if verbose:
        print "getting parameter [%s]"%param
    val = get_param(param)
    if pretty and type(val) in [dict, str]:
        if type(val) == dict:
            _pretty_print(val)
        else:
            if '\n' in val:
                print '|'
                for l in val.split('\n'):
                    print '  '+l
            else:
                print val
    else:
        dump = yaml.dump(val)
        # #1617
        # newer versions of python-yaml append the '...' document end
        # syntax.  as YAML functions fine w/o it, and as it is
        # confusing to users who are just getting a single scalar, we
        # strip it
        if dump.endswith('\n...\n'):
            dump = dump[:-5]

        sys.stdout.write(dump)

def dump_params(filename, param, verbose=False):
    """
    Download a parameter tree from the Parameter Server and store in a yaml file
    @param filename: name of file to save YAML representation
    @type  filename: str
    @param param: name of parameter/namespace to dump
    @type  param: str
    @param verbose: print verbose output for debugging
    @type  verbose: bool
    """
    tree = get_param(param)
    if verbose:
        print_params(tree, param)
    f = open(filename, 'w')
    try:
        yaml.dump(tree, f)
    finally:
        f.close()


def delete_param(param, verbose=False):
    """
    Delete a parameter from the Parameter Server
    @param param: parameter name
    @type  param: str
    @param verbose: print verbose output for debugging
    @type  verbose: bool
    """
    try:
        if param == GLOBALNS:
            # not allowed to delete the root of the tree as it must always
            # have a value. the equivalent command is setting the root to an
            # empty dictionary
            _succeed(get_param_server().setParam(_get_caller_id(), GLOBALNS, {})) 
            if verbose:
                print "deleted ENTIRE parameter server"
        else:
            _succeed(get_param_server().deleteParam(_get_caller_id(), param))
            if verbose:
                print "deleted parameter [%s]"%param
    except socket.error:
        raise ROSParamIOException("Unable to communicate with master!")
    
# LOAD/SET

def _set_param(param, value, verbose=False):
    """
    Set param on the Parameter Server. Unlike L{set_param()}, this takes in a Python value to set instead of YAML.
    @param param: parameter name
    @type  param: str
    @param value XmlRpcLegalValue: value to upload
    @type  value: XmlRpcLegalValue
    """
    if type(value) == dict:
        # #1098 changing dictionary behavior to be an update, rather
        # than replace behavior.
        for k, v in value.iteritems():
            # dictionary keys must be non-unicode strings
            if isinstance(k, str):
                _set_param(ns_join(param, k), v, verbose=verbose)
            else:
                raise ROSParamException("YAML dictionaries must have string keys. Invalid dictionary is:\n%s"%value)
    else:
        if type(value) == long:
            if value > sys.maxint:
                raise ROSParamException("Overflow: Parameter Server integers must be 32-bit signed integers:\n\t-%s <= value <= %s"%(sys.maxint-1, sys.maxint))
            
        try:
            _succeed(get_param_server().setParam(_get_caller_id(), param, value))
        except socket.error:
            raise ROSParamIOException("Unable to communicate with master!")
        if verbose:
            print "set parameter [%s] to [%s]"%(param, value)

def set_param(param, value, verbose=False):
    """
    Set param on the ROS parameter server using a YAML value
    @param param: parameter name
    @type  param: str
    @param value: yaml-encoded value
    @type  value: str
    """
    _set_param(param, yaml.load(value), verbose=verbose)

def upload_params(ns, values, verbose=False):
    """
    Upload params to the Parameter Server
    @param values: key/value dictionary, where keys are parameter names and values are parameter values
    @type  values: dict
    @param ns: namespace to load parameters into        
    @type  ns: str
    """
    if ns == '/' and not type(values) == dict:
        raise ROSParamException("global / can only be set to a dictionary")
    if verbose:
        print_params(values, ns)
    _set_param(ns, values)


# LIST

def list_params(ns):
    """
    Get list of parameters in ns
    @param ns: namespace to match
    @type  ns: str
    """
    try:
        ns = make_global_ns(ns)
        names = _succeed(get_param_server().getParamNames(_get_caller_id()))
        names.sort()
        return [n for n in names if n.startswith(ns)]
    except socket.error:
        raise ROSParamIOException("Unable to communicate with master!")

# COMMAND-LINE PARSING
    
def _rosparam_cmd_get_dump(cmd):
    """
    Process command line for rosparam get/dump, e.g.::
      rosparam get param
      rosparam dump file.yaml [namespace]
    @param cmd str: command ('get' or 'dump')
    @type  cmd: str
    """
    # get and dump are equivalent functionality, just different arguments
    if cmd == 'dump':
        parser = OptionParser(usage="usage: %prog dump [options] file [namespace]", prog=NAME)
    elif cmd == 'get':
        parser = OptionParser(usage="usage: %prog get [options] parameter", prog=NAME)        
        parser.add_option("-p", dest="pretty", default=False,
                          action="store_true", help="pretty print. WARNING: not YAML-safe")

    parser.add_option("-v", dest="verbose", default=False,
                      action="store_true", help="turn on verbose output")
    options, args = parser.parse_args(sys.argv[2:])

    arg = None
    ns = ''
    
    if len(args) == 0:
        if cmd == 'dump':
            parser.error("invalid arguments. Please specify a file name")
        elif cmd == 'get':
            parser.error("invalid arguments. Please specify a parameter name")
    elif len(args) == 1:
        arg = args[0]
    elif len(args) == 2 and cmd == 'dump':
        arg = args[0]
        ns = args[1]
    else:
        parser.error("too many arguments")

    if cmd == 'get':
        _rosparam_cmd_get_param(script_resolve_name(NAME, arg), pretty=options.pretty, verbose=options.verbose)
    else:
        if options.verbose:
            print "dumping namespace [%s] to file [%s]"%(ns, arg)
        dump_params(arg, script_resolve_name(NAME, ns), verbose=options.verbose)

def _rosparam_cmd_set_load(cmd):
    """
    Process command line for rosparam set/load, e.g.::
      rosparam load file.yaml [namespace]
      rosparam set param value
    @param cmd: command name
    @type  cmd: str
    """
    if cmd == 'load':
        parser = OptionParser(usage="usage: %prog load [options] file [namespace]", prog=NAME)
    elif cmd == 'set':
        parser = OptionParser(usage="usage: %prog set [options] parameter value", prog=NAME)
    parser.add_option("-v", dest="verbose", default=False,
                      action="store_true", help="turn on verbose output")

    # we don't use optparse to parse actual arguments, just options,
    # due to the fact that optparse doesn't handle negative numbers as
    # arguments.
    args = []
    optparse_args = []
    for s in sys.argv[2:]:
        if s.startswith('-'):
            if len(s) > 1 and ord(s[1]) >= ord('0') and ord(s[1]) < ord('9'):
                args.append(s)
            else:
                optparse_args.append(s)
        else:
            args.append(s)
    options, _ = parser.parse_args(optparse_args)

    arg2 = None
    if len(args) == 0:
        if cmd == 'load':
            parser.error("invalid arguments. Please specify a file name")
        elif cmd == 'set':
            parser.error("invalid arguments. Please specify a parameter name")
    elif len(args) == 1:
        arg = args[0]
        if cmd == 'set':
            parser.error("invalid arguments. Please specify a parameter value")
    elif len(args) == 2:
        arg = args[0]
        arg2 = args[1]
    else:
        parser.error("too many arguments")

    if cmd == 'set':
        set_param(script_resolve_name(NAME, arg), arg2, verbose=options.verbose)
    else:
        paramlist = load_file(arg, default_namespace=script_resolve_name(NAME, arg2), verbose=options.verbose)
        for params,ns in paramlist:
            upload_params(ns, params, verbose=options.verbose)

def _rosparam_cmd_list(cmd):
    """
    Process command line for rosparam set/load, e.g.::
      rosparam load file.yaml [namespace]
      rosparam set param value
    @param cmd: command name
    @type  cmd: str
    """
    parser = OptionParser(usage="usage: %prog load [namespace]", prog=NAME)
    options, args = parser.parse_args(sys.argv[2:])

    ns = GLOBALNS
    if len(args) == 1:
        ns = script_resolve_name(NAME, args[0])
    elif len(args) == 2:
        parser.error("too many arguments")

    print '\n'.join(list_params(ns))


def _rosparam_cmd_delete(cmd):
    """
    Process command line for rosparam delete, e.g.::
      rosparam delete param 
    @param cmd: command name
    @type  cmd: str
    """
    parser = OptionParser(usage="usage: %prog delete [options] parameter", prog=NAME)
    parser.add_option("-v", dest="verbose", default=False,
                      action="store_true", help="turn on verbose output")
    options, args = parser.parse_args(sys.argv[2:])

    arg2 = None
    if len(args) == 0:
        parser.error("invalid arguments. Please specify a parameter name")
    elif len(args) == 1:
        arg = args[0]
    else:
        parser.error("too many arguments")

    delete_param(script_resolve_name(NAME, arg), verbose=options.verbose)

def _fullusage():
    """
    Prints rosparam usage
    """
    print """rosparam is a command-line tool for getting, setting, and deleting parameters from the ROS Parameter Server.

Commands:
\trosparam set\tset parameter
\trosparam get\tget parameter
\trosparam load\tload parameters from file
\trosparam dump\tdump parameters to file
\trosparam delete\tdelete parameter
\trosparam list\tlist parameter names
"""
    sys.exit(0)

def yamlmain():
    """
    Command-line main routine. Loads in one or more input files    
    """
    if len(sys.argv) == 1:
        _fullusage()
    try:
        command = sys.argv[1]
        if command in ['get', 'dump']:
            _rosparam_cmd_get_dump(command)
        elif command in ['set', 'load']:
            _rosparam_cmd_set_load(command)
        elif command in ['delete']:
            _rosparam_cmd_delete(command)
        elif command == 'list':
            _rosparam_cmd_list(command)            
        else:
            _fullusage()
    except ROSParamException, e:
        print >> sys.stderr, "ERROR: "+str(e)

# YAML configuration. Doxygen does not like these being higher up in the code

yaml.add_constructor(u'!radians', construct_angle_radians)
yaml.add_constructor(u'!degrees', construct_angle_degrees)

# allow both !degrees 180, !radians 2*pi
pattern = re.compile(r'^deg\([^\)]*\)$')
yaml.add_implicit_resolver(u'!degrees', pattern)
pattern = re.compile(r'^rad\([^\)]*\)$')
yaml.add_implicit_resolver(u'!radians', pattern)


if __name__ == '__main__':
    yamlmain()

