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

PKG = 'rosparam'
NAME = 'rosparam'
import roslib; roslib.load_manifest(PKG)

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

## rosparam exception type
class ROSParamException(Exception): pass
class ROSParamIOException(ROSParamException): pass

# pyyaml customizations for binary and angle data

## adds a pyyaml serializer to handle xmlrpclib.Binary objects
def represent_xml_binary(loader, data):
    data = base64.b64encode(data.data)
    return loader.represent_scalar(u'tag:yaml.org,2002:binary', data, style='|')

## overrides pyaml's constructor for binary data. Wraps binary data in
## xmlrpclib.Binary container instead of straight string
## representation.
def construct_yaml_binary(loader, node):
    return xmlrpclib.Binary(loader.construct_yaml_binary(node))
        
# register the (de)serializers with pyyaml
yaml.add_representer(xmlrpclib.Binary,represent_xml_binary)
yaml.add_constructor(u'tag:yaml.org,2002:binary', construct_yaml_binary)

## utility for converting rad(num) into float value
def construct_angle_radians(loader, node):
    value = loader.construct_scalar(node).strip()
    exprvalue = value.replace('pi', 'math.pi')
    if exprvalue.startswith("rad("):
        exprvalue = exprvalue[4:-1]
    try:
        return  float(eval(exprvalue))
    except SyntaxError, e:
        raise ROSParamException("invalid radian expression: %s"%value)

## utility for converting deg(num) into float value
def construct_angle_degrees(loader, node):
    value = loader.construct_scalar(node)
    exprvalue = value
    if exprvalue.startswith("deg("):
        exprvalue = exprvalue.strip()[4:-1]
    try:
        return float(exprvalue) * math.pi / 180.0
    except ValueError:
        raise ROSParamException("invalid degree value: %s"%value)


# utilities

## Utility routine for checking ROS XMLRPC API call
## @return value field from ROS xmlrpc call
## @throws ROSParamException if call to ROS xmlrpc API did not succeed
def succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSParamException(msg)
    return val

## @return str: caller ID for rosparam ROS client calls
def _get_caller_id():
    return make_caller_id('rosparam-%s'%os.getpid())

## print contents of param dictionary to screen
def print_params(params, ns):
    if type(params) == dict:
        for k, v in params.iteritems():
            if type(v) == dict:
                print_params(v, ns_join(ns, k))
            else:
                print "%s=%s"%(ns_join(ns, k), v)
    else:
        print params
    
# yaml processing

## load the YAML document from the specified file
## @param filename str: name of filename
## @param default_namespace str: namespace to load filename into
## @return [(dict, str)...]: list of parameter dictionary and
## corresponding namespaces for each YAML document in the file
## @throws ROSParamException if unable to load contents of \a filename
def load_file(filename, default_namespace=None, verbose=False):
    if not os.path.exists(filename):
        raise ROSParamException("file [%s] does not exist"%filename)
    if verbose:
        print "reading parameters from [%s]"%filename
    f = open(filename, 'r')
    try:
        return load_str(f.read(), filename, default_namespace=default_namespace, verbose=verbose)
    finally:
        f.close()
        
## load the YAML document as a string
## @param filename str: name of filename, only used for debugging    
## @param default_namespace str: namespace to load filename into
## @param str str: YAML text
## @return [(dict, str)...]: list of parameter dictionary and
## corresponding namespaces for each YAML document in the file
def load_str(str, filename, default_namespace=None, verbose=False):
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

## download a parameter from parameter server
## @param param str: parameter name to retrieve from parameter
## server. If \a param is a parameter namespace, entire parameter
## subtree will be downloaded.
def get_param(param):
    try:
        return succeed(get_param_server().getParam(_get_caller_id(), param))
    except socket.error:
        raise ROSParamIOException("Unable to communicate with master!")
    
## pretty print get value #698
def _pretty_print(value, indent=''):
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
            
## download a parameter tree and print to screen
## @param param str: parameter name to retrieve from parameter
## server. If \a param is a parameter namespace, entire parameter
## subtree will be downloaded.
def rosparam_cmd_get_param(param, pretty=False, verbose=False):
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

## download a parameter tree from the param server and store in a yaml file
## @param filename str: name of file to save YAML representation
## @param param str: name of parameter/namespace to dump
def dump_params(filename, param, verbose=False):
    tree = get_param(param)
    if verbose:
        print_params(tree, param)
    f = open(filename, 'w')
    try:
        yaml.dump(tree, f)
    finally:
        f.close()


# DELETE

def delete_param(param, verbose=False):
    try:
        if param == GLOBALNS:
            # not allowed to delete the root of the tree as it must always
            # have a value. the equivalent command is setting the root to an
            # empty dictionary
            succeed(get_param_server().setParam(_get_caller_id(), GLOBALNS, {})) 
            if verbose:
                print "deleted ENTIRE parameter server"
        else:
            succeed(get_param_server().deleteParam(_get_caller_id(), param))
            if verbose:
                print "deleted parameter [%s]"%param
    except socket.error:
        raise ROSParamIOException("Unable to communicate with master!")
    
# LOAD/SET

## Set \a param on the ROS parameter server
## @param param str: \a parameter name
## @param value XmlRpcLegalValue: value to upload
## @internal
def _set_param(param, value, verbose=False):
    if type(value) == dict:
        # #1098 changing dictionary behavior to be an update, rather
        # than replace behavior.
        for k, v in value.iteritems():
            _set_param(ns_join(param, k), v, verbose=verbose)
    else:
        try:
            succeed(get_param_server().setParam(_get_caller_id(), param, value))
        except socket.error:
            raise ROSParamIOException("Unable to communicate with master!")
        if verbose:
            print "set parameter [%s] to [%s]"%(param, value)

## Set \a param on the ROS parameter server
## @param param str: \a parameter name
## @param value str: yaml-encoded value
def set_param(param, value, verbose=False):
    _set_param(param, yaml.load(value), verbose=verbose)

## upload \a params to the ROS parameter server
## @param values dict: key/value dictionary, where keys are parameter names and values are parameter values
## @param ns str: namespace to load parameters into        
def upload_params(ns, values, verbose=False):
    if ns == '/' and not type(values) == dict:
        raise ROSParamException("global / can only be set to a dictionary")
    if verbose:
        print_params(values, ns)
    _set_param(ns, values)


# LIST

## Get list of parameters in \a ns
## @param ns str: namespace to match
## @param [str]: list of matching parameters
def list_params(ns):
    try:
        ns = make_global_ns(ns)
        names = succeed(get_param_server().getParamNames(_get_caller_id()))
        names.sort()
        return [n for n in names if n.startswith(ns)]
    except socket.error:
        raise ROSParamIOException("Unable to communicate with master!")

# COMMAND-LINE PARSING
    
## process command line for rosparam get/dump, e.g.
## \verbatim
## rosparam get param
## rosparam dump file.yaml [namespace]
## \endverbatim
## @param cmd str: 'get' or 'dump'
## @internal
def rosparam_cmd_get_dump(cmd):
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
        rosparam_cmd_get_param(script_resolve_name(NAME, arg), pretty=options.pretty, verbose=options.verbose)
    else:
        if options.verbose:
            print "dumping namespace [%s] to file [%s]"%(ns, arg)
        dump_params(arg, script_resolve_name(NAME, ns), verbose=options.verbose)

## process command line for rosparam set/load, e.g.
## \verbatim
## rosparam load file.yaml [namespace]
## rosparam set param value
## \endverbatim
## @internal
def rosparam_cmd_set_load(cmd):
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

## process command line for rosparam set/load, e.g.
## \verbatim
## rosparam load file.yaml [namespace]
## rosparam set param value
## \endverbatim
## @internal
def rosparam_cmd_list(cmd):
    parser = OptionParser(usage="usage: %prog load [namespace]", prog=NAME)
    options, args = parser.parse_args(sys.argv[2:])

    ns = GLOBALNS
    if len(args) == 1:
        ns = script_resolve_name(NAME, args[0])
    elif len(args) == 2:
        parser.error("too many arguments")

    print '\n'.join(list_params(ns))


## process command line for rosparam delete, e.g.
## \verbatim
## rosparam delete param 
## \endverbatim
## @internal
def rosparam_cmd_delete(cmd):
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

## prints rosparam usage
## @internal
def _fullusage():
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

## Command-line main routine. Loads in one or more input files
def yamlmain():
    if len(sys.argv) == 1:
        _fullusage()
    try:
        command = sys.argv[1]
        if command in ['get', 'dump']:
            rosparam_cmd_get_dump(command)
        elif command in ['set', 'load']:
            rosparam_cmd_set_load(command)
        elif command in ['delete']:
            rosparam_cmd_delete(command)
        elif command == 'list':
            rosparam_cmd_list(command)            
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


