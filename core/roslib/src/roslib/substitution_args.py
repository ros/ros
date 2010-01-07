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

"""
Library for processing XML substitution args. This is currently used
by roslaunch and xacro, but it is not yet a top-level ROS feature.
"""

import os
import cStringIO

import roslib.exceptions
import roslib.packages

class SubstitutionException(roslib.exceptions.ROSLibException):
    """
    Base class for exceptions in roslib.substitution_args routines
    """
    pass

def resolve_args(arg_str, context=None, resolve_anon=True):
    """
    Resolves substitution args (see wiki spec U{http://ros.org/wiki/roslaunch}).

    @param arg_str: string to resolve zero or more substitution args
        in. arg_str may be None, in which case resolve_args will
        return None
    @type  arg_str: str
    @param context dict: (optional) dictionary for storing results of
        the 'anon' substitution arg. multiple calls to resolve_args
        should use the same context so that 'anon' substitions resolve
        consistently. If no context is provided, a new one will be
        created for each call.
    @type  context: dict
    @param resolve_anon bool: If True (default), will resolve $(anon
        foo). If false, will leave these args as-is.
    @type  resolve_anon: bool
    @return str: arg_str with substitution args resolved
    @rtype:  str
    @raise SubstitutionException: if there is an error resolving substitution args
    """
    if context is None:
        context = {}
    #parse found substitution args
    if not arg_str:
        return arg_str
    valid = ['find', 'env', 'optenv', 'anon']
    # disabled 'export' due to lack of use and API change
    resolved = arg_str
    for a in _collect_args(arg_str):
        splits = [s for s in a.split(' ') if s]
        if not splits[0] in valid:
            raise SubstitutionException("Unknown substitution command [%s]. Valid commands are %s"%(a, valid))
        command = splits[0]
        args = splits[1:]
        if command == 'find':
            if len(args) != 1:
                raise SubstitutionException("$(find pkg) command only accepts one argument [%s]"%a)
            arg = "$(%s)"%a
            sep = os.sep #set to var for easier testing

            #Force / and \ file separators to the os-native
            #convention. We replace everything from the end of the
            #$(find) command to the next space. As we don't support
            #filenames with spaces, this is dandy.
            idx = resolved.find(arg)+len(arg)
            endidx = resolved.find(' ', idx)
            if endidx < 0:
                endidx = len(resolved)
            slash_orig = resolved[idx:endidx]
            resolved = resolved.replace(slash_orig, slash_orig.replace('/', sep))
            resolved = resolved.replace(slash_orig, slash_orig.replace('\\', sep))
            
            resolved = resolved[0:idx-len(arg)] + roslib.packages.get_pkg_dir(args[0]) + resolved[idx:]
        elif command == 'env':
            if len(args) != 1:
                raise SubstitutionException("$(env var) command only accepts one argument [%s]"%a)
            try:
                resolved = resolved.replace("$(%s)"%a, os.environ[args[0]])
            except KeyError, e:
                raise SubstitutionException("environment variable %s is not set"%str(e))
        elif command == 'optenv':
            if len(args) == 0:
                raise SubstitutionException("$(optenv var) must specify an environment variable [%s]"%a)

            if args[0] in os.environ:
                resolved = resolved.replace("$(%s)"%a, os.environ[args[0]])
            elif len(args) > 1:
                resolved = resolved.replace("$(%s)"%a, ' '.join(args[1:]))
            else:
                resolved = resolved.replace("$(%s)"%a, '')
        # #1559 #1660
        elif command == 'anon' and resolve_anon:
            import socket, time
            if len(args) == 0:
                raise SubstitutionException("$(anon var) must specify a name [%s]"%a)
            elif len(args) > 1:
                raise SubstitutionException("$(anon var) may only specify one name [%s]"%a)
            id = args[0]
            if id in context:
                resolved = resolved.replace("$(%s)"%a, context[id])
            else:
                resolve_to = "%s_%s_%s_%s"%(id, socket.gethostname(), os.getpid(), int(time.time()*1000))
                # RFC 952 allows hyphens, IP addrs can have '.'s, both
                # of which are illegal for ROS names. For good
                # measure, screen ipv6 ':'. 
                resolve_to = resolve_to.replace('.', '_')
                resolve_to = resolve_to.replace('-', '_')                
                resolve_to = resolve_to.replace(':', '_')                
                resolved = resolved.replace("$(%s)"%a, resolve_to)
                context[id] = resolve_to
            
    return resolved

_OUT  = 0
_DOLLAR = 1
_LP = 2
_IN = 3
def _collect_args(arg_str):
    """
    State-machine parser for resolve_args. Substitution args are of the form:
    $(find rospy)/scripts/foo.py $(export some/attribute blar) non-relevant stuff
    
    @param arg_str: argument string to parse args from
    @type  arg_str: str
    @raise SubstitutionException: if args are invalidly specified
    @return: list of arguments
    @rtype: [str]
    """
    buff = cStringIO.StringIO()
    args = []
    state = _OUT
    for c in arg_str:
        # No escapes supported
        if c == '$':
            if state == _OUT:
                state = _DOLLAR
            elif state == _DOLLAR:
                pass
            else:
                raise SubstitutionException("Dollar signs '$' cannot be inside of substitution args [%s]"%arg_str)
        elif c == '(':
            if state == _DOLLAR:
                state = _LP
            elif state != _OUT:
                raise SubstitutionException("Invalid left parenthesis '(' in substitution args [%s]"%arg_str)
        elif c == ')':
            if state == _IN:
                #save contents of collected buffer
                args.append(buff.getvalue())
                buff.truncate(0)
                buff.seek(0)
                state = _OUT
            else:
                state = _OUT
        elif state == _DOLLAR:
            # left paren must immediately follow dollar sign to enter _IN state
            state = _OUT
        elif state == _LP:
            state = _IN

        if state == _IN:
            buff.write(c)
    return args


