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
General routines and representations for loading roslaunch model.
"""

from __future__ import with_statement

import os
import string
import sys
import traceback
import xmlrpclib
import yaml

from roslib.names import make_global_ns, ns_join, is_global, is_private, PRIV_NAME
from roslib.packages import InvalidROSPkgException

from roslaunch.core import Param, PHASE_SETUP, Master, RosbinExecutable, Node, Test, Machine

# maps master auto attribute to Master auto property
_master_auto = {
    'no': Master.AUTO_NO, 'start': Master.AUTO_START, 'restart': Master.AUTO_RESTART,
}

# TODO: unit test
# #1269, #1270

# TODO: this should be unnecessary once we remove support for unnamed nodes
def command_line_param(key, value):
    """
    Convert parameter into a ROS command-line remapping argument.

    @return: remapping argument. remapping argument does not have a
        leading or trailing space.
    @rtype: str
    """
    # return double-quoted representation of value. In simple
    # command-line tests, this appears to cover common cases. YAML
    # dump uses single-quoted strings to disambiguate overlapping
    # cases (e.g. 'true')

    # have to force UTF-8 in order to get python-yaml to encode cleanly
    if type(value) == unicode:
        value = value.encode('UTF-8')
    # strip the yaml encoding as python-yaml adds a newline
    encoded = yaml.dump(value).strip()
    # #1731 strip the '...' end-of-document indicator as it is not
    # #required (and confusing to users)
    if encoded.endswith('\n...'):
        encoded = encoded[:-4]
    return '_%s:="%s"'%(key, encoded)

#TODO: lists, maps(?)
def convert_value(value, type_):
    """
    Convert a value from a string representation into the specified
    type
    
    @param value: string representation of value
    @type  value: str
    @param type_: int, double, string, bool, or auto
    @type  type_: str
    @raise ValueError: if parameters are invalid
    """
    type_ = type_.lower()
    # currently don't support XML-RPC date, dateTime, maps, or list
    # types
    if type_ == 'auto':
        #attempt numeric conversion
        try:
            if '.' in value:
                return string.atof(value)
            else:
                return string.atoi(value)
        except ValueError, e:
            pass
        #bool
        lval = value.lower()
        if lval == 'true' or lval == 'false':
            return convert_value(value, 'bool')
        #string
        return value
    elif type_ == 'str' or type_ == 'string':
        return value
    elif type_ == 'int':
        return string.atoi(value)
    elif type_ == 'double':
        return string.atof(value)
    elif type_ == 'bool' or type_ == 'boolean':
        value = value.lower()
        if value == 'true' or value == '1':
            return True
        elif value == 'false' or value == '0':
            return False
        raise ValueError("%s is not a '%' type"%(value, type_))
    else:
        raise ValueError("Unknown type '%s'"%type_)        

# TODO: now that 'name' is required, it should be possible to remove params once this is enforced

class LoaderContext(object):
    """
    Container for storing current loader context (e.g. namespace,
    local parameter state, remapping state).
    """
    
    def __init__(self, ns, filename, parent=None, params=None, env_args=None, resolve_dict={}):
        self.parent = parent
        self.ns = make_global_ns(ns or '/')
        self._remap_args = []
        self.params = params or []
        self.env_args = env_args or []
        self.filename = filename
        # for substitution args
        self.resolve_dict = resolve_dict
        
    def add_param(self, p):
        """
        Add a ~param to the context. ~params are evaluated by any node
        declarations that occur later in the same context.

        @param p: parameter
        @type  p: L{Param}
        """
        
        # override any params already set
        matches = [m for m in self.params if m.key == p.key]
        for m in matches:
            self.params.remove(m)
        self.params.append(p)
        
    def add_remap(self, remap):
        """
        Add a new remap setting to the context. if a remap already
        exists with the same from key, it will be removed
        
        @param remap: remap setting
        @type  remap: (str, str)
        """
        matches = [r for r in self._remap_args if r[0] == remap[0]]
        for m in matches:
            self._remap_args.remove(m)
        self._remap_args.append(remap)
        
    def remap_args(self):
        """
        @return: copy of the current remap arguments
        @rtype: [(str, str)]
        """
        if self.parent:
            args = []
            # filter out any parent remap args that have the same from key
            for pr in self.parent.remap_args():
                if not [r for r in self._remap_args if r[0] == pr[0]]:
                    args.append(pr)
            args.extend(self._remap_args)
            return args
        return self._remap_args[:]
    
    def child(self, ns):
        """
        @param ns: sub-namespace of child context, or None if the
           child context shares the same namespace
        @type  ns: str
        @return: A child xml context that inherits from this context
        @rtype: L{LoaderContext}
        """
        if ns:
            if ns[0] == '/': # global (discouraged)
                return LoaderContext(ns, self.filename, parent=self,
                                     params=self.params, env_args=self.env_args[:],
                                     resolve_dict=self.resolve_dict)
            elif ns == PRIV_NAME: # ~name
                # private names can only be scoped privately or globally
                return LoaderContext(PRIV_NAME, self.filename, parent=self,
                                     params=self.params, env_args=self.env_args[:],
                                     resolve_dict=self.resolve_dict)
            else:
                return LoaderContext(ns_join(self.ns, ns), self.filename,
                                     parent=self, params=self.params,
                                     env_args=self.env_args[:], resolve_dict=self.resolve_dict)
        else:
            return LoaderContext(self.ns, self.filename, parent=self,
                                 params=self.params, env_args=self.env_args[:],
                                 resolve_dict=self.resolve_dict)

#TODO: in-progress refactorization. I'm slowly separating out
#non-XML-specific logic from xmlloader and moving into Loader. Soon
#this will mean that it will be easier to write coverage tests for
#lower-level logic.
        
class Loader(object):
    """
    Lower-level library for loading ROS launch model. It provides an
    abstraction between the representation (e.g. XML) and the
    validation of the property values.
    """
    
    def create_master(self, type_, uri, auto_str):
        """
        @param type_: type attribute or None if type attribute not provided
        @type  type_: str
        @param uri: URI attribute or None if not specified
        @type  uri: str
        @param auto_str: auto attribute or None if not provided
        @type  auto_str: str
        @raise ValueError: if parameters cannot be processed into valid Master
        """
        if type_ is not None and type_.strip() == '':
            raise ValueError("'type' must be non-empty")
        
        try: # auto attribute
            auto_str = (auto_str or 'no').lower().strip()
            auto = _master_auto[auto_str]
        except KeyError:
            raise ValueError("invalid <master> 'auto' value: %s"%auto_str)

        # TODO: URI validation
        return Master(type_=type_, uri=uri, auto=auto)

    def add_param(self, ros_config, param_name, param_value, verbose=True):
        """
        Add L{Param} instances to launch config. Dictionary values are
        unrolled into individual parameters.

        @param ros_config: launch configuration
        @type  ros_config: L{ROSLaunchConfig}
        @param param_name: name of parameter namespace to load values
            into. If param_name is '/', param_value must be a dictionary
        @type  param_name: str
        @param param_value: value to assign to param_name. If
            param_value is a dictionary, it's values will be unrolled
            into individual parameters.
        @type  param_value: str
        @raise ValueError: if parameters cannot be processed into valid Params
        """
        
        # shouldn't ever happen
        if not param_name:
            raise ValueError("no parameter name specified")
        
        if param_name == '/' and type(param_value) != dict:
            raise ValueError("Cannot load non-dictionary types into global namespace '/'")

        if type(param_value) == dict:
            # unroll params
            for k, v in param_value.iteritems():
                self.add_param(ros_config, ns_join(param_name, k), v, verbose=verbose)
        else:
            ros_config.add_param(Param(param_name, param_value), verbose=verbose)
        
    def load_rosparam(self, context, ros_config, cmd, param, file, text, verbose=True):
        """
        Load rosparam setting
        
        @param context: Loader context
        @type  context: L{LoaderContext}
        @param ros_config: launch configuration
        @type  ros_config: L{ROSLaunchConfig}
        @param cmd: 'load', 'dump', or 'delete'
        @type  cmd: str
        @param file: filename for rosparam to use or None
        @type  file: str
        @param text: text for rosparam to load. Ignored if file is set.
        @type  text: str
        @raise ValueError: if parameters cannot be processed into valid rosparam setting
        """
        if not cmd in ('load', 'dump', 'delete'):
            raise ValueError("command must be 'load', 'dump', or 'delete'")
        if file is not None:
            if cmd == 'load' and not os.path.isfile(file):
                raise ValueError("file does not exist [%s]"%file)
            if cmd == 'delete':
                raise ValueError("'file' attribute is invalid with 'delete' command.")

        full_param = ns_join(context.ns, param) if param else context.ns

        if cmd == 'dump':
            ros_config.add_executable(RosbinExecutable('rosparam', (cmd, file, full_param), PHASE_SETUP))
        elif cmd == 'delete':
            ros_config.add_executable(RosbinExecutable('rosparam', (cmd, full_param), PHASE_SETUP))
        elif cmd == 'load':
            # load YAML text
            if file:
                with open(file, 'r') as f:
                    text = f.read()
                    
            if not text:
                if file:
                    raise ValueError("no YAML in file %s"%file)
                else:
                    raise ValueError("no YAML to load")
            
            # parse YAML text
            try:
                data = yaml.load(text)
            except Exception, e:
                if not file:
                    raise ValueError("invalid YAML: %s\n\nYAML is:\n%s"%(str(e), text))
                else:
                    raise ValueError("file %s contains invalid YAML:\n%s"%(file, str(e)))

            # 'param' attribute is required for non-dictionary types
            if not param and type(data) != dict:
                raise ValueError("'param' attribute must be set for non-dictionary values")

            self.add_param(ros_config, full_param, data, verbose=verbose)

        else:
            raise XmlParseException("unknown command %s"%cmd)


    def load_env(self, context, ros_config, name, value):
        """
        Load environment variable setting

        @param context: Loader context
        @type  context: L{LoaderContext}
        @param ros_config: launch configuration
        @type  ros_config: L{ROSLaunchConfig}
        @param name: environment variable name
        @type  name: str
        @param value: environment variable value
        @type  value: str
        """
        if not name:
            raise ValueError("'name' attribute must be non-empty")
        context.env_args.append((name, value))


    def param_value(self, verbose, name, ptype, value, textfile, binfile, command):
        """
        Parse text representation of param spec into Python value
        @param name: param name, for error message use only
        @type  name: str
        @param verbose: print verbose output
        @type  verbose: bool
        @param textfile: name of text file to load from, or None
        @type  textfile: str        
        @param binfile: name of binary file to load from, or None
        @type  binfile: str        
        @param command: command to execute for parameter value, or None
        @type  command: str
        @raise ValueError: if parameters are invalid
        """
        if value is not None:
            return convert_value(value.strip(), ptype)
        elif textfile is not None:
            f = open(textfile, 'r')
            try:
                return f.read()
            finally:
                f.close()
        elif binfile is not None:
            f = open(binfile, 'rb')
            try:
                return xmlrpclib.Binary(f.read())
            finally:
                f.close()
        elif command is not None:
            if type(command) == unicode:
                command = command.encode('UTF-8') #attempt to force to string for shlex/subprocess
            if verbose:
                print "... executing command param [%s]"%command
            import subprocess, shlex #shlex rocks
            try:
                p = subprocess.Popen(shlex.split(command), stdout=subprocess.PIPE)
                c_value = p.communicate()[0]
                if p.returncode != 0:
                    raise ValueError("Cannot load command parameter [%s]: command [%s] returned with code [%s]"%(name, command, p.returncode))
            except OSError, (errno, strerr):
                if errno == 2:
                    raise ValueError("Cannot load command parameter [%s]: no such command [%s]"%(name, command))
                raise
            if c_value is None:
                raise ValueError("parameter: unable to get output of command [%s]"%command)
            return c_value
        else: #_param_tag prevalidates, so this should not be reachable
            raise ValueError("unable to determine parameter value")

