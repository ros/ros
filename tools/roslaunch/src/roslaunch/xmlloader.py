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

import itertools
import os
import sys
import xmlrpclib
import yaml
import traceback

from xml.dom.minidom import parse, parseString
from xml.dom import Node as DomNode
from core import *

from roslib.names import make_global_ns, ns_join, is_global, is_private, PRIV_NAME
from roslib.packages import InvalidROSPkgException
import roslib.substitution_args

# use in our namespace
SubstitutionException = roslib.substitution_args.SubstitutionException


NS='ns'
CLEAR_PARAMS='clear_params'

# TODO: unit test
# #1269, #1270

## convert parameter \a key with \a value to a ROS command-line
## remapping argument.
## @return str: remapping argument. remapping argument does not have a
## leading or trailing space.
def _command_line_param(key, value):
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

# This code has gotten a bit crufty as roslaunch has grown far beyond
# its original spec. It needs to be far more generic than it is in
# order to not replicate bugs in multiple places.

class XmlParseException(RLException): pass
class XmlLoadException(RLException): pass

class _XmlContext(object):
    def __init__(self, ns, filename, parent=None, params=None, env_args=None, resolve_dict={}):
        self.parent = parent
        self.ns = make_global_ns(ns or '/')
        self._remap_args = []
        self.params = params or []
        self.env_args = env_args or []
        self.filename = filename
        # for substitution args
        self.resolve_dict = resolve_dict
        
    ## add a ~param to the context. ~params are evaluated by any node
    ## declarations that occur later in the same context.
    def add_param(self, p):
        # override any params already set
        matches = [m for m in self.params if m.key == p.key]
        for m in matches:
            self.params.remove(m)
        self.params.append(p)
        
    ## add a new remap setting to the context. if a remap already
    ## exists with the same from key, it will be removed
    def add_remap(self, remap):
        matches = [r for r in self._remap_args if r[0] == remap[0]]
        for m in matches:
            self._remap_args.remove(m)
        self._remap_args.append(remap)
    ## get a copy of the current remap arguments
    def remap_args(self):
        if self.parent:
            args = []
            # filter out any parent remap args that have the same from key
            for pr in self.parent.remap_args():
                if not [r for r in self._remap_args if r[0] == pr[0]]:
                    args.append(pr)
            args.extend(self._remap_args)
            return args
        return self._remap_args[:]
    ## @param ns str: sub-namespace of child context, or None if the
    ##   child context shares the same namespace
    ## @return A child xml context that inherits from this context
    def child(self, ns):
        if ns:
            if ns[0] == '/': # global (discouraged)
                return _XmlContext(ns, self.filename, parent=self, params=self.params, env_args=self.env_args[:], resolve_dict=self.resolve_dict)
            elif ns == PRIV_NAME: # ~name
                # private names can only be scoped privately or globally
                return _XmlContext(PRIV_NAME, self.filename, parent=self, params=self.params, env_args=self.env_args[:], resolve_dict=self.resolve_dict)
            else:
                return _XmlContext(ns_join(self.ns, ns), self.filename, parent=self, params=self.params, env_args=self.env_args[:], resolve_dict=self.resolve_dict)
        else:
            return _XmlContext(self.ns, self.filename, parent=self, params=self.params, env_args=self.env_args[:], resolve_dict=self.resolve_dict)

# maps master auto attribute to Master auto property
_master_auto = {
    'no': Master.AUTO_NO, 'start': Master.AUTO_START, 'restart': Master.AUTO_RESTART,
}
# maps node respawn attribute to Node respawn property
_respawn = { 'true': True, 'false': False }
# maps machine 'default' attribute to Machine default property
_is_default = {'true': True, 'false': False, 'never': False }
# maps machine 'default' attribute to Machine assignable property
_assignable = {'true': True, 'false': True, 'never': False }

## Parser for roslaunch XML format. Loads parsed representation into ROSConfig model.
class XmlLoader(object):

    ## @param resolve_anon bool: If True (default), will resolve $(anon foo). If
    ## false, will leave these args as-is.
    def __init__(self, resolve_anon=True):
        # store the root XmlContext so that outside code can access it
        self.root_context = None
        self.resolve_anon = resolve_anon

    ## wrapper around roslib.substitution_args.resolve_args to set common parameters
    def resolve_args(self, args, context):
        return roslib.substitution_args.resolve_args(args, context=context.resolve_dict, resolve_anon=self.resolve_anon)

    ## helper routine for fetching and resolving optional tag attributes
    ## @param tag DOM tag
    ## @param context _XmlContext
    ## @param attrs (str): list of attributes to resolve        
    def opt_attrs(self, tag, context, attrs):
        def tag_value(tag, a):
            if tag.hasAttribute(a):
                # getAttribute returns empty string for non-existent
                # attributes, which makes it impossible to distinguish
                # with actual empty values
                return tag.getAttribute(a)
            else:
                return None
        return [self.resolve_args(tag_value(tag,a), context) for a in attrs]

    ## helper routine for fetching and resolving required tag attributes
    ## @param tag DOM tag
    ## @param attrs (str): list of attributes to resolve        
    ## @raise KeyError if required attribute is missing
    def reqd_attrs(self, tag, context, attrs):
        return [self.resolve_args(tag.attributes[a].value, context) for a in attrs]

    def _check_attrs(self, tag, context, ros_config, attrs):
        tag_attrs = tag.attributes.keys()
        for t_a in tag_attrs:
            if not t_a in attrs:
                ros_config.add_config_error("[%s] unknown <%s> attribute '%s'"%(context.filename, tag.tagName, t_a))
                #print >> sys.stderr, "WARNING: 

    MASTER_ATTRS = ('type', 'uri', 'auto')
    def _master_tag(self, tag, context, ros_config):
        self._check_attrs(tag, context, ros_config, XmlLoader.MASTER_ATTRS)
        type, uri, auto_str = self.opt_attrs(tag, context, ('type', 'uri', 'auto'))
        if tag.hasAttribute('type') and not len(type.strip()):
            raise XmlParseException("<master> 'type' must be non-empty")
        try: # auto attribute
            auto_str = (auto_str or 'no').lower().strip()
            auto = _master_auto[auto_str]
        except KeyError:
            raise XmlParseException("invalid <master> 'auto' value: %s"%auto_str)
        try:
            return Master(type_=type, uri=uri, auto=auto)
        except ValueError, e:
            raise XmlParseException("invalid <master> tag: %s"%str(e))

    # rosparam tag also has optional 'ns' attribute and must have one
    # of 'file' or 'param'
    ROSPARAM_ATTRS = ('command',)
    ROSPARAM_OPT_ATTRS = ('file', 'param')
    def _rosparam_tag(self, tag, context, ros_config):
        try:
            cmd = self.reqd_attrs(tag, context, XmlLoader.ROSPARAM_ATTRS)[0]
            file, param = self.opt_attrs(tag, context, (XmlLoader.ROSPARAM_OPT_ATTRS))
            if not cmd in ('load', 'dump', 'delete'):
                raise XmlParseException("<rosparam> 'command' must be 'load', 'dump', or 'delete'")

            # validate file/param attributes with respect to command
            if cmd in ('load', 'dump'):
                yes_str, no_str, yes, no = 'file', 'param', file, param
            else:
                yes_str, no_str, yes, no = 'param', 'file', param, file
            if not yes:
                raise XmlParseException("<rosparam> '%s' must be set for '%s' command"%(yes_str, cmd))
            if no:
                raise XmlParseException("<rosparam> '%s' must not be set for '%s' command"%(no_str, cmd))
            
            child_ns = self._ns_clear_params_attr('rosparam', tag, context, ros_config)

            # return the rosparam exe
            if cmd in ('load', 'dump'):
                return RosbinExecutable('rosparam', (cmd, file, child_ns.ns), PHASE_SETUP)
            else:
                return RosbinExecutable('rosparam', (cmd, ns_join(child_ns.ns, param)), PHASE_SETUP)

        except KeyError, e:
            raise XmlParseException(
                "<rosparam> tag is missing required attribute: %s. rosparam xml is %s"%(e, tag.toxml()))

    ## @param name str: param name, for error message use only
    def _param_value(self, verbose, name, ptype, value, textfile, binfile, command):
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
                    raise XmlLoadException("Cannot load command parameter [%s]: command [%s] returned with code [%s]"%(name, command, p.returncode))
            except OSError, (errno, strerr):
                if errno == 2:
                    raise XmlLoadException("Cannot load command parameter [%s]: no such command [%s]"%(name, command))
                raise
            if c_value is None:
                raise XmlLoadException("parameter: unable to get output of command [%s]"%command)
            return c_value
        else: #_param_tag prevalidates, so this should not be reachable
            raise XmlParseException("unable to determine parameter value")

    PARAM_ATTRS = ('name', 'value', 'type', 'value', 'textfile', 'binfile', 'command')
    ## @param force_local bool: if True, param must be added to \a context instead of \a ros_config
    def _param_tag(self, tag, context, ros_config, force_local=False, verbose=True):
        try:
            self._check_attrs(tag, context, ros_config, XmlLoader.PARAM_ATTRS)

            # compute name and value
            ptype = (tag.getAttribute('type') or 'auto').lower().strip()
            
            vals = self.opt_attrs(tag, context, ('value', 'textfile', 'binfile', 'command'))
            if len([v for v in vals if v is not None]) != 1:
                raise XmlParseException(
                    "<param> tag must have one and only one of value/textfile/binfile.")

            # compute name. if name is a tilde name, it is placed in
            # the context. otherwise it is placed in the ros config.
            name = self.resolve_args(tag.attributes['name'].value.strip(), context)
            value = self._param_value(verbose, name, ptype, *vals)

            if is_private(name) or force_local:
                p = Param(name, value)
                context.add_param(p)
            else:
                p = Param(ns_join(context.ns, name), value)
                ros_config.add_param(Param(ns_join(context.ns, name), value), filename=context.filename, verbose=verbose)
            return p
        except KeyError, e:
            raise XmlParseException(
                "<param> tag is missing required attribute: %s. \n\nParam xml is %s"%(e, tag.toxml()))
        except XmlLoadException, e:
            raise
        except Exception, e:
            raise XmlParseException(
                "Invalid <param> tag: %s. \n\nParam xml is %s"%(e, tag.toxml()))


    ## process attributes of <test> tag not present in <node>
    ## @return str, int: test_name, time_limit
    def _test_attrs(self, tag, context):
        for attr in ['respawn', 'output']:
            if tag.hasAttribute(attr):
                raise XmlParseException("<test> tags cannot have '%s' attribute"%attr)

        test_name = self.resolve_args(tag.attributes['test-name'].value, context)
        time_limit = self.resolve_args(tag.getAttribute('time-limit'), context)
        retry = self.resolve_args(tag.getAttribute('retry'), context)        
        if time_limit:
            try:
                time_limit = float(time_limit)
            except ValueError:
                raise XmlParseException("'time-limit' must be a number: [%s]"%time_limit)
            if time_limit <= 0.0:
                raise XmlParseException("'time-limit' must be a positive number")
        if retry:
            try:
                retry = string.atoi(retry)
            except ValueError:
                raise XmlParseException("'retry' must be a number: [%s]"%retry)

        return test_name, time_limit, retry
        
    NODE_ATTRS = ['pkg', 'type', 'machine', 'name', 'args', 'output', 'respawn', 'cwd', NS, CLEAR_PARAMS, 'launch-prefix']
    TEST_ATTRS = NODE_ATTRS + ['test-name','time-limit', 'retry']
    
    ## @param tag Node: DOM node
    ## @param context _XmlContext: namespace context
    ## @param params [Param]: ROS parameter list
    ## @param clear_params [str]: list of ROS parameter names to clear before setting parameters
    ## @param default_machine str: default machine to assign to node
    ## @param is_test bool: if set, will load as Test object instead
    ## of Node object
    def _node_tag(self, tag, context, ros_config, default_machine, is_test=False, verbose=True):
        try:
            if is_test:
                self._check_attrs(tag, context, ros_config, XmlLoader.TEST_ATTRS)
            else:
                self._check_attrs(tag, context, ros_config, XmlLoader.NODE_ATTRS)
                
            # required attributes
            pkg, node_type = self.reqd_attrs(tag, context, ('pkg', 'type'))
            
            if not len(pkg.strip()):
                raise XmlParseException("<node> 'pkg' must be non-empty")
            if not len(node_type.strip()):
                raise XmlParseException("<node> 'type' must be non-empty")
            
            # optional attributes
            machine, name, args, output, respawn, cwd, launch_prefix = \
                     self.opt_attrs(tag, context, ('machine', 'name', 'args', 'output', 'respawn', 'cwd', 'launch-prefix'))
            if not name and not is_test:
                ros_config.add_config_error("WARN: un-named nodes in roslaunch are deprecated:\n[%s]: %s"%(context.filename, tag.toxml()))
                
            # #1821, namespaces in nodes need to be banned
            if name and roslib.names.SEP in name:
                raise XmlParseException("<%s> 'name' cannot contain a namespace"%tag.tagName)

            args = args or ''
            child_ns = self._ns_clear_params_attr('node', tag, context, ros_config, node_name=name)

            if tag.hasAttribute('machine') and not len(machine.strip()):
                raise XmlParseException("<node> 'machine' must be non-empty: [%s]"%machine)
            if not machine and default_machine:
                machine = default_machine.name
            # valid values are 'log' or 'screen'
            output = output or 'log'
            if not output in ['log', 'screen']:
                raise XmlParseException("<%s> 'output' attribute must be one of: 'log', 'screen'"%tag.tagName)

            try:
                respawn = _respawn[(respawn or "false").lower()]
            except KeyError:
                raise XmlParseException("Invalid respawn value: %s"%respawn)

            valid_cwd = ['ros-root', 'node']
            if cwd and cwd not in valid_cwd:
                raise XmlParseException("<%s> 'cwd' attribute must be one of: %s"%(tag.tagName, ','.join(valid_cwd)))

            # each node gets its own copy of <remap> arguments, which
            # it inherits from its parent
            remap_context = context.child('')

            param_ns = child_ns.child(name)
            
            # nodes can have individual env args set in addition to
            # the ROS-specific ones.  
            env_args = context.env_args
            for t in [c for c in tag.childNodes if c.nodeType == DomNode.ELEMENT_NODE]:
                tagName = t.tagName.lower()
                if tagName == 'remap':
                    remap_context.add_remap(self._remap_tag(t, context, ros_config))
                elif tagName == 'param':
                    self._param_tag(t, param_ns, ros_config, force_local=True, verbose=verbose)
                elif tagName == 'rosparam':
                    # #1883 <test> tags use test-name attribute instead
                    if not is_test and not name:
                        raise XmlParseException(
                            "<node> tag must have a 'name' attribute in order to use <rosparam> tags: %s"%t.toxml())
                    ros_config.add_executable(self._rosparam_tag(t, param_ns, ros_config)) 
                elif tagName == 'env':
                    env_args.append(self._env_tag(t, context, ros_config))
                else:
                    print >> sys.stderr, \
                          "WARN: unrecognized '%s' tag in <node> tag"%t.tagName

            # #1036 evaluate all ~params in context
            for p in itertools.chain(context.params, param_ns.params):
                pkey = p.key
                if is_private(pkey):
                    # strip leading ~, which is optional/inferred
                    pkey = pkey[1:]

                if name:
                    pkey = param_ns.ns + pkey
                    ros_config.add_param(Param(pkey, p.value), verbose=verbose)
                elif args:
                    # don't know node name, have to pass in on command-line
                    # strip ~ as parameter args are always private
                    args = args + " " + _command_line_param(pkey, p.value)
                else:
                    args = _command_line_param(pkey, p.value)
                    
            if not is_test:
                return Node(pkg, node_type, name=name, namespace=child_ns.ns, machine_name=machine, 
                            args=args, respawn=respawn, 
                            remap_args=remap_context.remap_args(), env_args=env_args,
                            output=output, cwd=cwd, launch_prefix=launch_prefix)
            else:
                test_name, time_limit, retry = self._test_attrs(tag, context)
                if not name:
                    name = test_name
                return Test(test_name, pkg, node_type, name=name, namespace=child_ns.ns, 
                            machine_name=machine, args=args,
                            remap_args=context.remap_args(), env_args=env_args,
                            time_limit=time_limit, cwd=cwd, launch_prefix=launch_prefix,
                            retry=retry)
        except KeyError, e:
            raise XmlParseException(
                "<%s> tag is missing required attribute: %s. Node xml is %s"%(tag.tagName, e, tag.toxml()))
        except Exception, e:
            raise XmlParseException(
                "Invalid <node> tag: %s. \n\nNode xml is %s"%(e, tag.toxml()))

    MACHINE_ATTRS = ('name', 'address', 'ros-root', 'ros-package-path', 'ros-ip', 'ros-host-name', 
                     'ssh-port', 'user', 'password', 'default', 'timeout')
    def _machine_tag(self, tag, context, ros_config, verbose=True):
        try:
            self._check_attrs(tag, context, ros_config, XmlLoader.MACHINE_ATTRS)
            # required attributes
            name, address = self.reqd_attrs(tag, context, ('name', 'address'))
            
            # optional attributes

            attrs = self.opt_attrs(tag, context,
                                   ('ros-root', 'ros-package-path', 'ros-ip', 'ros-host-name', 
                                    'ssh-port', 'user', 'password', 'default', 'timeout'))
            rosroot, ros_package_path, ros_ip, ros_host_name, \
                ssh_port, user, password, default, timeout = attrs

            if ros_host_name and ros_ip:
                raise XmlParseException("only one of 'ros-host-name' or 'ros-ip' may be set")
            ros_host_name = ros_host_name or ros_ip  #alias
            
            if not ros_package_path:
                # the default vale of ros_package_path is dependent on
                # ros-root. If user has set ros-root they must also
                # explicitly set ros-package-path. If neither is set,
                # they default to the environment.
                if rosroot:
                    ros_package_path = ''
                else:
                    ros_package_path = get_ros_package_path()
            if not rosroot:
                try:
                    rosroot = os.environ['ROS_ROOT']
                except KeyError, e:
                    pass
            ssh_port = int(ssh_port or '22')

            # check for default switch
            default = (default or 'false').lower()
            try:
                assignable = _assignable[default]
                is_default = _is_default[default]
            except KeyError, e:
                raise XmlParseException("Invalid value for 'attribute': %s"%default)

            # load env args
            env_args = []
            for t in [c for c in tag.childNodes if c.nodeType == DomNode.ELEMENT_NODE]:
                if t.tagName == 'env':
                    env_args.append(self._env_tag(t, context, ros_config))
                else:
                    ros_config.add_config_error("unrecognized '%s' tag in <%s> tag"%(t.tagName, tag.tagName))
            # cast timeout to float. make sure timeout wasn't an empty string or negative
            if timeout:
                try:
                    timeout = float(timeout)
                except ValueError:
                    raise XmlParseException("'timeout' be a number: [%s]"%timeout)
            elif timeout == '':
                raise XmlParseException("'timeout' cannot be empty")
            if timeout is not None and timeout <= 0.:
                raise XmlParseException("'timeout' be a positive number: [%s]"%timeout)                    
                    
            m = Machine(name, rosroot, ros_package_path, address, 
                        ros_ip=ros_host_name, ssh_port=ssh_port, user=user, password=password, 
                        assignable=assignable, env_args=env_args, timeout=timeout)
            return (m, is_default)
        except KeyError, e:
            raise XmlParseException("<machine> tag is missing required attribute: %s"%e)
        except SubstitutionException, e:
            raise XmlParseException(
                "%s. \n\nMachine xml is %s"%(e, tag.toxml()))
        except RLException, e:
            raise XmlParseException(
                "%s. \n\nMachine xml is %s"%(e, tag.toxml()))
        
    REMAP_ATTRS = ('from', 'to')
    def _remap_tag(self, tag, context, ros_config):
        try:
            self._check_attrs(tag, context, ros_config, XmlLoader.REMAP_ATTRS)
            return self.reqd_attrs(tag, context, XmlLoader.REMAP_ATTRS)
        except KeyError, e:
            raise XmlParseException("<remap> tag is missing required from/to attributes: %s"%tag.toxml())
        
    ENV_ATTRS = ('name', 'value')
    def _env_tag(self, tag, context, ros_config):
        try:
            self._check_attrs(tag, context, ros_config, XmlLoader.ENV_ATTRS)
            name, value = self.reqd_attrs(tag, context, XmlLoader.ENV_ATTRS)
            if not name:
                raise XmlParseException("<env> 'name' attribute must be non-empty: %s"%tag.toxml())                
            return name, value
        except KeyError, e:
            raise XmlParseException("<env> tag is missing required name/value attributes: %s"%tag.toxml())
    
    ## common processing routine for xml tags with NS and CLEAR_PARAMS attributes
    ## @param tag Node: DOM Node
    ## @param context _XmlContext: current namespace context 
    ## @param clear_params [str]: list of params to clear
    ## @param node_name str: name of node (for use when \a tagName == 'node')
    ## @return _XmlContext namespace context 
    def _ns_clear_params_attr(self, tagName, tag, context, ros_config, node_name=None):
        if tag.hasAttribute(NS):
            ns = self.resolve_args(tag.getAttribute(NS), context)
            if not ns:
                raise XmlParseException("<%s> tag has an empty '%s' attribute"%(tagName, NS))
        else:
            ns = None
        child_ns = context.child(ns)
        clear_p = self.resolve_args(tag.getAttribute(CLEAR_PARAMS), context)
        if clear_p:
            clear_p = clear_p.lower()
            if not clear_p in ['true', 'false']:
                raise XmlParseException("'clear_params' attribute must be set to true or false")
            if clear_p == 'true':
                if tagName == 'node':
                    if not node_name:
                        raise XmlParseException("<%s> tag must have a 'name' attribute to use '%s' attribute"%(tagName, CLEAR_PARAMS))
                    # use make_global_ns to give trailing slash in order to be consistent with XmlContext.ns
                    ros_config.add_clear_param(make_global_ns(ns_join(child_ns.ns, node_name)))
                else:
                    if not ns:
                        raise XmlParseException("'ns' attribute must be set in order to use 'clear_params'")                
                    ros_config.add_clear_param(child_ns.ns)
        return child_ns
        
    INCLUDE_ATTRS = ('file', NS, CLEAR_PARAMS)
    def _include_tag(self, tag, context, ros_config, default_machine, is_core, verbose):
        self._check_attrs(tag, context, ros_config, XmlLoader.INCLUDE_ATTRS)
        inc_filename = self.resolve_args(tag.attributes['file'].value, context)
        child_ns = self._ns_clear_params_attr(tag.tagName, tag, context, ros_config)
        child_ns.filename = inc_filename

        env_args = child_ns.env_args
        for t in [c for c in tag.childNodes if c.nodeType == DomNode.ELEMENT_NODE]:
            tagName = t.tagName.lower()
            if tagName == 'env':
                env_args.append(self._env_tag(t, child_ns, ros_config))
            else:
                print >> sys.stderr, \
                    "WARN: unrecognized '%s' tag in <%s> tag"%(t.tagName, tag.tagName)

        launch = self._parse_launch(inc_filename, verbose=verbose)
        default_machine = \
            self._recurse_load(ros_config, launch.childNodes, child_ns, \
                                   default_machine, is_core, verbose)
        if verbose:
            print "... done importing include file [%s]"%inc_filename
        return default_machine
                
    GROUP_ATTRS = (NS, CLEAR_PARAMS)
    ## @return default_machine Machine new default machine for current context
    def _recurse_load(self, ros_config, tags, context, default_machine, is_core, verbose):
        for tag in [t for t in tags if t.nodeType == DomNode.ELEMENT_NODE]:
            name = tag.tagName
            if name == 'group':
                self._check_attrs(tag, context, ros_config, XmlLoader.GROUP_ATTRS)
                try:
                    child_ns = self._ns_clear_params_attr(name, tag, context, ros_config)
                    default_machine = \
                                    self._recurse_load(ros_config, tag.childNodes, child_ns, \
                                                       default_machine, is_core, verbose)
                except KeyError, e:
                    raise XmlParseException("<ns> tag is missing required 'name' attribute")
            elif name == 'node':
                # clone the context so that nodes' env does not pollute global env
                n = self._node_tag(tag, context.child(''), ros_config, default_machine, verbose=verbose)
                ros_config.add_node(n, core=is_core, verbose=verbose)
            elif name == 'test':
                # clone the context so that nodes' env does not pollute global env                
                t = self._node_tag(tag, context.child(''), ros_config, default_machine, is_test=True, verbose=verbose)
                ros_config.add_test(t, verbose=verbose)
            elif name == 'param':
                self._param_tag(tag, context, ros_config, verbose=verbose)
            elif name == 'remap':
                context.add_remap(self._remap_tag(tag, context, ros_config))
            elif name == 'machine':
                (m, is_default) = self._machine_tag(tag, context, ros_config, verbose=verbose)
                if is_default:
                    default_machine = m
                ros_config.add_machine(m, verbose=verbose)
            elif name == 'rosparam':
                ros_config.add_executable(self._rosparam_tag(tag, context, ros_config))
            elif name == 'master':
                pass #handled non-recursively
            elif name == 'include':
                default_machine = self._include_tag(tag, context, ros_config, default_machine, is_core, verbose)
            elif name == 'env':
                context.env_args.append(self._env_tag(tag, context, ros_config))
            else:
                ros_config.add_config_error("WARNING: unrecognized tag"+tag.tagName)
        return default_machine

    ## subroutine of launch for loading XML DOM into config. Load_launch assumes that it is
    ## creating the root XmlContext.
    ## @param launch Node: DOM node of the root <launch> tag in the file
    ## @param ros_config ROSLaunchConfig: launch configuration to load XML file into
    ## @param is_core bool: if True, load file using ROS core rules
    def _load_launch(self, launch, ros_config, is_core=False, filename=None, verbose=True):
        # The <master> tag is special as we only only process a single
        # tag in the top-level file. We ignore master tags in
        # included files.
        master_tags = launch.getElementsByTagName('master')
        self.root_context = _XmlContext('', filename)
        if len(master_tags) > 1:
            raise XmlParseException("multiple <master> tags in top-level xml file not allowed")
        elif len(master_tags) == 1:
            ros_config.set_master(self._master_tag(master_tags[0], self.root_context, ros_config))
        self._recurse_load(ros_config, launch.childNodes, self.root_context, None, is_core, verbose)
        
    def _parse_launch(self, filename, verbose):
        try:
            if verbose:            
                print "... loading XML file [%s]"%filename
            root = parse(filename).getElementsByTagName('launch')
        except Exception, e:
            raise XmlParseException("Invalid roslaunch XML syntax: %s"%e)
        if len(root) != 1:
            raise XmlParseException("Invalid roslaunch XML syntax: no root <launch> tag")
        return root[0]
        
    ## load XML file into launch configuration
    ## @param filename str: XML config file to load
    ## @param ros_config ROSLaunchConfig: launch configuration to load XML file into
    ## @param core bool: if True, load file using ROS core rules
    def load(self, filename, ros_config, core=False, verbose=True):
        try:
            launch = self._parse_launch(filename, verbose)
            self._load_launch(launch, ros_config, is_core=core, filename=filename, verbose=verbose)
        except SubstitutionException, e:
            raise XmlParseException(str(e))

    ## load XML text into launch configuration
    ## @param xml_text str: XML configuration
    ## @param ros_config ROSLaunchConfig: launch configuration to load XML file into
    ## @param core bool: if True, load file using ROS core rules
    def load_string(self, xml_text, ros_config, core=False, verbose=True):
        try:
            if verbose:
                print "... loading XML"
            root = parseString(xml_text).getElementsByTagName('launch')
        except Exception, e:
            logging.getLogger('roslaunch').error("Invalid roslaunch XML syntax:\nstring[%s]\ntraceback[%s]"%(xml_text, traceback.format_exc()))
            raise XmlParseException("Invalid roslaunch XML syntax: %s"%e)
        
        if len(root) != 1:
            raise XmlParseException("Invalid roslaunch XML syntax: no root <launch> tag")
        self._load_launch(root[0], ros_config, core, filename='string', verbose=verbose)
