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
Roslaunch XML file parser.
"""

from __future__ import with_statement

import itertools
import os
import sys

from xml.dom.minidom import parse, parseString
from xml.dom import Node as DomNode

from roslib.names import make_global_ns, ns_join, is_global, is_private, PRIV_NAME
import roslib.substitution_args

from roslaunch.core import *
from roslaunch.loader import Loader, LoaderContext, command_line_param

# use in our namespace
SubstitutionException = roslib.substitution_args.SubstitutionException

NS='ns'
CLEAR_PARAMS='clear_params'

def _get_text(tag):
    buff = ''
    for t in tag.childNodes:
        if t.nodeType in [t.TEXT_NODE, t.CDATA_SECTION_NODE]:
            buff += t.data
    return buff

# This code has gotten a bit crufty as roslaunch has grown far beyond
# its original spec. It needs to be far more generic than it is in
# order to not replicate bugs in multiple places.

class XmlParseException(RLException):
    """Error with the XML syntax (e.g. invalid attribute/value combinations)"""
    pass
class XmlLoadException(RLException):
    """Error loading XML data as specified (e.g. cannot find included files, etc...)"""
    pass

def _bool_attr(v, default, label):
    """
    Validate boolean xml attribute. 
    @param v: parameter value or None if no value provided
    @type v: any
    @param default: default value
    @type  default: bool
    @param label: parameter name/label
    @type  label: str
    @return: boolean value for attribute
    @rtype: bool
    @raise XmlParseException: if v is not in correct range or is empty.
    """
    if v is None:
        return default
    if v.lower() == 'true':
        return True
    elif v.lower() == 'false':
        return False
    elif not v:
        raise XmlParseException("bool value for %s must be non-empty"%(label))
    else:
        raise XmlParseException("invalid bool value for %s: %s"%(label, v))

def _enum_attr(v, enums, label):
    """
    @param v: parameter value
    @type v: any
    @param enums: valid values for parameter
    @type enums: [any]
    @param label: parameter name/label
    @type  label: str
    @return: value for attribute
    @raise XmlParseException: if v is not in correct range
    """
    if not v in enums:
        raise XmlParseException("'%s' attribute must be one of: %s"%(label, ', '.join(enums)))
    return v

# maps machine 'default' attribute to Machine default property
_is_default = {'true': True, 'false': False, 'never': False }
# maps machine 'default' attribute to Machine assignable property
_assignable = {'true': True, 'false': True, 'never': False }

# NOTE: code is currently in a semi-refactored state. I'm slowly
# migrating common routines into the Loader class in the hopes it will
# make it easier to write alternate loaders and also test.
class XmlLoader(Loader):
    """
    Parser for roslaunch XML format. Loads parsed representation into ROSConfig model.
    """

    def __init__(self, resolve_anon=True):
        """
        @param resolve_anon: If True (default), will resolve $(anon foo). If
        false, will leave these args as-is.
        @type  resolve_anon: bool
        """        
        # store the root XmlContext so that outside code can access it
        self.root_context = None
        self.resolve_anon = resolve_anon

    def resolve_args(self, args, context):
        """
        Wrapper around roslib.substitution_args.resolve_args to set common parameters
        """
        # resolve_args gets called a lot, so we optimize by testing for dollar sign before resolving
        if args and '$' in args:
            return roslib.substitution_args.resolve_args(args, context=context.resolve_dict, resolve_anon=self.resolve_anon)
        else:
            return args

    def opt_attrs(self, tag, context, attrs):
        """
        Helper routine for fetching and resolving optional tag attributes
        @param tag DOM tag
        @param context LoaderContext
        @param attrs (str): list of attributes to resolve
        """            
        def tag_value(tag, a):
            if tag.hasAttribute(a):
                # getAttribute returns empty string for non-existent
                # attributes, which makes it impossible to distinguish
                # with actual empty values
                return tag.getAttribute(a)
            else:
                return None
        return [self.resolve_args(tag_value(tag,a), context) for a in attrs]

    def reqd_attrs(self, tag, context, attrs):
        """
        Helper routine for fetching and resolving required tag attributes
        @param tag: DOM tag
        @param attrs: list of attributes to resolve        
        @type  attrs: (str)
        @raise KeyError: if required attribute is missing
        """            
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
        try:
            return self.create_master(*self.opt_attrs(tag, context, ('type', 'uri', 'auto')))
        except ValueError, e:
            raise XmlParseException("invalid <master> tag: %s"%str(e))

    # 'ns' attribute is now deprecated and is an alias for
    # 'param'. 'param' is required if the value is a non-dictionary
    # type
    ROSPARAM_OPT_ATTRS = ('command', 'ns', 'file', 'param')
    def _rosparam_tag(self, tag, context, ros_config, verbose=True):
        try:
            cmd, ns, file, param = self.opt_attrs(tag, context, (XmlLoader.ROSPARAM_OPT_ATTRS))
            # ns atribute is a bit out-moded and is only left in for backwards compatibility
            param = ns_join(ns or '', param or '')
            
            # load is the default command            
            cmd = cmd or 'load'
            
            self.load_rosparam(context, ros_config, cmd, param, file, _get_text(tag), verbose=verbose)

        except ValueError, e:
            raise XmlLoadException("error loading <rosparam> tag: \n\t"+str(e)+"\nXML is %s"%tag.toxml())

    PARAM_ATTRS = ('name', 'value', 'type', 'value', 'textfile', 'binfile', 'command')
    def _param_tag(self, tag, context, ros_config, force_local=False, verbose=True):
        """
        @param force_local: if True, param must be added to context instead of ros_config
        @type  force_local: bool
        """
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
            value = self.param_value(verbose, name, ptype, *vals)

            # TODO: this first branch should be unnecessary once we remove support for command-line param
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
        except Exception, e:
            raise XmlParseException(
                "Invalid <param> tag: %s. \n\nParam xml is %s"%(e, tag.toxml()))


    def _test_attrs(self, tag, context):
        """
        Process attributes of <test> tag not present in <node>
        @return: test_name, time_limit
        @rtype: str, int
        """
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
                retry = int(retry)
            except ValueError:
                raise XmlParseException("'retry' must be a number: [%s]"%retry)

        return test_name, time_limit, retry
        
    NODE_ATTRS = ['pkg', 'type', 'machine', 'name', 'args', 'output', 'respawn', 'cwd', NS, CLEAR_PARAMS, 'launch-prefix', 'required']
    TEST_ATTRS = NODE_ATTRS + ['test-name','time-limit', 'retry']
    
    def _node_tag(self, tag, context, ros_config, default_machine, is_test=False, verbose=True):
        """
        Process XML <node> or <test> tag
        @param tag: DOM node
        @type  tag: Node
        @param context: namespace context
        @type  context: L{LoaderContext}
        @param params: ROS parameter list
        @type  params: [L{Param}]
        @param clear_params: list of ROS parameter names to clear before setting parameters
        @type  clear_params: [str]
        @param default_machine: default machine to assign to node
        @type  default_machine: str
        @param is_test: if set, will load as L{Test} object instead of L{Node} object
        @type  is_test: bool
        """
        try:
            if is_test:
                self._check_attrs(tag, context, ros_config, XmlLoader.TEST_ATTRS)
                (name,) = self.opt_attrs(tag, context, ('name',)) 
            else:
                self._check_attrs(tag, context, ros_config, XmlLoader.NODE_ATTRS)
                (name,) = self.reqd_attrs(tag, context, ('name',)) 
                
            # required attributes
            pkg, node_type = self.reqd_attrs(tag, context, ('pkg', 'type'))

            
            if not len(pkg.strip()):
                raise XmlParseException("<node> 'pkg' must be non-empty")
            if not len(node_type.strip()):
                raise XmlParseException("<node> 'type' must be non-empty")
            
            # optional attributes
            machine, args, output, respawn, cwd, launch_prefix, required = \
                     self.opt_attrs(tag, context, ('machine', 'args', 'output', 'respawn', 'cwd', 'launch-prefix', 'required'))
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
            # validate respawn, required, output and cwd
            # TODO: move more attribute validation into Node tag itself
            output = _enum_attr(output or 'log', ['log', 'screen'], 'output')
            cwd = _enum_attr(cwd or 'ros-root', ['ros-root', 'node'], 'cwd')
            # - required and respawn both have no meaning to Tests and aren't passed on
            required, respawn = [_bool_attr(*rr) for rr in ((required, False, 'required'),\
                                                                (respawn, False, 'respawn'))]

            # each node gets its own copy of <remap> arguments, which
            # it inherits from its parent
            remap_context = context.child('')

            param_ns = child_ns.child(name)
            
            # nodes can have individual env args set in addition to
            # the ROS-specific ones.  
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
                    self._rosparam_tag(t, param_ns, ros_config, verbose=verbose)
                elif tagName == 'env':
                    self._env_tag(t, context, ros_config)
                else:
                    ros_config.add_config_error("WARN: unrecognized '%s' tag in <node> tag. Node xml is %s"%(t.tagName, tag.toxml()))

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
                    args = args + " " + command_line_param(pkey, p.value)
                else:
                    args = command_line_param(pkey, p.value)
                    
            if not is_test:
                return Node(pkg, node_type, name=name, namespace=child_ns.ns, machine_name=machine, 
                            args=args, respawn=respawn, 
                            remap_args=remap_context.remap_args(), env_args=context.env_args,
                            output=output, cwd=cwd, launch_prefix=launch_prefix,
                            required=required, filename=context.filename)
            else:
                test_name, time_limit, retry = self._test_attrs(tag, context)
                if not name:
                    name = test_name
                return Test(test_name, pkg, node_type, name=name, namespace=child_ns.ns, 
                            machine_name=machine, args=args,
                            remap_args=remap_context.remap_args(), env_args=context.env_args,
                            time_limit=time_limit, cwd=cwd, launch_prefix=launch_prefix,
                            retry=retry, filename=context.filename)
        except KeyError, e:
            raise XmlParseException(
                "<%s> tag is missing required attribute: %s. Node xml is %s"%(tag.tagName, e, tag.toxml()))
        except XmlParseException, e:
            raise XmlParseException(
                "Invalid <node> tag: %s. \n\nNode xml is %s"%(e, tag.toxml()))
        except ValueError, e:
            raise XmlParseException(
                "Invalid <node> tag: %s. \n\nNode xml is %s"%(e, tag.toxml()))

    MACHINE_ATTRS = ('name', 'address', 'ros-root', 'ros-package-path', 'ros-ip', 'ros-host-name', 
                     'ssh-port', 'user', 'password', 'default', 'timeout')
    def _machine_tag(self, tag, context, ros_config, verbose=True):
        try:
            # clone context as <machine> tag sets up its own env args
            context = context.child(None)
            
            self._check_attrs(tag, context, ros_config, XmlLoader.MACHINE_ATTRS)
            # required attributes
            name, address = self.reqd_attrs(tag, context, ('name', 'address'))
            
            # optional attributes
            attrs = self.opt_attrs(tag, context,
                                   ('ros-root', 'ros-package-path', 'ros-ip', 'ros-host-name', 
                                    'ssh-port', 'user', 'password', 'default', 'timeout'))
            rosroot, ros_package_path, ros_ip, ros_host_name, \
                ssh_port, user, password, default, timeout = attrs

            # DEPRECATED: remove in ROS 0.11 if possible
            if ros_host_name and ros_ip:
                raise XmlParseException("only one of 'ros-host-name' or 'ros-ip' may be set")
            if ros_ip:
                ros_config.add_config_error("WARN: ros-ip in <machine> tags is now deprecated. Use <env> tags instead")
            if ros_host_name:
                ros_config.add_config_error("WARN: ros-host-name in <machine> tags is now deprecated. Use <env> tags instead")

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
            for t in [c for c in tag.childNodes if c.nodeType == DomNode.ELEMENT_NODE]:
                if t.tagName == 'env':
                    self._env_tag(t, context, ros_config)
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

            #TODO: change Machine representation to use an environment
            # dictionary instead of the umpteen ROS environment
            # variable settings.
            m = Machine(name, rosroot, ros_package_path, address, 
                        ros_ip=ros_host_name, ssh_port=ssh_port, user=user, password=password, 
                        assignable=assignable, env_args=context.env_args, timeout=timeout)
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
            self.load_env(context, ros_config, *self.reqd_attrs(tag, context, XmlLoader.ENV_ATTRS))
        except ValueError, e:
            raise XmlParseException("Invalid <env> tag: %s. \nXML is %s"%(str(e), tag.toxml()))
        except KeyError, e:
            raise XmlParseException("<env> tag is missing required name/value attributes: %s"%tag.toxml())
    
    def _ns_clear_params_attr(self, tagName, tag, context, ros_config, node_name=None):
        """
        Common processing routine for xml tags with NS and CLEAR_PARAMS attributes
        
        @param tag: DOM Node
        @type  tag: Node
        @param context: current namespace context 
        @type  context: LoaderContext
        @param clear_params: list of params to clear
        @type  clear_params: [str]
        @param node_name: name of node (for use when tagName == 'node')
        @type  node_name: str
        @return: loader context 
        @rtype:  L{LoaderContext}
        """
        if tag.hasAttribute(NS):
            ns = self.resolve_args(tag.getAttribute(NS), context)
            if not ns:
                raise XmlParseException("<%s> tag has an empty '%s' attribute"%(tagName, NS))
        else:
            ns = None
        child_ns = context.child(ns)
        clear_p = self.resolve_args(tag.getAttribute(CLEAR_PARAMS), context)
        if clear_p:
            clear_p = _bool_attr(clear_p, False, 'clear_params')
            if clear_p:
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
        
    def _launch_tag(self, tag, ros_config, filename=None):
        # #2499
        deprecated = tag.getAttribute('deprecated')
        if deprecated:
            if filename:
                ros_config.add_config_error("[%s] DEPRECATED: %s"%(filename, deprecated))
            else:
                ros_config.add_config_error("Deprecation Warning: "+deprecated)

    INCLUDE_ATTRS = ('file', NS, CLEAR_PARAMS)
    def _include_tag(self, tag, context, ros_config, default_machine, is_core, verbose):
        self._check_attrs(tag, context, ros_config, XmlLoader.INCLUDE_ATTRS)
        inc_filename = self.resolve_args(tag.attributes['file'].value, context)
        child_ns = self._ns_clear_params_attr(tag.tagName, tag, context, ros_config)
        child_ns.filename = inc_filename

        for t in [c for c in tag.childNodes if c.nodeType == DomNode.ELEMENT_NODE]:
            tagName = t.tagName.lower()
            if tagName == 'env':
                self._env_tag(t, child_ns, ros_config)
            else:
                print >> sys.stderr, \
                    "WARN: unrecognized '%s' tag in <%s> tag"%(t.tagName, tag.tagName)

        try:
            launch = self._parse_launch(inc_filename, verbose=verbose)
            self._launch_tag(launch, ros_config, filename=inc_filename)
            default_machine = \
                self._recurse_load(ros_config, launch.childNodes, child_ns, \
                                       default_machine, is_core, verbose)
        except XmlParseException, e:
            raise XmlParseException("while processing %s:\n%s"%(inc_filename, str(e)))
        if verbose:
            print "... done importing include file [%s]"%inc_filename
        return default_machine
                
    GROUP_ATTRS = (NS, CLEAR_PARAMS)
    def _recurse_load(self, ros_config, tags, context, default_machine, is_core, verbose):
        """
        @return: new default machine for current context
        @rtype: L{Machine}
        """
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
                self._rosparam_tag(tag, context, ros_config, verbose=verbose)
            elif name == 'master':
                pass #handled non-recursively
            elif name == 'include':
                default_machine = self._include_tag(tag, context, ros_config, default_machine, is_core, verbose)
            elif name == 'env':
                self._env_tag(tag, context, ros_config)
            else:
                ros_config.add_config_error("unrecognized tag "+tag.tagName)
        return default_machine

    def _load_launch(self, launch, ros_config, is_core=False, filename=None, verbose=True):
        """
        subroutine of launch for loading XML DOM into config. Load_launch assumes that it is
        creating the root XmlContext.
        @param launch: DOM node of the root <launch> tag in the file
        @type  launch: L{Node}
        @param ros_config: launch configuration to load XML file into
        @type  ros_config: L{ROSLaunchConfig}
        @param is_core: if True, load file using ROS core rules
        @type  is_core: bool
        """        
        # The <master> tag is special as we only only process a single
        # tag in the top-level file. We ignore master tags in
        # included files.

        self._launch_tag(launch, ros_config, filename)
        master_tags = launch.getElementsByTagName('master')
        self.root_context = LoaderContext('', filename)
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
        
    def load(self, filename, ros_config, core=False, verbose=True):
        """
        load XML file into launch configuration
        @param filename: XML config file to load
        @type  filename: str
        @param ros_config: launch configuration to load XML file into
        @type  ros_config: L{ROSLaunchConfig}
        @param core: if True, load file using ROS core rules
        @type  core: bool
        """
        try:
            launch = self._parse_launch(filename, verbose)
            self._load_launch(launch, ros_config, is_core=core, filename=filename, verbose=verbose)
        except SubstitutionException, e:
            raise XmlParseException(str(e))

    def load_string(self, xml_text, ros_config, core=False, verbose=True):
        """
        Load XML text into launch configuration
        @param xml_text: XML configuration
        @type  xml_text: str
        @param ros_config: launch configuration to load XML file into
        @type  ros_config: L{ROSLaunchConfig}
        @param core: if True, load file using ROS core rules
        @type  core: bool
        """
        try:
            if verbose:
                print "... loading XML"
            root = parseString(xml_text).getElementsByTagName('launch')
        except Exception, e:
            import traceback
            import logging
            logging.getLogger('roslaunch').error("Invalid roslaunch XML syntax:\nstring[%s]\ntraceback[%s]"%(xml_text, traceback.format_exc()))
            raise XmlParseException("Invalid roslaunch XML syntax: %s"%e)
        
        if len(root) != 1:
            raise XmlParseException("Invalid roslaunch XML syntax: no root <launch> tag")
        self._load_launch(root[0], ros_config, core, filename='string', verbose=verbose)
