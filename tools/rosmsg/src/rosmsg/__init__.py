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

The code API of the rosmsg module is unstable. 
"""

from __future__ import print_function

import collections
import inspect
import os
import sys
import yaml

import rospkg
import genmsg

import roslib.message
import rosbag

from optparse import OptionParser

MODE_MSG = '.msg'
MODE_SRV = '.srv'

class ROSMsgException(Exception): pass
class RosMsgProtoException(Exception): pass
class RosMsgProtoArgsException(Exception): pass

# If flowtype chosen is default, instead use flow-style
# False except if meeting objects or arrays with more than
# this size of sub-elements
MAX_DEFAULT_NON_FLOW_ITEMS = 4

## copied from the web, recipe for ordered yaml output ######
def construct_ordered_mapping(self, node, deep=False):
    if not isinstance(node, yaml.MappingNode):
        raise yaml.constructor.ConstructorError(None, None,
                "expected a mapping node, but found %s" % node.id,
                node.start_mark)
    mapping = collections.OrderedDict()
    for key_node, value_node in node.value:
        key = self.construct_object(key_node, deep=deep)
        if not isinstance(key, collections.Hashable):
            raise yaml.constructor.ConstructorError("while constructing a mapping", node.start_mark,
                    "found unhashable key", key_node.start_mark)
        value = self.construct_object(value_node, deep=deep)
        mapping[key] = value
    return mapping

def construct_yaml_map_with_ordered_dict(self, node):
    data = collections.OrderedDict()
    yield data
    value = self.construct_mapping(node)
    data.update(value)

def represent_ordered_mapping(self, tag, mapping, flow_style=None):
    value = []
    node = yaml.MappingNode(tag, value, flow_style=flow_style)
    if self.alias_key is not None:
        self.represented_objects[self.alias_key] = node
    best_style = True
    if hasattr(mapping, 'items'):
        mapping = list(mapping.items())
    for item_key, item_value in mapping:
        node_key = self.represent_data(item_key)
        node_value = self.represent_data(item_value)
        if not (isinstance(node_key, yaml.ScalarNode) and not node_key.style):
            best_style = False
        if not (isinstance(node_value, yaml.ScalarNode) and not node_value.style):
            best_style = False
        value.append((node_key, node_value))
    if flow_style is None:
        if self.default_flow_style is not None:
            node.flow_style = self.default_flow_style
        else:
            node.flow_style = best_style
    return node

## end recipe for ordered yaml output ######


def get_array_type_instance(field_type, default_package = None):
    """
    returns a single instance of field_type, where field_type can be a
    message or ros primitive or an flexible size array.
    """
    field_type = field_type.strip().rstrip("[]")
    if field_type == "empty":
        return None
    if not "/" in field_type:
        # is either built-in, Header, or in same package
        # it seems built-in types get a priority
        if field_type in ['byte', 'int8', 'int16', 'int32', 'int64',\
                          'char', 'uint8', 'uint16', 'uint32', 'uint64']:
            return 0
        elif field_type in ['float32', 'float64']:
            return 0
        elif field_type in ['string']:
            return ""
        elif field_type == 'bool':
            return False
        elif field_type == 'time':
            field_type = "std_msgs/Time"
        elif field_type == 'duration':
            field_type = "std_msgs/Duration"
        elif field_type == 'Header':
            field_type = "std_msgs/Header"
        else:
            if default_package is None:
                return None
            field_type = default_package + "/" + field_type
    msg_class = roslib.message.get_message_class(field_type)
    if (msg_class == None):
        # not important enough to raise exception?
        return None
    instance = msg_class()
    return instance

def get_yaml_for_msg(msg, prefix='', time_offset=None, current_time=None, field_filter=None, flow_style_ = None, fill_arrays_ = False):
    """
    Builds a YAML string of message.
    @param msg: A object, dict or array
    @param flow_style_: if True, produces one line with brackets, if false uses multiple lines with indentation, if None uses both using heuristics
    @param prefix: prefixes all lines with this string
    @param fill_arrays_: if True, for all flexible size arrays an element will be generated
    @param current_time: currently not used. Only provided for API compatibility. current_time passes in the current time with respect to the message.
    @type  current_time: Time
    @param field_filter: filter the fields that are strified for Messages.
    @type  field_filter: fn(Message)->iter(str)
    @type  flow_style_: bool
    @return: a string
    """
    def object_representer(dumper, obj):
        ndict = collections.OrderedDict()
        index = 0
        # allow caller to select which fields of message are strified
        if field_filter != None:
            fields = list(field_filter(obj))
        else:
            fields = obj.__slots__
        for key in fields:
            if not key.startswith('_'):
                val = getattr(obj, key)
                if type(val) == list and len(val) > MAX_DEFAULT_NON_FLOW_ITEMS:
                     dumper.default_flow_style = flow_style_
                if time_offset is not None and isinstance(val, Time):
                    ndict[key] = val-time_offset
                # create initial array element (e.g. for code completion)
                elif fill_arrays_ == True and val == []:
                    message_type = obj._slot_types[index]
                    if (obj._type != None) and "/" in obj._type:
                        def_pack = obj._type.split("/")[0]
                        instance = get_array_type_instance(message_type, default_package = def_pack)
                    if instance == None:
                        # not important enough to raise exception?
                        ndict[key] = val
                    else:
                        ndict[key] = [instance]
                elif not inspect.ismethod(val) and not inspect.isfunction(val):
                    ndict[key] = val
            index += 1
        # as a hack, we improve the heuristics of pyyaml and say with less than 5 objects, no need for brackets
        if len(ndict) > MAX_DEFAULT_NON_FLOW_ITEMS:
            dumper.default_flow_style = flow_style_
        return dumper.represent_dict(ndict)
    yaml.representer.SafeRepresenter.add_representer(None, object_representer)

     # we force False over None here and set the style in dumper, to
     # avoid unecessary outer brackets pyyaml chooses e.g. to
     # represent msg Int32 as "{data: 0}"
    initial_flow_style = False
    if flow_style_ == True:
        initial_flow_style = True

    # need to set default flow style True for bash prototype
    # means will generate one line with [] and {} brackets
    # bash needs bracket notation for rostopic pub
    txt = yaml.safe_dump(msg,
                         # None, True, False (None chooses a compromise)
                         default_flow_style = initial_flow_style,
                         # Can be None, '', '\'', '"', '|', '>'.
                         default_style = '',
                         #indent=2, #>=2, indentation depth
                         #line_break=?,
                         #allow_unicode=?,
                         #if true writes plenty of tags
                         #canonical = False,
                         #version={}?,
                         #width=40,
                         #encoding=?,
                         #tags={}?,
                         # when True, produces --- at start
                         #explicit_start=False,
                         # when True, produces ... at end
                         #explicit_end=False
                         )
    if prefix != None and prefix != '':
        result = prefix + ("\n"+prefix).join(txt.splitlines())
    else:
        result = txt.rstrip('\n')
    return result


def create_names_filter(names):
    """
    returns a function to use as filter that returns all objects slots except those with names in list.
    """
    return lambda obj : filter(lambda slotname : not slotname in names, obj.__slots__)


def init_rosmsg_proto():
    if "OrderedDict" in collections.__dict__:
        yaml.constructor.BaseConstructor.construct_mapping = construct_ordered_mapping
        yaml.constructor.Constructor.add_constructor(
            'tag:yaml.org,2002:map',
            construct_yaml_map_with_ordered_dict)

        yaml.representer.BaseRepresenter.represent_mapping = represent_ordered_mapping
        yaml.representer.Representer.add_representer(collections.OrderedDict,
                                                     yaml.representer.SafeRepresenter.represent_dict)
    
def rosmsg_cmd_prototype(args):
    init_rosmsg_proto()
    parser = OptionParser(usage="usage: rosmsgproto msg/srv [options]",
                          description="Produces output or a msg or service request, intended for tab completion support.")
    parser.add_option("-f","--flow_style",
                      dest="flow_style", type="int", default=None, action="store",
                      help="if 1 always use brackets, if 0 never use brackets. Default is a heuristic mix.")
    parser.add_option("-e","--empty-arrays",
                      dest="empty_arrays", default=False, action="store_true",
                      help="if true flexible size arrays are not filled with default instance")
    parser.add_option("-s","--silent",
                      dest="silent", default=False, action="store_true",
                      help="if true supresses all error messages")
    parser.add_option("-p", "--prefix", metavar="prefix", default="",
                      help="prefix to print before each line, can be used for indent")
    parser.add_option("-H","--no-hyphens",
                      dest="no_hyphens", default="", action="store_true",
                      help="if true output has no outer hyphens")
    parser.add_option("-x", "--exclude-slots", metavar="exclude_slots", default="",
                      help="comma separated list of slot names not to print")

    options, args = parser.parse_args(args)

    try:
        if len(args) < 2:
            raise RosMsgProtoArgsException("Insufficient arguments")
        mode = ".%s"%args[0]
        message_type=args[1]
        field_filter = None
        if options.exclude_slots != None and options.exclude_slots.strip() != "":
            field_filter = create_names_filter(options.exclude_slots.split(','))
    
        # possible extentions: options for
        # - target language
        # - initial values for standard types
        # - get partial message (subtree)

        # try to catch the user specifying code-style types and error
        if '::' in message_type:
            if not options.silent:
                parser.error("rosmsgproto does not understand C++-style namespaces (i.e. '::').\nPlease refer to msg/srv types as 'package_name/Type'.")
        elif '.' in message_type:
            if not options.silent:
                parser.error("invalid message type '%s'.\nPlease refer to msg/srv types as 'package_name/Type'." % message_type)
        if not '/' in message_type:
            # if only one such msg or srv exists, use it
            results = []
            for found in rosmsg_search(rospkg.RosPack(), mode, message_type):
                results.append(found)
            if len(results) > 1:
                raise RosMsgProtoException("Ambiguous message name %s"%message_type)
            elif len(results) < 1:
                raise RosMsgProtoException("Unknown message name %s"%message_type)
            else:
                message_type=results[0]
    
        if mode == MODE_SRV:
            msg_class = roslib.message.get_service_class(message_type)
            if (msg_class == None):
                raise RosMsgProtoException("Unknown service class: %s"%message_type)
            instance = msg_class()._request_class()
        elif mode == MODE_MSG:
            msg_class = roslib.message.get_message_class(message_type)
            if (msg_class == None):
                raise RosMsgProtoException("Unknown message class: %s"%message_type)
            instance = msg_class()
        else:
            raise RosMsgProtoException("Invalid mode: %s"%mode)
        txt = get_yaml_for_msg(instance,
                               prefix = options.prefix,
                               flow_style_ = options.flow_style,
                               fill_arrays_ = not options.empty_arrays,
                               field_filter = field_filter)

        if options.no_hyphens == True:
            return txt
        else:
            return '"' + txt + '"'

    except KeyError as e:
        if not options.silent:
            sys.stderr.write("Unknown message type: %s"%e, file=sys.stderr)
            sys.exit(getattr(os, 'EX_USAGE', 1))
    # except rospkg.InvalidROSPkgException as e:
    #     if not options.silent:
    #         print(file=sys.stderr, "Invalid package: '%s'"%e)
    #         sys.exit(getattr(os, 'EX_USAGE', 1))
    except ValueError, e:
        if not options.silent:
            sys.stderr.write("Invalid type: '%s'"%e)
            sys.exit(getattr(os, 'EX_USAGE', 1))
    except RosMsgProtoException as e:
        if not options.silent:
            sys.stderr.write(str(e))
            sys.exit(1)
    except RosMsgProtoArgsException as e:
        if not options.silent:
            sys.stderr.write("%s"%e)
            sys.exit(getattr(os, 'EX_USAGE', 1))
    except KeyboardInterrupt:
        pass

#### Start of rosmsg ####

try:
    from cStringIO import StringIO  # Python 2.x
except ImportError:
    from io import StringIO  # Python 3.x
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

def get_srv_text(type_, raw=False, rospack=None):
    """
    Get .srv file for type_ as text
    :param type_: service type, ``str``
    :param raw: if True, include comments and whitespace (default False), ``bool``
    :returns: text of .srv file, ``str``
    @raise ROSMsgException: if type_ is unknown
    """
    if rospack is None:
        rospack = rospkg.RosPack()
    srv_search_path = {}
    msg_search_path = {}
    for p in rospack.list():
        path = rospack.get_path(p)
        msg_search_path[p] = [os.path.join(path, 'msg')]
        srv_search_path[p] = [os.path.join(path, 'srv')]
        
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

def get_msg_text(type_, raw=False, rospack=None):
    """
    Get .msg file for type_ as text
    :param type_: message type, ``str``
    :param raw: if True, include comments and whitespace (default False), ``bool``
    :returns: text of .msg file, ``str``
    :raises :exc:`ROSMsgException` If type_ is unknown
    """
    if rospack is None:
        rospack = rospkg.RosPack()
    search_path = {}
    for p in rospack.list():
        search_path[p] = [os.path.join(rospack.get_path(p), 'msg')]

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

def rosmsg_debug(rospack, mode, type_, raw=False):
    """
    Prints contents of msg/srv file
    :param mode: MODE_MSG or MODE_SRV, ``str``
    """
    if mode == MODE_SRV:
        print(get_srv_text(type_, raw=raw, rospack=rospack))
    elif mode == MODE_MSG:
        print(get_msg_text(type_, raw=raw, rospack=rospack))
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
        subdir = 'msg'
    elif mode == MODE_SRV:
        subdir = 'srv'
    else:
        raise ValueError('Unknown mode for list_types: %s'%mode)

    path = os.path.join(rospack.get_path(package), subdir)
    
    return [genmsg.resource_name(package, t) for t in _list_types(path, subdir, mode)]
    
def _msg_filter(ext):
    def mfilter(f):
        """
        Predicate for filtering directory list. matches message files
        :param f: filename, ``str``
        """
        return os.path.isfile(f) and f.endswith(ext)
    return mfilter

def _list_types(path, subdir, ext):
    """
    List all messages in the specified package
    :param package str: name of package to search
    :param include_depends bool: if True, will also list messages in package dependencies
    :returns [str]: message type names
    """
    types = _list_resources(path, _msg_filter(ext))
    result = [x[:-len(ext)] for x in types]
    result.sort()
    return result

def _list_resources(path, rfilter=os.path.isfile):
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

def iterate_packages(rospack, mode):
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

    pkgs = rospack.list()
    for p in pkgs:
        d = os.path.join(rospack.get_path(p), subdir)
        if os.path.isdir(d):
            yield p, d
    
def rosmsg_search(rospack, mode, base_type):
    """
    Iterator for all packages that contain a message matching base_type

    :param base_type: message base type to match, e.g. 'String' would match std_msgs/String, ``str``
    """
    for p, path in iterate_packages(rospack, mode):
        if os.path.isfile(os.path.join(path, "%s%s"%(base_type, mode))):
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
        rospack = rospkg.RosPack()
        if '/' in arg: #package specified
            rosmsg_debug(rospack, mode, arg, options.raw)
        else:
            for found in rosmsg_search(rospack, mode, arg):
                print("[%s]:"%found)
                rosmsg_debug(rospack, mode, found, options.raw)

def rosmsg_md5(mode, type_):
    try:
        if mode == MODE_MSG:
            msg_class = roslib.message.get_message_class(type_)
        else:
            msg_class = roslib.message.get_service_class(type_)            
    except ImportError:
        raise IOError("cannot load [%s]"%(type_))
    if msg_class is not None:
        return msg_class._md5sum
    else:
        raise IOError("cannot load [%s]"%(type_))        
    
def rosmsg_cmd_md5(mode, full):
    parser = OptionParser(usage="usage: ros%s md5 <%s>"%(mode[1:], full))
    options, arg = _stdin_arg(parser, full)

    if '/' in arg: #package specified
        try:
            md5 = rosmsg_md5(mode, arg)
            print(md5)
        except IOError:
            print("Cannot locate [%s]"%arg, file=sys.stderr)
    else:
        rospack = rospkg.RosPack()
        matches = [m for m in rosmsg_search(rospack, mode, arg)]
        for found in matches:
            try:
                md5 = rosmsg_md5(mode, found)
                print("[%s]: %s"%(found, md5))
            except IOError:
                print("Cannot locate [%s]"%found, file=sys.stderr)
        if not matches:
            print("No messages matching the name [%s]"%arg, file=sys.stderr)
                
def rosmsg_cmd_package(mode, full):
    parser = OptionParser(usage="usage: ros%s package <package>"%mode[1:])
    parser.add_option("-s",
                      dest="single_line", default=False,action="store_true",
                      help="list all msgs on a single line")
    options, arg = _stdin_arg(parser, full)
    joinstring='\n'
    if options.single_line:
        joinstring=' '
    print(joinstring.join(list_types(arg, mode=mode)))
    
def rosmsg_cmd_packages(mode, full, argv=None):
    if argv is None:
        argv = sys.argv[1:]
    parser = OptionParser(usage="usage: ros%s packages"%mode[1:])
    parser.add_option("-s",
                      dest="single_line", default=False,action="store_true",
                      help="list all packages on a single line")
    options, args = parser.parse_args(argv[1:])
    rospack = rospkg.RosPack()
    joinstring='\n'
    if options.single_line:
        joinstring=' '
    p1 = [p for p, _ in iterate_packages(rospack, mode)]
    p1.sort()
    print(joinstring.join(p1))
    
def rosmsg_cmd_list(mode, full, argv=None):
    if argv is None:
        argv = sys.argv[1:]
    parser = OptionParser(usage="usage: ros%s list"%mode[1:])
    options, args = parser.parse_args(argv[1:])
    if mode == MODE_MSG:
        subdir = 'msg'
    elif mode == MODE_SRV:
        subdir = 'srv'
    else:
        raise ValueError('Unknown mode for iterate_packages: %s'%mode)
    rospack = rospkg.RosPack()
    packs = sorted([x for x in iterate_packages(rospack, mode)])
    for (p, direc) in packs:
        for file in _list_types(direc, subdir, mode):
            print( "%s/%s"%(p, file))
        

def fullusage(cmd):
    """
    :param cmd: command name, ``str``
    :returns: usage text for cmd, ``str``
    """
    return """rosmsg is a command-line tool for displaying information about ROS Message types.

Commands:
\t%(cmd)s show\tShow message description
\t%(cmd)s list\tList all messages
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
        if command == 'show':
            rosmsg_cmd_show(ext, full)
        elif command == 'package':
            rosmsg_cmd_package(ext, full)
        elif command == 'packages':
            rosmsg_cmd_packages(ext, full)
        elif command == 'list':
            rosmsg_cmd_list(ext, full)
        elif command == 'md5':
            rosmsg_cmd_md5(ext, full)
        elif command == '--help':
            print(fullusage('ros'+mode[1:]))
            sys.exit(0)
        else:
            print(fullusage('ros'+mode[1:]))
            sys.exit(getattr(os, 'EX_USAGE', 1))
    except KeyError as e:
        print("Unknown message type: %s"%e, file=sys.stderr)
        sys.exit(getattr(os, 'EX_USAGE', 1))
    except rospkg.ResourceNotFound as e:
        print("Invalid package: %s"%e, file=sys.stderr)
        sys.exit(getattr(os, 'EX_USAGE', 1))
    except ValueError as e:
        print("Invalid type: '%s'"%e, file=sys.stderr)
        sys.exit(getattr(os, 'EX_USAGE', 1))
    except ROSMsgException as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)        
    except KeyboardInterrupt:
        pass
