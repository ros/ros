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
# Revision $Id: network.py 3371 2009-01-13 21:53:02Z sfkwc $

import os
import socket
import string
import sys
import platform
import urlparse

import roslib.rosenv 

SIOCGIFCONF=0x8912
SIOCGIFADDR = 0x8915

try:
    import netifaces
    ## @internal
    _use_netifaces = True
except:
    # NOTE: in rare cases, I've seen Python fail to extract the egg
    # cache when launching multiple python nodes.  Thus, we do
    # except-all instead of except ImportError (kwc).
    ## @internal
    _use_netifaces = False

## convenience routine to handle parsing and validation of HTTP URL
## port due to the fact that Python only provides easy accessors in
## Python 2.5 and later. Validation checks that the protocol and host
## are set.
## @param url str: URL to parse
## @return str, int: hostname and port number in URL or 80
## (default).
## @throws ValueError if the url does not validate
def parse_http_host_and_port(url):
    # can't use p.port because that's only available in Python 2.5
    if not url:
        raise ValueError('not a valid URL')        
    p = urlparse.urlparse(url)
    if not p[0] or not p[1]: #protocol and host
        raise ValueError('not a valid URL')
    if ':' in p[1]:
        hostname, port = p[1].split(':')
        port = string.atoi(port)
    else: 
        hostname, port = p[1], 80
    return hostname, port
    
## @internal
def _is_unix_like_platform():
    #return platform.system() in ['Linux', 'Mac OS X', 'Darwin']
    return platform.system() in ['Linux']

## @return str: ROS_IP/ROS_HOSTNAME override or None
## @throws ValueError if ROS_IP/ROS_HOSTNAME/__ip/__hostname are invalidly specified
def get_address_override():
    # #998: check for command-line remappings first
    for arg in sys.argv:
        if arg.startswith('__hostname:=') or arg.startswith('__ip:='):
            try:
                _, val = arg.split(':=')
                return val
            except: #split didn't unpack properly
                raise ValueError("invalid ROS command-line remapping argument '%s'"%arg)

    # check ROS_HOSTNAME and ROS_IP environment variables, which are
    # aliases for each other
    if roslib.rosenv.ROS_HOSTNAME in os.environ:
        return os.environ[roslib.rosenv.ROS_HOSTNAME]
    elif roslib.rosenv.ROS_IP in os.environ:
        return os.environ[roslib.rosenv.ROS_IP]
    return None

## @return str: default local IP address (e.g. eth0). May be overriden by ROS_IP/ROS_HOSTNAME/__ip/__hostname
def get_local_address():
    override = get_address_override()
    if override:
        return override
    addrs = get_local_addresses()
    if len(addrs) == 1:
        return addrs[0]
    for addr in addrs:
        # pick first non 127/8 address
        if not addr.startswith('127.'):
            return addr
    else: # loopback 
        return '127.0.0.1'

## @return [str]: known local addresses. Not affected by ROS_IP/ROS_HOSTNAME
def get_local_addresses():
    if _use_netifaces:
        # #552: netifaces is a more robust package for looking up
        # #addresses on multiple platforms (OS X, Unix, Windows)
        local_addrs = []
        # see http://alastairs-place.net/netifaces/
        for i in netifaces.interfaces():
            try:
                local_addrs.extend([d['addr'] for d in netifaces.ifaddresses(i)[netifaces.AF_INET]])
            except KeyError: pass
        return local_addrs
    elif _is_unix_like_platform():
        # unix-only branch
        # adapted from code from Rosen Diankov (rdiankov@cs.cmu.edu)
        # and from ActiveState recipe

        import fcntl
        import struct
        import array

        ifsize = 32
        if platform.architecture()[0] == '64bit':
            ifsize = 40 # untested

        # 32 interfaces allowed, far more than ROS can sanely deal with
        max_bytes = 32 * ifsize
        buff = array.array('B', '\0' * max_bytes)
        # serialize the buffer length and address to ioctl
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)        
        info = fcntl.ioctl(sock.fileno(), SIOCGIFCONF,
                           struct.pack('iL', max_bytes, buff.buffer_info()[0]))
        retbytes = struct.unpack('iL', info)[0]
        buffstr = buff.tostring()
        return [socket.inet_ntoa(buffstr[i+20:i+24]) for i in range(0, retbytes, ifsize)]
    else:
        # cross-platform branch, can only resolve one address
        return [socket.gethostbyname(socket.gethostname())]


## @param address str: (optional) address to compare against
## @return address TCP/IP sockets should use for binding. This is
## generally 0.0.0.0, but if \a address or ROS_IP/ROS_HOSTNAME is set
## to localhost it will return 127.0.0.1
def get_bind_address(address=None):
    if address is None:
        address = get_address_override()
    if address and \
           (address == 'localhost' or address.startswith('127.')):
        #localhost or 127/8
        return '127.0.0.1' #loopback
    else:
        return '0.0.0.0'

def get_host_name():
    ## #528: semi-complicated logic for determining XML-RPC URI
    ## - if ROS_IP/ROS_HOSTNAME is set, use that address
    ## - if the hostname returns a non-localhost value, use that
    ## - use whatever network.get_local_address() returns
    hostname = get_address_override()
    if not hostname:
        try:
            hostname = socket.gethostname()
        except:
            pass
        if not hostname or hostname == 'localhost' or hostname.startswith('127.'):
            hostname = network.get_local_address()
    return hostname

## utility routine for determine the XMLRPC URI for local servers. this handles
## the search logic of checking ROS environment variables, the known hostname, and local
## interface IP addresses to determine the best possible URI.
## @param port int: port that server is running on
## @return str: XMLRPC URI    
def create_local_xmlrpc_uri(port):
    #TODO: merge logic in roslib.xmlrpc with this routine
    # in the future we may not want to be locked to http protocol nor root path
    return 'http://%s:%s/'%(get_host_name(), port)
    
