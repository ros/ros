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
Network APIs for ROS-based systems, including IP address and ROS
TCP header libraries. Because ROS-based runtimes must respect the
ROS_IP and ROS_HOSTNAME environment variables, ROS-specific APIs
are necessary for correctly retrieving local IP address
information.

roslib.network supports the netifaces library as an optional
add-on. netifaces improves IP address configuration detection.
"""

import os
import socket
import string
import struct
import sys
import platform
import urlparse

import roslib.exceptions
import roslib.rosenv 

SIOCGIFCONF=0x8912
SIOCGIFADDR = 0x8915

if 0:
    # disabling netifaces as it accounts for 50% of startup latency
    try:
        import netifaces
        _use_netifaces = True
    except:
        # NOTE: in rare cases, I've seen Python fail to extract the egg
        # cache when launching multiple python nodes.  Thus, we do
        # except-all instead of except ImportError (kwc).
        _use_netifaces = False
else:
    _use_netifaces = False

def parse_http_host_and_port(url):
    """
    Convenience routine to handle parsing and validation of HTTP URL
    port due to the fact that Python only provides easy accessors in
    Python 2.5 and later. Validation checks that the protocol and host
    are set.
    
    @param url: URL to parse
    @type  url: str
    @return: hostname and port number in URL or 80 (default).
    @rtype: (str, int)
    @raise ValueError: if the url does not validate
    """
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
    
def _is_unix_like_platform():
    """
    @return: true if the platform conforms to UNIX/POSIX-style APIs
    @rtype: bool
    """
    #return platform.system() in ['Linux', 'Mac OS X', 'Darwin']
    return platform.system() in ['Linux']

def get_address_override():
    """
    @return: ROS_IP/ROS_HOSTNAME override or None
    @rtype: str
    @raise ValueError: if ROS_IP/ROS_HOSTNAME/__ip/__hostname are invalidly specified
    """
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

def get_local_address():
    """
    @return: default local IP address (e.g. eth0). May be overriden by ROS_IP/ROS_HOSTNAME/__ip/__hostname
    @rtype: str
    """
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

# cache for performance reasons
_local_addrs = None
def get_local_addresses():
    """
    @return: known local addresses. Not affected by ROS_IP/ROS_HOSTNAME
    @rtype:  [str]
    """
    # cache address data as it can be slow to calculate
    global _local_addrs
    if _local_addrs is not None:
        return _local_addrs

    local_addrs = None
    if _use_netifaces:
        # #552: netifaces is a more robust package for looking up
        # #addresses on multiple platforms (OS X, Unix, Windows)
        local_addrs = []
        # see http://alastairs-place.net/netifaces/
        for i in netifaces.interfaces():
            try:
                local_addrs.extend([d['addr'] for d in netifaces.ifaddresses(i)[netifaces.AF_INET]])
            except KeyError: pass
    elif _is_unix_like_platform():
        # unix-only branch
        # adapted from code from Rosen Diankov (rdiankov@cs.cmu.edu)
        # and from ActiveState recipe

        import fcntl
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
        local_addrs = [socket.inet_ntoa(buffstr[i+20:i+24]) for i in range(0, retbytes, ifsize)]
    else:
        # cross-platform branch, can only resolve one address
        local_addrs = [socket.gethostbyname(socket.gethostname())]
    _local_addrs = local_addrs
    return local_addrs


def get_bind_address(address=None):
    """
    @param address: (optional) address to compare against
    @type  address: str
    @return: address TCP/IP sockets should use for binding. This is
    generally 0.0.0.0, but if \a address or ROS_IP/ROS_HOSTNAME is set
    to localhost it will return 127.0.0.1
    @rtype: str
    """
    if address is None:
        address = get_address_override()
    if address and \
           (address == 'localhost' or address.startswith('127.')):
        #localhost or 127/8
        return '127.0.0.1' #loopback
    else:
        return '0.0.0.0'

# #528: semi-complicated logic for determining XML-RPC URI
def get_host_name():
    """
    Determine host-name for use in host-name-based addressing (e.g. XML-RPC URIs):
     - if ROS_IP/ROS_HOSTNAME is set, use that address
     - if the hostname returns a non-localhost value, use that
     - use whatever L{get_local_address()} returns
    """
    hostname = get_address_override()
    if not hostname:
        try:
            hostname = socket.gethostname()
        except:
            pass
        if not hostname or hostname == 'localhost' or hostname.startswith('127.'):
            hostname = get_local_address()
    return hostname

def create_local_xmlrpc_uri(port):
    """
    Determine the XMLRPC URI for local servers. This handles the search
    logic of checking ROS environment variables, the known hostname,
    and local interface IP addresses to determine the best possible
    URI.
    
    @param port: port that server is running on
    @type  port: int
    @return: XMLRPC URI    
    @rtype: str
    """
    #TODO: merge logic in roslib.xmlrpc with this routine
    # in the future we may not want to be locked to http protocol nor root path
    return 'http://%s:%s/'%(get_host_name(), port)


## handshake utils ###########################################

class ROSHandshakeException(roslib.exceptions.ROSLibException):
    """
    Exception to represent errors decoding handshake
    """
    pass

def decode_ros_handshake_header(header_str):
    """
    Decode serialized ROS handshake header into a Python dictionary

    header is a list of string key=value pairs, each prefixed by a
    4-byte length field. It is preceeded by a 4-byte length field for
    the entire header.
    
    @param header_str: encoded header string. May contain extra data at the end.
    @type  header_str: str
    @return: key value pairs encoded in \a header_str
    @rtype: {str: str} 
    """
    (size, ) = struct.unpack('<I', header_str[0:4])
    size += 4 # add in 4 to include size of size field
    header_len = len(header_str)
    if size > header_len:
        raise ROSHandshakeException("Incomplete header. Expected %s bytes but only have %s"%((size+4), header_len))

    d = {}
    start = 4
    while start < size:
        (field_size, ) = struct.unpack('<I', header_str[start:start+4])
        if field_size == 0:
            raise ROSHandshakeException("Invalid 0-length handshake header field")
        start += field_size + 4
        if start > size:
            raise ROSHandshakeException("Invalid line length in handshake header: %s"%size)
        line = header_str[start-field_size:start]
        idx = line.find('=')
        if idx < 0:
            raise ROSHandshakeException("Invalid line in handshake header: [%s]"%line)
        key = line[:idx]
        value = line[idx+1:]
        d[key.strip()] = value
    return d
    
def read_ros_handshake_header(sock, b, buff_size):
    """
    Read in tcpros header off the socket \a sock using buffer \a b.
    
    @param sock: socket must be in blocking mode
    @type  sock: socket
    @param b: buffer to use
    @type  b: cStringIO
    @param buff_size: incoming buffer size to use
    @type  buff_size: int
    @return: key value pairs encoded in handshake
    @rtype: {str: str}
    @raise ROSHandshakeException: If header format does not match expected
    """
    header_str = None
    while not header_str:
        d = sock.recv(buff_size)
        if not d:
            raise ROSHandshakeException("connection from sender terminated before handshake header received. %s bytes were received. Please check sender for additional details."%b.tell())
        b.write(d)
        btell = b.tell()
        if btell > 4:
            # most likely we will get the full header in the first recv, so
            # not worth tiny optimizations possible here
            bval = b.getvalue()
            (size,) = struct.unpack('<I', bval[0:4])
            if btell - 4 >= size:
                header_str = bval
                    
                # memmove the remnants of the buffer back to the start
                leftovers = bval[size+4:]
                b.truncate(len(leftovers))
                b.seek(0)
                b.write(leftovers)
                header_recvd = True
                    
    # process the header
    return decode_ros_handshake_header(bval)

def encode_ros_handshake_header(header):
    """
    Encode ROS handshake header as a byte string. Each header
    field is a string key value pair. The encoded header is
    prefixed by a length field, as is each field key/value pair.
    key/value pairs a separated by a '=' equals sign.

    FORMAT: (4-byte length + [4-byte field length + field=value ]*)

    @param header: header field keys/values
    @type  header: dict
    @return: header encoded as byte string
    @rtype: str
    """    
    fields = ["%s=%s"%(k,v) for k,v in header.iteritems()]
    s = ''.join(["%s%s"%(struct.pack('<I', len(f)), f) for f in fields])
    return struct.pack('<I', len(s)) + s
                                        
def write_ros_handshake_header(sock, header):
    """
    Write ROS handshake header header to socket sock
    @param sock: socket to write to (must be in blocking mode)
    @type  sock: socket.socket
    @param header: header field keys/values
    @type  header: {str : str}
    @return: Number of bytes sent (for statistics)
    @rtype: int
    """
    s = encode_ros_handshake_header(header)
    sock.sendall(s)
    return len(s) #STATS
    
