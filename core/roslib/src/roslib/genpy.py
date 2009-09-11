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
# Revision $Id: genpy.py 3357 2009-01-13 07:13:05Z jfaustwg $

## optimize the struct format pattern. 
def reduce_pattern(pattern):
    if not pattern or len(pattern) == 1 or '%' in pattern:
        return pattern
    prev = pattern[0]
    count = 1
    new_pattern = ''
    nums = [str(i) for i in range(0, 9)]
    for c in pattern[1:]:
        if c == prev and not c in nums:
            count += 1
        else:
            if count > 1:
                new_pattern = new_pattern + str(count) + prev
            else:
                new_pattern = new_pattern + prev
            prev = c
            count = 1
    if count > 1:
        new_pattern = new_pattern + str(count) + c
    else:
        new_pattern = new_pattern + prev
    return new_pattern

## @param expr str: string python expression that is evaluated for serialization
## @return str: python call to write value returned by expr to serialization buffer
def serialize(expr):
    return "buff.write(%s)"%expr
    
#NOTE: '<' = little endian
def pack(pattern, vars):
    """create struct.pack call for when pattern is a string pattern
    @param pattern: string pattern for pack
    @param vars: name of variables to pack"""
    return serialize("struct.pack('<"+reduce_pattern(pattern)+"', "+vars+")")
def pack2(pattern, vars):
    """create struct.pack call for when pattern is the name of a variable
    @param pattern: name of variable storing string pattern
    @param vars: name of variables to pack"""
    return serialize("struct.pack(%s, %s)"%(pattern, vars))
def unpack(var, pattern, buff):
    """create struct.unpack call for when pattern is a string pattern"""
    return var + " = struct.unpack('<"+reduce_pattern(pattern)+"',"+buff+")"
def unpack2(var, pattern, buff):
    """create struct.unpack call for when pattern refers to variable
    @param var: variable the stores the result of unpack call
    @param pattern: name of variable that unpack will read from
    @param buff: buffer that the unpack reads from"""
    return "%s = struct.unpack(%s, %s)"%(var, pattern, buff)

