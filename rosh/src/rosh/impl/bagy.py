# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

from __future__ import with_statement

import roslib.message
import yaml

# our declaration of open overrides the global open, which we need
f_open = open

def open(filename, mode, type_=None):
    """
    @param mode: open mode. Valid modes are 'r' (read), 'w' (write), and 'a' (append). 
    @type  mode: str
    """
    if mode in ['r', 'w', 'a']:
        if mode == 'r' and type_ is None:
            raise ValueError("type_ must be specified when reading from bagy files")
        bagy = Bagy(filename, f_open(filename, mode), type_)
    else:
        raise ValueError("mode is invalid")
    return bagy
    
def create(bagy, type_, filename=None):
    """
    Load single message from YAML dictionary representation.

    @param type_: Message class
    @type  type_: class (Message subclass)
    @param bagy: Message in bagy string or dictionary representation
    @type  bagy: str or dict
    @param filename: Name of bagy YAML file for debugging.
    @type  filename: str
    """
    if type(bagy) == str:
        bagy = yaml.load(bagy)
    if type(bagy) != dict:
        if filename:
            raise ValueError("bagy file [%s] does not contain a YAML dictionary"%filename)
        else:
            raise ValueError("bagy must contain a dictionary")
    m = type_()
    # TODO: add new API to roslib.message that doesn't assume a rostopic-like API source
    roslib.message.fill_message_args(m, [bagy])
    return m

class Bagy(object):

    def __init__(self, name, stream, type_):
        """
        ctor.
        """
        self.name    = name
        self.type_   = type_
        self._stream = stream
        self._buff   = None
        self._yaml_iter = None
        
    def __enter__(self):
        return self

    def __exit__(self):
        self.close()

    def __iter__(self):
        for doc in yaml.load_all(self._stream):
            yield create(doc, self.type_)

    def write(self, msg):
                        
        if not isinstance(msg, roslib.message.Message):
            raise ValueError("msg is not a message instance")
        self._stream.write(roslib.message.strify_message(msg) + '\n---\n')

    def read(self):
        retval = [create(d, self.type_) for d in yaml.load_all(self._stream)]
        self.close()
        return retval

    def next(self):
        if self._yaml_iter is None:
            self._yaml_iter = yaml.load_all(self._stream)
        v = self._yaml_iter.next()
        if v is None:
            return None
        return create(v, self.type_)

    def close(self):
        self._stream.close()

def LabeledBagy(object):
    pass
