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

import sys

import roslib.packages
import rospy

import rosmsg

from rosh.impl.namespace import Namespace, Concept
    
class MsgPackage(Namespace):

    def __init__(self, name, config):
        """
        ctor.
        @param config: Namespace configuration instance with additional 'listener' attribute. 
        @type  config: L{NamespaceConfig}
        """
        super(MsgPackage, self).__init__(name, config)

        # an artifact of reusing the Namespace class is that we have to strip slashes
        if self._ns == '/':
            self._ns = ''
            
        self._msg_cache = {}

    def _list(self):
        """
        Override Namespace._list()
        """
        try:
            if not self._ns:
                return rosmsg.list_packages(mode=self._config.mode)
            else:
                return rosmsg.list_types(self._name, mode=self._config.mode)
        except:
            return []

    def _getAttributeNames(self):
        return list(set([s[len(self._ns):].split('/')[0] for s in self._list()]))

    def __repr__(self):
        return self.__str__()
        
    def __str__(self):
        return self._name

    def __getitem__(self, key):
        if not self._ns:
            return super(MsgPackage, self).__getitem__(key)
        elif key in self._msg_cache:
            return self._msg_cache[key]
        else:
            if self._config.mode == roslib.msgs.EXT:
                v = self._type = get_message_class(self._ns + key)
            else:
                v = self._type = get_service_class(self._ns + key)
            if v is not None:
                self._msg_cache[key] = v
            return v

def get_message_class(type_name):
    val = roslib.message.get_message_class(type_name, reload_on_error=True)
    if not val:
        pkg, base_type = roslib.names.package_resource_name(type_name)
        print >> sys.stderr, """Cannot retrieve type [%s].
Please type 'rosmake %s'"
"""%(type_name, pkg)
        return None
    else:
        return val

def get_service_class(type_name):
    val = roslib.message.get_service_class(type_name, reload_on_error=True)
    if not val:
        pkg, base_type = roslib.names.package_resource_name(type_name)
        print >> sys.stderr, """Cannot retrieve type [%s].
Please type 'rosmake %s'"
"""%(type_name, pkg)
        return None
    else:
        return val
    
class Msgs(Concept):

    def __init__(self, ctx, lock):
        super(Msgs, self).__init__(ctx, lock, MsgPackage)
        self._config.mode = roslib.msgs.EXT

class Srvs(Concept):

    def __init__(self, ctx, lock):
        super(Srvs, self).__init__(ctx, lock, MsgPackage)
        self._config.mode = roslib.srvs.EXT

