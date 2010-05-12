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

"""Parameter Server Cache"""

import threading

class ParamServerCache(object):
    """
    Cache of values on the parameter server. Implementation
    is just a thread-safe dictionary.
    """
    
    def __init__(self):
        self.lock = threading.Lock()
        self.d = {}
        
    ## Delete parameter from cache
    def delete(self, key):
        try:
            self.lock.acquire()
            del self.d[key]
        finally:
            self.lock.release()
        
    def update(self, key, value):
        """
        Update the value of the parameter in the cache
        @param key: parameter key
        @type  key: str
        @param value: parameter value
        @type  value: str
        @raise: KeyError if key is not already in the cache.
        """
        if not key in self.d:
            raise KeyError(key)
        try:
            self.lock.acquire()
            self.d[key] = value
        finally:
            self.lock.release()
            
    def set(self, key, value):
        """
        Set the value of the parameter in the cache. This is a
        prerequisite of calling update().
        @param key: parameter key
        @type  key: str
        @param value: parameter value
        @type  value: str
        """
        try:
            self.lock.acquire()
            self.d[key] = value
        finally:
            self.lock.release()
            
    def get(self, key):
        """
        @param key: parameter key
        @type  key: str
        @return: Current value for parameter
        @raise: KeyError
        """
        try:
            self.lock.acquire()
            return self.d[key]
        finally:
            self.lock.release()

_param_server_cache = None
def get_param_server_cache():
    """
    Get a handle on the client-wide parameter server cache
    """
    global _param_server_cache
    if _param_server_cache is None:
        _param_server_cache = ParamServerCache()        
    return _param_server_cache
