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
import threading

from rospy.names import ns_join, GLOBALNS, SEP, canonicalize_name, is_global, is_private

## helper recursive routine for getParamNames()
## @param names [str]: list of param names to append to
## @param d dict: parameter tree node
## @param key str: parameter key for tree node \a d
def _get_param_names(names, key, d):
    #TODOXXX
    for k,v in d.iteritems():
        if type(v) == dict:
            _get_param_names(names, ns_join(key, k), v)
        else:
            names.append(ns_join(key, k))

class ParamDictionary(object):
    
    ## ctor
    ## @param self
    ## @param subscribers Registrations: parameter subscribers
    def __init__(self, reg_manager):
        self.lock = threading.RLock()
        self.parameters = {}
        self.reg_manager = reg_manager

    ## Get list of all parameter names stored on this server.
    ## @param self
    ## @return [int, str, [str]]: [code, statusMessage, parameterNameList]
    def get_param_names(self):
        try:
            self.lock.acquire()
            param_names = []
            _get_param_names(param_names, '/', self.parameters)
        finally:
            self.lock.release()
        return param_names
        
    ## Search for matching parameter key for search param \a
    ## key. Search for \a key starts at \a ns and proceeds upwards to
    ## the root. As such, search_param should only be called with a
    ## relative parameter name.
    ##
    ## search_param's behavior is to search for the first partial match.
    ## For example, imagine that there are two 'robot_description' parameters:
    ##
    ##  - /robot_description
    ##  -   /robot_description/arm
    ##  -   /robot_description/base
    ##
    ##  - /pr2/robot_description
    ##  -   /pr2/robot_description/base
    ##
    ## If I start in the namespace /pr2/foo and search for
    ## 'robot_description', search_param will match
    ## /pr2/robot_description. If I search for 'robot_description/arm'
    ## it will return /pr2/robot_description/arm, even though that
    ## parameter does not exist (yet).
    ##
    ## @param ns str: namespace to begin search from.
    ## @param key str: Parameter key. 
    ## @return str: key of matching parameter or None if no matching
    ## parameter. 
    def search_param(self, ns, key):
        if not key or is_private(key):
            raise ValueError("invalid key")
        if not is_global(ns):
            raise ValueError("namespace must be global")            
        if is_global(key):
            if self.has_param(key):
                return key
            else:
                return None

        # there are more efficient implementations, but our hiearchy
        # is not very deep and this is fairly clean code to read.

        # - we only search for the first namespace in the key to check for a match
        key_namespaces = [x for x in key.split(SEP) if x]
        key_ns = key_namespaces[0]

        #  - corner case: have to test initial namespace first as
        #    negative indices won't work with 0
        search_key = ns_join(ns, key_ns)
        if self.has_param(search_key):
            # resolve to full key
            return ns_join(ns, key) 
        
        namespaces = [x for x in ns.split(SEP) if x]
        for i in xrange(1, len(namespaces)+1):
            search_key = SEP + SEP.join(namespaces[0:-i] + [key_ns])
            if self.has_param(search_key):
                # we have a match on the namespace of the key, so
                # compose the full key and return it
                full_key = SEP + SEP.join(namespaces[0:-i] + [key]) 
                return full_key
        return None

    ## Get the parameter in the parameter dictionary.
    ## @param self
    ## @param key str: parameter key
    ## @return parameter value
    def get_param(self, key):
        try:
            self.lock.acquire()
            val = self.parameters
            if key != GLOBALNS:
                # split by the namespace separator, ignoring empty splits
                namespaces = [x for x in key.split(SEP)[1:] if x]
                for ns in namespaces:
                    if not type(val) == dict:
                        raise KeyError(val)
                    val = val[ns]
            return val
        finally:
            self.lock.release()
    
    ## Set the parameter in the parameter dictionary.
    ## @param self
    ## @param key str: parameter key
    ## @param value parameter value
    ## @param notify_task fn(updates): function to call with
    ## subscriber updates. updates is of the form
    ## [(subscribers, param_key, param_value)*]. The empty dictionary
    ## represents an unset parameter.
    def set_param(self, key, value, notify_task=None): 
        try:
            self.lock.acquire()
            if key == GLOBALNS:
                if type(value) != dict:
                    raise TypeError("cannot set root of parameter tree to non-dictionary")
                self.parameters = value
            else:
                namespaces = [x for x in key.split(SEP) if x]
                # - last namespace is the actual key we're storing in
                value_key = namespaces[-1]
                namespaces = namespaces[:-1]
                d = self.parameters
                # - descend tree to the node we're setting
                for ns in namespaces:
                    if not ns in d:
                        new_d = {}
                        d[ns] = new_d
                        d = new_d
                    else:
                        val = d[ns]
                        # implicit type conversion of value to namespace
                        if type(val) != dict:
                            d[ns] = val = {}
                        d = val

                d[value_key] = value

            # ParamDictionary needs to queue updates so that the updates are thread-safe
            if notify_task:
                updates = compute_param_updates(self.reg_manager.param_subscribers, key, value)
                if updates:
                    notify_task(updates)
        finally:
            self.lock.release()

    ## @param self
    ## @param key str: parameter key
    ## @param registration_args tuple: additional args to pass to
    ## subscribers.register. First parameter is always the parameter
    ## key.
    def subscribe_param(self, key, registration_args):
        if key != SEP:
            key = canonicalize_name(key) + SEP
        try:
            self.lock.acquire()
            # fetch parameter value
            try:
                val = self.get_param(key)
            except KeyError:
                # parameter not set yet
                val = {}
            self.reg_manager.register_param_subscriber(key, *registration_args)
            return val
        finally:
            self.lock.release()
            
    ## @param self
    ## @param key str: parameter key
    ## @param unregistration_args tuple: additional args to pass to
    ## subscribers.unregister. i.e. unregister will be called with
    ## (key, *unregistration_args)
    ## @return val : return value of subscribers.unregister()
    def unsubscribe_param(self, key, unregistration_args):
        if key != SEP:
            key = canonicalize_name(key) + SEP
        return self.reg_manager.unregister_param_subscriber(key, *unregistration_args)

    ## Delete the parameter in the parameter dictionary.
    ## @param self
    ## @param key str: parameter key
    ## @param notify_task fn(updates): function to call with
    ## subscriber updates. updates is of the form
    ## [(subscribers, param_key, param_value)*]. The empty dictionary
    ## represents an unset parameter.
    def delete_param(self, key, notify_task=None): 
        try:
            self.lock.acquire()
            if key == GLOBALNS:
                raise KeyError("cannot delete root of parameter tree")
            else:
                # key is global, so first split is empty
                namespaces = [x for x in key.split(SEP) if x]
                # - last namespace is the actual key we're deleting
                value_key = namespaces[-1]
                namespaces = namespaces[:-1]
                d = self.parameters
                # - descend tree to the node we're setting
                for ns in namespaces:
                    if type(d) != dict or not ns in d:
                        raise KeyError(key)
                    else:
                        d = d[ns]

                if not value_key in d:
                    raise KeyError(key)
                else:
                    del d[value_key]
                    
                # ParamDictionary needs to queue updates so that the updates are thread-safe
                if notify_task:
                    updates = compute_param_updates(self.reg_manager.param_subscribers, key, {})
                    if updates:
                        notify_task(updates)
        finally:
            self.lock.release()
    
    ## Test for parameter existence
    ## @param self
    ## @param key str: parameter key
    ## @return bool True if parameter set, False otherwise
    def has_param(self, key):
        try:
            # more efficient implementations are certainly possible,
            # but this guarantees correctness for now
            self.get_param(key)
            return True
        except KeyError:
            return False
    
## Compute which subscribers should be notified based on the parameter update
## @param param_key str: key of updated parameter
## @param param_value value: value of updated parameter
## @param all_keys [str]: (internal use only) list of parameter keys
## to append to for recursive calls.
## @return [str]: list of parameter keys. All keys will be canonicalized with trailing slash.
def _compute_all_keys(param_key, param_value, all_keys=None):
    if all_keys is None:
        all_keys = []
    for k, v in param_value.iteritems():
        new_k = ns_join(param_key, k) + SEP 
        all_keys.append(new_k)
        if type(v) == dict:
            _compute_all_keys(new_k, v, all_keys)
    return all_keys

## Compute subscribers that should be notified based on the parameter update
## @param subscribers Registrations: parameter subscribers
## @param param_key str: parameter key
## @param param_value str: parameter value
def compute_param_updates(subscribers, param_key, param_value):
    # logic correct for both updates and deletions

    if not subscribers:
        return []
    
    # end with a trailing slash to optimize startswith check from
    # needing an extra equals check
    if param_key != SEP:
        param_key = canonicalize_name(param_key) + SEP    

    # compute all the updated keys
    if type(param_value) == dict:
        all_keys = _compute_all_keys(param_key, param_value)
    else:
        all_keys = None
        
    updates = []
    
    # subscriber gets update if anything in the subscribed namespace is updated or if its deleted
    for sub_key in subscribers.iterkeys():
        ns_key = sub_key
        if ns_key[-1] != SEP:
            ns_key = sub_key + SEP
        if param_key.startswith(ns_key):
            node_apis = subscribers[sub_key]
            updates.append((node_apis, param_key, param_value))
        elif all_keys is not None and ns_key.startswith(param_key) \
             and not sub_key in all_keys:
            # parameter was deleted
            node_apis = subscribers[sub_key]
            updates.append((node_apis, sub_key, {}))

    # add updates for exact matches within tree
    if all_keys is not None:
        # #586: iterate over parameter tree for notification
        for key in all_keys:
            if key in subscribers:
                # compute actual update value
                sub_key = key[len(param_key):]
                namespaces = [x for x in sub_key.split(SEP) if x]
                val = param_value
                for ns in namespaces:
                    val = val[ns]

                updates.append((subscribers[key], key, val))

    return updates
        
#########################################################
# Parameter Server Cache

## Cache of values on the parameter server. Implementation
## is just a thread-safe dictionary.
class ParamServerCache(object):
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
        
    ## Update the value of the parameter in the cache
    ## @param self
    ## @param key str: parameter key
    ## @param value str: parameter value
    ## @throws KeyError if \a key is not already in the cache.
    def update(self, key, value):
        if not key in self.d:
            raise KeyError(key)
        try:
            self.lock.acquire()
            self.d[key] = value
        finally:
            self.lock.release()
            
    ## Set the value of the parameter in the cache. This is a
    ## prerequisite of calling update().
    ## @param self
    ## @param key str: parameter key
    ## @param value str: parameter value
    def set(self, key, value):
        try:
            self.lock.acquire()
            self.d[key] = value
        finally:
            self.lock.release()
            
    ## @param self
    ## @param key str: parameter key
    ## @return Current value for parameter
    ## @throws KeyError: if value is not currently cached
    def get(self, key):
        try:
            self.lock.acquire()
            return self.d[key]
        finally:
            self.lock.release()

_param_server_cache = None
## Get a handle on the client-wide parameter server cache
def get_param_server_cache():
    global _param_server_cache
    if _param_server_cache is None:
        _param_server_cache = ParamServerCache()        
    return _param_server_cache
