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

from threading import RLock

from rosgraph.names import ns_join, GLOBALNS, SEP, is_global, is_private, canonicalize_name

def _get_param_names(names, key, d):
    """
    helper recursive routine for getParamNames()
    @param names: list of param names to append to
    @type  names: [str]
    @param d: parameter tree node
    @type  d: dict
    @param key: parameter key for tree node d
    @type  key: str
    """
    
    #TODOXXX
    for k,v in d.iteritems():
        if type(v) == dict:
            _get_param_names(names, ns_join(key, k), v)
        else:
            names.append(ns_join(key, k))

class ParamDictionary(object):
    
    def __init__(self, reg_manager):
        """
        ctor.
        @param subscribers: parameter subscribers
        @type  subscribers: Registrations
        """
        self.lock = RLock()
        self.parameters = {}
        self.reg_manager = reg_manager

    def get_param_names(self):
        """
        Get list of all parameter names stored on this server.

        @return: [code, statusMessage, parameterNameList]
        @rtype: [int, str, [str]]
        """
        try:
            self.lock.acquire()
            param_names = []
            _get_param_names(param_names, '/', self.parameters)
        finally:
            self.lock.release()
        return param_names
        
    def search_param(self, ns, key):
        """
        Search for matching parameter key for search param
        key. Search for key starts at ns and proceeds upwards to
        the root. As such, search_param should only be called with a
        relative parameter name.

        search_param's behavior is to search for the first partial match.
        For example, imagine that there are two 'robot_description' parameters:

         - /robot_description
         -   /robot_description/arm
         -   /robot_description/base

         - /pr2/robot_description
         -   /pr2/robot_description/base

        If I start in the namespace /pr2/foo and search for
        'robot_description', search_param will match
        /pr2/robot_description. If I search for 'robot_description/arm'
        it will return /pr2/robot_description/arm, even though that
        parameter does not exist (yet).

        @param ns: namespace to begin search from.
        @type  ns: str
        @param key: Parameter key. 
        @type  key: str
        @return: key of matching parameter or None if no matching
        parameter.
        @rtype: str
        """
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

    def get_param(self, key):
        """
        Get the parameter in the parameter dictionary.

        @param key: parameter key
        @type  key: str
        @return: parameter value
        """
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
    
    def set_param(self, key, value, notify_task=None):
        """
        Set the parameter in the parameter dictionary.

        @param key: parameter key
        @type  key: str
        @param value: parameter value
        @param notify_task: function to call with
        subscriber updates. updates is of the form
        [(subscribers, param_key, param_value)*]. The empty dictionary
        represents an unset parameter.
        @type  notify_task: fn(updates)
        """
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


    def subscribe_param(self, key, registration_args):
        """
        @param key: parameter key
        @type  key: str
        @param registration_args: additional args to pass to
        subscribers.register. First parameter is always the parameter
        key.
        @type  registration_args: tuple
        """
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
            

    def unsubscribe_param(self, key, unregistration_args):
        """
        @param key str: parameter key
        @type  key: str
        @param unregistration_args: additional args to pass to
        subscribers.unregister. i.e. unregister will be called with
        (key, *unregistration_args)
        @type  unregistration_args: tuple
        @return: return value of subscribers.unregister()
        """
        if key != SEP:
            key = canonicalize_name(key) + SEP
        return self.reg_manager.unregister_param_subscriber(key, *unregistration_args)

    def delete_param(self, key, notify_task=None):
        """
        Delete the parameter in the parameter dictionary.
        @param key str: parameter key
        @param notify_task fn(updates): function to call with
        subscriber updates. updates is of the form
        [(subscribers, param_key, param_value)*]. The empty dictionary
        represents an unset parameter.
        """
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
    
    def has_param(self, key):
        """
        Test for parameter existence

        @param key: parameter key
        @type  key: str
        @return: True if parameter set, False otherwise
        @rtype: bool
        """
        try:
            # more efficient implementations are certainly possible,
            # but this guarantees correctness for now
            self.get_param(key)
            return True
        except KeyError:
            return False
    
def _compute_all_keys(param_key, param_value, all_keys=None):
    """
    Compute which subscribers should be notified based on the parameter update
    @param param_key: key of updated parameter
    @type  param_key: str
    @param param_value: value of updated parameter
    @param all_keys: (internal use only) list of parameter keys
        to append to for recursive calls.
    @type  all_keys: [str]
    @return: list of parameter keys. All keys will be canonicalized with trailing slash.
    @rtype: [str]
    """
    if all_keys is None:
        all_keys = []
    for k, v in param_value.iteritems():
        new_k = ns_join(param_key, k) + SEP 
        all_keys.append(new_k)
        if type(v) == dict:
            _compute_all_keys(new_k, v, all_keys)
    return all_keys

def compute_param_updates(subscribers, param_key, param_value):
    """
    Compute subscribers that should be notified based on the parameter update
    @param subscribers: parameter subscribers
    @type  subscribers: Registrations
    @param param_key: parameter key
    @type  param_key: str
    @param param_value: parameter value
    @type  param_value: str
    """
    
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
        
