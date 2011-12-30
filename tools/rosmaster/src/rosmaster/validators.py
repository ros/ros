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

"""Internal-use Python decorators for parameter validation"""

from rosgraph.names import resolve_name, ANYTYPE

TYPE_SEPARATOR = '/'
ROSRPC = "rosrpc://"

class ParameterInvalid(Exception):
    """Exception that is raised when a parameter fails validation checks"""
    def __init__(self, message):
        self._message = message

    def __str__(self):
        return str(self._message)

def non_empty(param_name):
    """Validator that checks that parameter is not empty"""
    def validator(param, context):
        if not param:
            raise ParameterInvalid("ERROR: parameter [%s] must be specified and non-empty"%param_name)
        return param
    return validator

def non_empty_str(param_name):
    """Validator that checks that parameter is a string and non-empty"""
    def validator(param, context):
        if not param:
            raise ParameterInvalid("ERROR: parameter [%s] must be specified and non-empty"%param_name)
        elif not isinstance(param, basestring):
            raise ParameterInvalid("ERROR: parameter [%s] must be a string"%param_name)            
        return param
    return validator
        
def not_none(param_name):
    """Validator that checks that parameter is not None"""
    def validator(param, context):
        if param is None:
            raise ParameterInvalid("ERROR: parameter [%s] must be specified"%param_name)
        return param
    return validator


# Validators ######################################

def is_api(paramName):
    """
    Validator that checks that parameter is a valid API handle
    (i.e. URI). Both http and rosrpc are allowed schemes.
    """
    def validator(param_value, callerId):
        if not param_value or not isinstance(param_value, basestring):
            raise ParameterInvalid("ERROR: parameter [%s] is not an XMLRPC URI"%paramName)
        if not param_value.startswith("http://") and not param_value.startswith(ROSRPC):
            raise ParameterInvalid("ERROR: parameter [%s] is not an RPC URI"%paramName)
        #could do more fancy parsing, but the above catches the major cases well enough
        return param_value
    return validator

def is_topic(param_name):
    """
    Validator that checks that parameter is a valid ROS topic name
    """    
    def validator(param_value, caller_id):
        v = valid_name_validator_resolved(param_name, param_value, caller_id)
        if param_value == '/':
            raise ParameterInvalid("ERROR: parameter [%s] cannot be the global namespace"%param_name)            
        return v
    return validator

def is_service(param_name):
    """Validator that checks that parameter is a valid ROS service name"""
    def validator(param_value, caller_id):
        v = valid_name_validator_resolved(param_name, param_value, caller_id)
        if param_value == '/':
            raise ParameterInvalid("ERROR: parameter [%s] cannot be the global namespace"%param_name)            
        return v
    return validator

def empty_or_valid_name(param_name):
    """
    empty or valid graph resource name.
    Validator that resolves names unless they an empty string is supplied, in which case
    an empty string is returned.
    """
    def validator(param_value, caller_id):
        if not isinstance(param_value, basestring):
            raise ParameterInvalid("ERROR: parameter [%s] must be a string"%param_name)              
        if not param_value:
            return ''
        #return resolve_name(param_value, namespace(caller_id))
        return resolve_name(param_value, caller_id)
    return validator

def valid_name_validator_resolved(param_name, param_value, caller_id):
    if not param_value or not isinstance(param_value, basestring):
        raise ParameterInvalid("ERROR: parameter [%s] must be a non-empty string"%param_name)            
    #TODO: actual validation of chars
    # I added the colon check as the common error will be to send an URI instead of name
    if ':' in param_value or ' ' in param_value:
        raise ParameterInvalid("ERROR: parameter [%s] contains illegal chars"%param_name) 
    #return resolve_name(param_value, namespace(caller_id))
    return resolve_name(param_value, caller_id)
def valid_name_validator_unresolved(param_name, param_value, caller_id):
    if not param_value or not isinstance(param_value, basestring):
        raise ParameterInvalid("ERROR: parameter [%s] must be a non-empty string"%param_name)            
    #TODO: actual validation of chars        
    # I added the colon check as the common error will be to send an URI instead of name
    if ':' in param_value or ' ' in param_value:
        raise ParameterInvalid("ERROR: parameter [%s] contains illegal chars"%param_name) 
    return param_value
    
def valid_name(param_name, resolve=True):
    """
    Validator that resolves names and also ensures that they are not empty
    @param param_name: name
    @type  param_name: str
    @param resolve: if True/omitted, the name will be resolved to
       a global form. Otherwise, no resolution occurs.
    @type  resolve: bool
    @return: resolved parameter value
    @rtype: str
    """
    def validator(param_value, caller_id):
        if resolve:
            return valid_name_validator_resolved(param_name, param_value, caller_id)
        return valid_name_validator_unresolved(param_name, param_value, caller_id)        
    return validator

def global_name(param_name):
    """
    Validator that checks for valid, global graph resource name.
    @return: parameter value
    @rtype: str
    """
    def validator(param_value, caller_id):
        if not param_value or not isinstance(param_value, basestring):
            raise ParameterInvalid("ERROR: parameter [%s] must be a non-empty string"%param_name)
        #TODO: actual validation of chars
        if not is_global(param_value):
            raise ParameterInvalid("ERROR: parameter [%s] must be a globally referenced name"%param_name)            
        return param_value
    return validator

def valid_type_name(param_name):
    """validator that checks the type name is specified correctly"""
    def validator(param_value, caller_id):
        if param_value == ANYTYPE:
            return param_value
        if not param_value or not isinstance(param_value, basestring):
            raise ParameterInvalid("ERROR: parameter [%s] must be a non-empty string"%param_name)            
        if not len(param_value.split(TYPE_SEPARATOR)) == 2:
            raise ParameterInvalid("ERROR: parameter [%s] is not a valid package resource name"%param_name)
        #TODO: actual validation of chars
        return param_value
    return validator
