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
# Revision $Id: __init__.py 8626 2010-03-11 21:17:42Z kwc $
"""
Pluggable API for showing ROS data
"""

import os
import sys
import subprocess

import roslib.rosenv

from rosh.impl.exceptions import ROSHException
from rosh.impl.namespace import Namespace, Concept

def show(*args):
    # TODO: use roslaunch for process management?
    # TODO: this will become pluggable
    if not args:
        raise ValueError("show requires an argument")
    arg = args[0]
    if isinstance(arg, Namespace):
        #TODO:
        # - stamped topics
        arg._show()
    elif isinstance(arg, Concept):
        arg._show()
    elif isinstance(arg, type) and issubclass(arg, roslib.message.Message):
        import rosmsg
        print rosmsg.get_msg_text(arg._type)
    elif isinstance(arg, type) and issubclass(arg, roslib.message.ServiceDefinition):
        import rosmsg
        print rosmsg.get_srv_text(arg._type)
    else:
        raise ValueError("unknown show arg: %s: %s"%(type(arg), arg))
    
