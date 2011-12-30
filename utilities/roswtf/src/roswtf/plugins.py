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
#
# Revision $Id$

"""
Plugin loader for roswtf.
"""

from __future__ import print_function

import os
import sys

import roslib
import rospkg

def load_plugins():
    """
    @return: list of static roswtf plugins, list of online
    roswtf plugins
    @rtype: [fn], [fn]
    """
    rospack = rospkg.RosPack()
    to_check = rospack.get_depends_on('roswtf', implicit=False)
    static_plugins = []
    online_plugins = []
    for pkg in to_check:
        m = rospack.get_manifest(pkg)
        p_module = m.get_export('roswtf', 'plugin')
        if not p_module:
            continue
        elif len(p_module) != 1:
            print("Cannot load plugin [%s]: invalid 'plugin' attribute"%(pkg), file=sys.stderr)
            continue
        p_module = p_module[0]
        try:
            # load that packages namespace
            roslib.load_manifest(pkg)
            
            # import the specified plugin module
            mod = __import__(p_module)
            for sub_mod in p_module.split('.')[1:]:
                mod = getattr(mod, sub_mod)

            # retrieve the roswtf_plugin_static and roswtf_plugin_online functions
            s_attr = o_attr = None
            try:
                s_attr = getattr(mod, 'roswtf_plugin_static')
            except AttributeError: pass
            try:
                o_attr = getattr(mod, 'roswtf_plugin_online')
            except AttributeError: pass
            if s_attr:
                static_plugins.append(s_attr)
            if o_attr:
                online_plugins.append(o_attr)
            if s_attr is None and o_attr is None:
                print("Cannot load plugin [%s]: no 'roswtf_plugin_static' or 'roswtf_plugin_online' attributes [%s]"%(p_module), file=sys.stderr)
            else:
                print("Loaded plugin", p_module)

        except Exception:
            print("Unable to load plugin [%s] from package [%s]"%(p_module, pkg), file=sys.stderr)
    return static_plugins, online_plugins

