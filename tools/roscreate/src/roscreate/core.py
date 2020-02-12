#!/usr/bin/env python
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

from __future__ import print_function

import os
import sys

import pkg_resources

import rospkg


def print_warning(msg):
    """print warning to screen (bold red)"""
    print('\033[31m%s\033[0m' % msg, file=sys.stderr)


def author_name():
    """
    Utility to compute logged in user name

    :returns: name of current user, ``str``
    """
    import getpass
    name = getpass.getuser()
    try:
        import pwd
        login = name
        name = pwd.getpwnam(login)[4]
        name = ''.join(name.split(','))  # strip commas
        # in case pwnam is not set
        if not name:
            name = login
    except Exception:
        # pwd failed
        pass
    try:
        name = name.decode('utf-8')
    except AttributeError:
        pass
    return name


def read_template(tmplf):
    """
    Read resource template from egg installation, or fallback on rospkg otherwise.

    :returns: text of template file
    """
    if pkg_resources.resource_exists('roscreate', tmplf):
        f = pkg_resources.resource_stream('roscreate', tmplf)
        t = f.read()
    else:
        # fallback on rospkg
        r = rospkg.RosPack()
        with open(os.path.join(r.get_path('roscreate'), 'templates', tmplf)) as f:
            t = f.read()
    try:
        t = t.decode('utf-8')
    except AttributeError:
        pass
    return t
