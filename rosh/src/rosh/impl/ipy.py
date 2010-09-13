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
"""
IPython extensions.
"""

import os
import sys

import roslib.packages
import roslib.rosenv

def ipy_magic_init():
    # Add some magic
    try:
        import IPython.Magic
        IPython.Magic.Magic.magic_roscd = ipy_roscd
        IPython.Magic.Magic.magic_rosmake = ipy_rosmake
    except ImportError:
        pass

def ipy_roscd(magic, target):
    if not target:
        magic.magic_cd(roslib.rosenv.get_ros_root())
    else:
        splits = target.split(os.sep)
        if splits:
            magic.magic_cd(os.sep.join( [roslib.packages.get_pkg_dir(splits[0])] + splits[1:]))
        else:
            magic.magic_cd(roslib.packages.get_pkg_dir(target))

def ipy_rosmake(magic, target):
    if not target:
        if os.path.isfile(os.path.join(os.getcwd(), roslib.manifest.MANIFEST_FILE)):
            import subprocess
            print "starting rosmake in %s, this may take awhile"%(os.getcwd())
            subprocess.Popen(['rosmake'])
        else:
            print >> sys.stderr, "no package present in cwd"
    elif type(target) != str:
        print >> sys.stderr, "argument must be a package name", type(target)
    else:
        if os.sep in target:
            print >> sys.stderr, "invalid argument: %s"%target
        else:
            #TODO: use python rosmake library instead

            import subprocess
            print "starting rosmake %s, this may take awhile"%target
            subprocess.Popen(['rosmake', target])
            
