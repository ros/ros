#!/usr/bin/env python
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

"""
ROS message source code generation for Python

Converts ROS .msg files in a package into Python source code implementations.
"""
import roslib; roslib.load_manifest('rospy')

import sys
import os
import traceback

# roslib.msgs contains the utilities for parsing .msg specifications. It is meant to have no rospy-specific knowledge
import roslib.msgs 
import roslib.packages 

# genutil is a utility package the implements the package crawling
# logic of genmsg_py and gensrv_py logic
import genutil

import roslib.genpy 

class GenmsgPackage(genutil.Generator):
    """
    GenmsgPackage generates Python message code for all messages in a
    package. See genutil.Generator. In order to generator code for a
    single .msg file, see msg_generator.
    """
    def __init__(self):
        super(GenmsgPackage, self).__init__(
            'genmsg_py', 'messages', roslib.msgs.EXT, roslib.packages.MSG_DIR, roslib.genpy.MsgGenerationException)

    def generate(self, package, f, outdir):
        """
        Generate python message code for a single .msg file
        @param f: path to .msg file
        @type  f: str
        @param outdir: output directory for generated code
        @type  outdir: str
        @return: filename of generated Python code 
        @rtype: str
        """
        verbose = True
        f = os.path.abspath(f)
        infile_name = os.path.basename(f)
        outfile_name = self.outfile_name(outdir, infile_name)

        (name, spec) = roslib.msgs.load_from_file(f, package)
        base_name = roslib.names.resource_name_base(name)
        
        self.write_gen(outfile_name, roslib.genpy.msg_generator(package, base_name, spec), verbose)

        roslib.msgs.register(name, spec)
        return outfile_name

if __name__ == "__main__":
    roslib.msgs.set_verbose(False)
    genutil.genmain(sys.argv, GenmsgPackage())
