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

from __future__ import with_statement

import os, sys
from subprocess import Popen, PIPE

## Main entrypoint into creating Eepydoc documentation
## @return [str]: list of packages that were successfully generated
def generate_epydoc(ctx):
    success = []
    for package, path in ctx.packages.iteritems():
        if package in ctx.doc_packages and ctx.should_document(package):
            builder = ctx.builder[package]
            if builder != 'epydoc':
                continue
            try:
                html_dir = os.path.join(ctx.docdir, package, 'html')
                if not os.path.isdir(html_dir):
                    os.makedirs(html_dir)
                command = ['epydoc', '--html', package, '-o', html_dir]

                # determine the python path of the package
                import roslib.launcher
                # - really dirty here, but the launcher intentionally caches the path for performance
                del roslib.launcher._bootstrapped[:]
                paths = roslib.launcher._generate_python_path(package, [], os.environ) 
                env = os.environ.copy()
                env['PYTHONPATH'] = os.pathsep.join(paths)

                print "epydoc-building %s [%s]"%(package, ' '.join(command))
                Popen(command, stdout=PIPE, env=env).communicate()
                success.append(package)
            except Exception, e:
                print >> sys.stderr, "Unable to generate eepydoc for [%s]. This is probably because eepydoc is not installed.\nThe exact error is:\n\t%s"%(package, str(e))
    return success
