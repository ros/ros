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

from rosdoc.rdcore import *

## Main entrypoint into creating Epydoc documentation
## @return [str]: list of packages that were successfully generated
def generate_epydoc(ctx):
    success = []
    for package, path in ctx.packages.iteritems():
        if package in ctx.doc_packages and ctx.should_document(package) \
                and ctx.has_builder(package, 'epydoc'):

            # currently only allow one epydoc build per package. This
            # is not inherent, it just requires rewriting higher-level
            # logic
            rd_config = [d for d in ctx.rd_configs[package] if d['builder'] == 'epydoc'][0]

            # Configuration Properties (all optional):
            #
            # output_dir: directory_name (default: '.')
            # name: Documentation Set Name (default: Python API)
            try:
                html_dir = html_path(package, ctx.docdir)
                if 'output_dir' in rd_config:
                    html_dir = os.path.join(html_dir, rd_config['output_dir'])
                if not os.path.isdir(html_dir):
                    os.makedirs(html_dir)
                    
                command = ['epydoc', '--html', package, '-o', html_dir]
                if 'exclude' in rd_config:
                    for s in rd_config['exclude']:
                        command.extend(['--exclude', s])

                if 'config' in rd_config:
                    import roslib.packages
                    pkg_dir = roslib.packages.get_pkg_dir(package)
                    command.extend(['--config', os.path.join(pkg_dir, rd_config['config']) ])
                else:
                    # default options
                    command.extend(['--inheritance', 'included', '--no-private'])
                
                # determine the python path of the package
                import roslib.launcher
                # - really dirty here, but the launcher intentionally caches the path for performance
                del roslib.launcher._bootstrapped[:]
                paths = roslib.launcher._generate_python_path(package, [], os.environ) 
                env = os.environ.copy()
                env['PYTHONPATH'] = os.pathsep.join([p for p in paths if os.path.exists(p)])

                print "epydoc-building %s [%s]"%(package, ' '.join(command))
                Popen(command, stdout=PIPE, env=env).communicate()
                success.append(package)
            except Exception, e:
                print >> sys.stderr, "Unable to generate epydoc for [%s]. This is probably because epydoc is not installed.\nThe exact error is:\n\t%s"%(package, str(e))
    return success
