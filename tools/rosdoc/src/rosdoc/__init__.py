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

import sys
import os
import time
import traceback
from subprocess import Popen, PIPE

NAME='rosdoc'

from rdcore import *

def main():
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog [options] [packages...]", prog=NAME)
    parser.add_option("-n", "--name",metavar="NAME",
                      dest="name", default="ROS Package", 
                      help="Name for documentation set")
    parser.add_option("-q", "--quiet",action="store_true", default=False,
                      dest="quiet",
                      help="Suppress doxygen errors")
    parser.add_option("--paths",metavar="PATHS",
                      dest="paths", default=None, 
                      help="package paths to document")
    parser.add_option("-o",metavar="OUTPUT_DIRECTORY",
                      dest="docdir", default='doc', 
                      help="directory to write documentation to")
    options, package_filters = parser.parse_args()

    # Load the ROS environment
    ctx = RosdocContext(options.name, options.docdir,
                        package_filters=package_filters, path_filters=options.paths)
    try:
        ctx.init()
    except Exception, e:
        traceback.print_exc()
        sys.exit(1)

    try:

        # Collect all packages that mention rosmake as a builder, and build them first
        to_rosmake = []
        for package in ctx.rd_configs:
            builders = [d['builder'] for d in ctx.rd_configs[package]]
            if 'rosmake' in builders:
                to_rosmake.append(package)
        if to_rosmake != []:
            command = ['rosmake'] + to_rosmake
            print " ".join(command)
            started = time.time()
            try:
                Popen(command, stdout=PIPE).communicate()
            except:
                print "command failed"
            print "rosmake took %ds" % (time.time() - started)

        # Generate Epydoc
        if 1:
            print "building epydoc packages"
            import epyenator
            epyenator_success = set(epyenator.generate_epydoc(ctx))
        else:
            epyenator_success = set()
            
        # Generate Sphinx
        if 1:
            print "building sphinx packages"            
            import sphinxenator
            try:
                sphinx_success = set(sphinxenator.generate_sphinx(ctx))
            except Exception, e:
                traceback.print_exc()
                print >> sys.stderr, "sphinxenator failed"
                sphinx_success = set()            
        else:
            sphinx_success = set()            
        
        # Generate Doxygen 
        if 1:
            print "building doxygen packages"
            try:
                import doxygenator
                doxy_success = doxygenator.generate_doxygen(ctx, quiet=options.quiet) 
            except Exception, e:
                traceback.print_exc()
                print >> sys.stderr, "package header generation failed"
                doxy_success = []                
        else:
            doxy_success = []
        success = list(sphinx_success) + doxy_success + list(epyenator_success)

        if 1:
            # Generate yaml data for wiki macros
            try:
                import package_header
                package_header.generate_package_headers(ctx)
                package_header.generate_stack_headers(ctx)
            except Exception, e:
                traceback.print_exc()
                print >> sys.stderr, "package header generation failed"

        if 1:
            # Generate msg/srv auto-docs
            import msgenator
            try:
                msgenator.generate_msg_docs(ctx)
            except Exception, e:
                traceback.print_exc()
                print >> sys.stderr, "msgenator failed"

        if 1:
            # Generate landing page
            import landing_page
            try:
                landing_page.generate_landing_page(ctx)
            except:
                traceback.print_exc()
                print >> sys.stderr, "landing page generator failed"
            
        if 1:
            # Generate Documentation Index
            import docindex 
            doc_index = os.path.join(ctx.docdir, 'index.html')
            docindex.generate_doc_index(ctx, success, doc_index)

        if 1:
            # Generate License Index
            import licenseindex
            license_index = os.path.join(ctx.docdir, 'licenses.html')
            licenseindex.generate_license_index(ctx, license_index)

        if 1:
            # support files
            import shutil
            styles_in = os.path.join(ctx.template_dir, 'styles.css')
            styles_css = os.path.join(ctx.docdir, 'styles.css')
            print "copying",styles_in, "to", styles_css
            shutil.copyfile(styles_in, styles_css)
    except:
        traceback.print_exc()
