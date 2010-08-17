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
import rosdoc.upload

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
    parser.add_option("--no-rxdeps", action="store_true",
                      dest="no_rxdeps", default=False, 
                      help="disable rxdeps")
    parser.add_option("-o",metavar="OUTPUT_DIRECTORY",
                      dest="docdir", default='doc', 
                      help="directory to write documentation to")
    parser.add_option("--repos", default=None,
                      dest="repos", metavar="ROSBROWSE_REPOS_FILE",
                      help="repos list from rosbrowse for determining repository names/roots")
    parser.add_option("-u", "--upload",action="store_true", default=False,
                      dest="upload",
                      help="Suppress doxygen errors")

    options, package_filters = parser.parse_args()

    # Load the ROS environment
    # - repos is for the rosdoc build on Hudson. It generates the correct repository roots for generating package_headers
    repos = None
    if options.repos:
        with open(options.repos, 'r') as f:
            # load file
            repos = [l.split() for l in f if not l.startswith('#')]
            # convert to dictionary
            repos = dict([(key, (type, uri)) for key, type, uri in repos])

    ctx = RosdocContext(options.name, options.docdir,
                        package_filters=package_filters, path_filters=options.paths,
                        repos=repos)
    try:
        ctx.init()
    except Exception, e:
        traceback.print_exc()
        sys.exit(1)

    try:
        import time

        rm_start = time.time()

        # Collect all packages that mention rosmake as a builder, and build them first
        to_rosmake = []
        for package in ctx.rd_configs:
                if (package in ctx.doc_packages and
                    ctx.should_document(package) and
                    ctx.has_builder(package, 'rosmake')):
                    to_rosmake.append(package)
        if to_rosmake != []:
            # command = ['rosmake', '--status-rate=0'] + to_rosmake
            command = ['rosmake', '-V'] + to_rosmake
            print " ".join(command)
            started = time.time()
            try:
                (stdoutdata, _) = Popen(command, stdout=PIPE).communicate()
                print stdoutdata
            except:
                print "command failed"
            print "rosmake took %ds" % (time.time() - started)
        rm_end = time.time()

        e_start = time.time()
        # Generate Epydoc
        if 1:
            print "building epydoc packages"
            import epyenator
            epyenator_success = set(epyenator.generate_epydoc(ctx))
        else:
            epyenator_success = set()

        e_end = time.time()

        s_start = time.time()        
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
        s_end = time.time()                    
        

        d_start = time.time()
        # Generate Doxygen 
        if 1:
            print "building doxygen packages"
            try:
                import doxygenator
                doxy_success = doxygenator.generate_doxygen(ctx, quiet=options.quiet, disable_rxdeps=options.no_rxdeps) 
            except Exception, e:
                traceback.print_exc()
                print >> sys.stderr, "package header generation failed"
                doxy_success = []                
        else:
            doxy_success = []
        success = list(sphinx_success) + doxy_success + list(epyenator_success)
        d_end = time.time()

        stack_dirs = []
        ph_start = time.time()        
        if 1:
            # Generate yaml data for wiki macros
            try:
                import package_header
                package_header.generate_package_headers(ctx)
                stack_dirs = package_header.generate_stack_headers(ctx)
            except Exception, e:
                traceback.print_exc()
                print >> sys.stderr, "package header generation failed"
        ph_end = time.time()        

        mg_start = time.time()                
        if 1:
            # Generate msg/srv auto-docs
            import msgenator
            try:
                msgenator.generate_msg_docs(ctx)
            except Exception, e:
                traceback.print_exc()
                print >> sys.stderr, "msgenator failed"
        mg_end = time.time()

        lp_start = time.time()        
        if 1:
            # Generate landing page
            import landing_page
            try:
                landing_page.generate_landing_page(ctx)
            except:
                traceback.print_exc()
                print >> sys.stderr, "landing page generator failed"
        lp_end = time.time()        

        di_start = time.time()                
        if 1:
            # Generate Documentation Index
            import docindex 
            doc_index = os.path.join(ctx.docdir, 'index.html')
            docindex.generate_doc_index(ctx, ctx.doc_packages , doc_index)
        di_end = time.time()                

        li_start = time.time()                
        if 1:
            # Generate License Index
            import licenseindex
            license_index = os.path.join(ctx.docdir, 'licenses.html')
            licenseindex.generate_license_index(ctx, license_index)
        li_end = time.time()
        sup_start = time.time()                        
        if 1:
            # support files
            import shutil
            styles_in = os.path.join(ctx.template_dir, 'styles.css')
            styles_css = os.path.join(ctx.docdir, 'styles.css')
            print "copying",styles_in, "to", styles_css
            shutil.copyfile(styles_in, styles_css)
        sup_end = time.time()

        upload_start = time.time()
        if options.upload:
            print success, stack_dirs
            rosdoc.upload.upload(success + stack_dirs + ['index.html', 'licenses.html', 'styles.css'], '/tmp/docs')
            print os.listdir('/tmp/docs')
            
        upload_end = time.time()

        print """Timings
 * %.2f Rosmake
 * %.2f Epydoc
 * %.2f Sphinx
 * %.2f Doxygen
 * %.2f Package Header
 * %.2f Landing Page
 * %.2f Documentation Index
 * %.2f License Index
 * %.2f Upload Packages
"""%( (rm_end - rm_start), 
      (e_end - e_start), (s_end - s_start), (d_end - d_start),
      (ph_end - ph_start), (lp_end - lp_start),
      (di_end - di_start), (li_end - li_start),
       (upload_end - upload_start),
      )

    except:
        traceback.print_exc()
