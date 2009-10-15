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
import time

from subprocess import Popen, PIPE

import roslib.msgs
import roslib.srvs
from rosdoc.rdcore import *

def _href(location, text):
    return '<a href="%(location)s">%(text)s</a>'%locals()

def link_name(rd_config):
    if 'name' in rd_config:
        n = rd_config['name']
    else:
        if rd_config['builder'] == 'doxygen':
            return 'C++ API'
        elif rd_config['builder'] in ['epydoc', 'sphinx']:
            return 'Python API'
        else:
            return rd_config['builder']
    return n
    
def generate_links(ctx, package, base_dir, rd_configs):
    # rosmake is the one builder that doesn't have an output_dir
    # specification, nor output for that matter, so filter it out from
    # landing page processing
    configs = [c for c in rd_configs if c['builder'] != 'rosmake']
    
    output_dirs = [c.get('output_dir', None) for c in configs]
    # filter out empties
    output_dirs = [d for d in output_dirs if d and d != '.']
    
    # length check. if these are unequal, cannot generate landing
    # page. this is often true if the config is merely generating
    # local. Ignore 'rosmake' builder as it doesn't have output spec
    if len(output_dirs) != len(configsj):
        return None

    links = [_href(d, link_name(c)) for c, d in zip(configs, output_dirs)]
        
    msgs = roslib.msgs.list_msg_types(package, False)
    srvs = roslib.srvs.list_srv_types(package, False)
    if msgs or srvs:
        if msgs and srvs:
            title = 'msg/srv API'
        elif msgs and not srvs:
            title = 'msg API'    
        elif srvs and not msgs:
            title = 'srv API'
        #TODO: this shouldn't be hardcoded to index-msg.html
        links.append(_href('index-msg.html', title))

    url = ctx.manifests[package].url
    if url:
        links.append(_href(url, '%s Package Documentation'%package))
    return links

## Generate landing page in the event that there are multiple documentation sets
## @return [str]: list of packages for which there are landing pages generated
def generate_landing_page(ctx):
    success = []
    template = load_tmpl('landing.template')
    #print "landing_page: packages are", ctx.packages.keys()
    for package in ctx.packages.iterkeys():
        #print "landing page", package
        try:
            if package in ctx.doc_packages and ctx.should_document(package) and \
                package in ctx.rd_configs:
                rd_configs = ctx.rd_configs[package]
                links = generate_links(ctx, package, ctx.docdir, rd_configs)
                # if links is empty, it means that the rd_configs builds
                # to the base directory and no landing page is required
                # (or it means that the config is corrupt)
                if not links:
                    #print "ignoring landing page for", package
                    continue

                html_dir = html_path(package, ctx.docdir)
                #print "generating landing page", html_dir

                if not os.path.isdir(html_dir):
                    os.makedirs(html_dir)

                links_html = '\n'.join(['<li class="landing-li">%s</li>'%l for l in links])
                date = str(time.strftime('%a, %d %b %Y %H:%M:%S'))                
                vars = {
                    '$package': package,
                    '$links': links_html,
                    '$date': date,
                        }

                with open(os.path.join(html_dir, 'index.html'), 'w') as f:
                    f.write(instantiate_template(template, vars))
                success.append(package)
        except Exception, e:
            print >> sys.stderr, "Unable to generate landing_page for [%s]:\n\t%s"%(package, str(e))
    return success
