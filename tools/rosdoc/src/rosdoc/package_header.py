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
# Revision $Id: msgspec.py 3357 2009-01-13 07:13:05Z jfaustwg $
# $Author: jfaustwg $
from __future__ import with_statement

import os
import sys

import roslib.msgs
import roslib.scriptutil
import roslib.srvs
import roslib.stacks

_api_url = "http://pr.willowgarage.com/pr-docs/ros-packages/"
def package_link(package):
  return _api_url + package + "/html/"

## generate manifest.yaml files for MoinMoin PackageHeader macro
def generate_package_headers(ctx):
    try:
        import yaml
    except ImportError:
        print >> sys.stderr, "Cannot import yaml, will not generate MoinMoin PackageHeader files"
        return

    docdir = ctx.docdir
    manifests = ctx.manifests
    packages = ctx.packages
    for p in packages.iterkeys():
        if not ctx.should_document(p):
            continue
        m = manifests[p]
        d = {
            'brief': m.brief.encode('utf-8'),
            'description': m.description.strip().encode('utf-8') or '',
            'license': m.license or '',
            'authors': m.author.encode('utf-8') or '',
            'depends': [d.package for d in m.depends],
            'review_status': m.status or '',
            'review_notes': m.notes or '',
            'url': m.url,

            }

        if m.versioncontrol:
            d['version_control'] = m.versioncontrol.url
        
        siblings = []
        stack = 'foo'
        stack = roslib.stacks.stack_of(p) or ''
        if stack:
          d['stack'] = stack
          d['siblings'] = roslib.stacks.packages_of(stack)

        depends_on = []
        d['depends_on'] = roslib.scriptutil.rospackexec(['depends-on1', p]).split('\n')
    
        d['api_documentation'] = package_link(p)

        if p in ctx.external_docs:
            d['external_documentation'] = ctx.external_docs[p]

        d['msgs'] = roslib.msgs.list_msg_types(p, False)
        d['srvs'] = roslib.srvs.list_srv_types(p, False)        

        d['dependency_tree'] = package_link(p) + '%s.pdf'%p

        # encode unicode entries
        d_copy = d.copy()
        for k, v in d_copy.iteritems():
          if isinstance(v, basestring):
            d[k] = v.encode("utf-8")
          elif type(v) == list:
            d[k] = [x.encode("utf-8") for x in v]

        file_p = os.path.join(docdir, p, 'manifest.yaml')
        file_p_dir = os.path.dirname(file_p)
        if not os.path.isdir(file_p_dir):
          os.makedirs(file_p_dir)
        print "writing package properties to", file_p
        with open(file_p, 'w') as f:
            f.write(yaml.safe_dump(d))
        
        
