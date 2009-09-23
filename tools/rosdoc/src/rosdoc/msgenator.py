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

import cStringIO
import os
import sys
import time
import shutil

import roslib.msgs
import roslib.scriptutil
import roslib.srvs
import roslib.stacks

from roslib.msgs import msg_file
from roslib.srvs import srv_file

from rosdoc.rdcore import *
import rosmsg

msg_template = load_tmpl('msg.template')
msg_index_template = load_tmpl('msg-index.template')

_api_url = "http://pr.willowgarage.com/pr-docs/ros-packages/"
def package_link(package):
  return _api_url + package + "/html/"

def _href(link, text):
    return '<a href="%(link)s">%(text)s</a>'%locals()

def type_link(type_, base_package):
    if type_ in roslib.msgs.BUILTIN_TYPES:
        return type_
    base_type_ = roslib.msgs.base_msg_type(type_)
    package, base_type_ = roslib.names.package_resource_name(base_type_)
    if not package or package == base_package:
        return _href("%s.html"%base_type_, type_)
    else:
        return _href("../../../%(package)s/html/msg/%(base_type_)s.html"%locals(), type_)

def index_type_link(pref, type_, base_package):
    if type_ in roslib.msgs.BUILTIN_TYPES:
        return type_
    base_type_ = roslib.msgs.base_msg_type(type_)
    package, base_type_ = roslib.names.package_resource_name(base_type_)
    if not package or package == base_package:
        return _href("%s/%s.html"%(pref, base_type_), type_)
    else:
        return _href("../../%(package)s/html/%(pref)s/%(base_type_)s.html"%locals(), type_)
    
def _generate_raw_text(raw_fn, msg):
    raw_text = raw_fn(msg, raw=True)
    s = ''
    for line in raw_text.split('\n'):
        parts = line.split('#')
        if len(parts) > 1:
            s = s + parts[0]+'<font color="blue">#%s</font><br/>'%('#'.join(parts[1:]))
        else:
            s = s + "%s<br />"%parts[0]
    return s

def _generate_msg_text_from_spec(package, spec, buff=None, indent=0):
    if buff is None:
        buff = cStringIO.StringIO()
    for c in spec.constants:
        buff.write("%s%s %s=%s<br />"%("&nbsp;"*indent, c.type, c.name, c.val_text))
    for type_, name in zip(spec.types, spec.names):
        buff.write("%s%s %s<br />"%("&nbsp;"*indent, type_link(type_, package), name))
        base_type = roslib.msgs.base_msg_type(type_)
        if not base_type in roslib.msgs.BUILTIN_TYPES:
            subspec = roslib.msgs.get_registered(base_type)
            _generate_msg_text_from_spec(package, subspec, buff, indent + 4)
    return buff.getvalue()

def _generate_msg_text(package, type_):
    #print "generate", package, type_
    name, spec = roslib.msgs.load_from_file(msg_file(package, type_))
    return _generate_msg_text_from_spec(package, spec)

def _generate_srv_text(package, type_):
    name, spec = roslib.srvs.load_from_file(srv_file(package, type_))
    return _generate_msg_text_from_spec(package, spec.request) + \
        '<hr />'+\
        _generate_msg_text_from_spec(package, spec.response) 

def generate_srv_doc(srv):
    package, base_type = roslib.names.package_resource_name(srv)
    d = { 'name': srv, 'ext': 'srv', 'type': 'Service',
          'package': package, 'base_type' : base_type,
          'date': str(time.strftime('%a, %d %b %Y %H:%M:%S'))}
    d['fancy_text'] = _generate_srv_text(package, base_type)
    d['raw_text'] = _generate_raw_text(rosmsg.get_srv_text, srv)
    raw_text = _generate_raw_text(rosmsg.get_srv_text, srv)
    return msg_template%d

def generate_msg_doc(msg):
    package, base_type = roslib.names.package_resource_name(msg)    
    d = { 'name': msg, 'ext': 'msg', 'type': 'Message',
          'package': package, 'base_type' : base_type,
          'date': str(time.strftime('%a, %d %b %Y %H:%M:%S'))}
    d['fancy_text'] = _generate_msg_text(package, base_type)
    d['raw_text'] = _generate_raw_text(rosmsg.get_msg_text, msg)
    return msg_template%d

def generate_msg_index(package, file_d, msgs, srvs, wiki_url):
    d = {'package': package, 'msg_list' : '', 'srv_list': '',
         'package_url': wiki_url,
         'date': str(time.strftime('%a, %d %b %Y %H:%M:%S'))}
    if msgs:
        d['msg_list'] = """<h2>Message types</h2>
<div class="msg-list">
  <ul>
%s
  </ul>
</div>"""%'\n'.join([" <li>%s</li>"%index_type_link('msg', m, package) for m in msgs])

    if srvs:
        d['srv_list'] = """<h2>Service types</h2>
<div class="srv-list">
  <ul>
%s
  </ul>
</div>"""%'\n'.join([" <li>%s</li>"%index_type_link('srv', s, package) for s in srvs])
        
    file_p = os.path.join(file_d, 'index-msg.html')
    text = msg_index_template % d
    with open(file_p, 'w') as f:
        #print "writing", file_p
        f.write(text)
        

## generate manifest.yaml files for MoinMoin PackageHeader macro
def generate_msg_docs(ctx):
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

        # roslib.msgs work, load specs into memory
        roslib.msgs.reinit()
        roslib.msgs.load_package_dependencies(p)
        roslib.msgs.load_package(p)

        # create the directory for the autogen files
        file_d = os.path.join(docdir, p, 'html')
        if not os.path.exists(file_d):
            os.makedirs(file_d)

        # get a list of what we are documenting
        msgs = roslib.msgs.list_msg_types(p, False)
        srvs = roslib.srvs.list_srv_types(p, False)

        # generate the top-level index
        wiki_url = '<li>%s</li>\n'%_href(ctx.manifests[p].url, '%s Package Documentation'%p)
        generate_msg_index(p, file_d, msgs, srvs, wiki_url)

        # create dir for msg documentation
        if msgs:
            msg_d = os.path.join(file_d, 'msg')
            if not os.path.exists(msg_d):
                os.makedirs(msg_d)

        # document the messages
        for m in msgs:
            try:
                text = generate_msg_doc('%s/%s'%(p,m))
                file_p = os.path.join(msg_d, '%s.html'%m)
                with open(file_p, 'w') as f:
                    #print "writing", file_p
                    f.write(text)
            except Exception, e:
                print >> sys.stderr, "FAILED to generate for %s/%s: %s"%(p, m, str(e))

        # create dir for srv documentation                
        if srvs:
            srv_d = os.path.join(file_d,'srv')
            if not os.path.exists(srv_d):
                os.makedirs(srv_d)

        # document the services
        for s in srvs:
            try:
                text = generate_srv_doc('%s/%s'%(p,s))
                file_p = os.path.join(srv_d, '%s.html'%s)
                with open(file_p, 'w') as f:
                    #print "writing", file_p
                    f.write(text)
            except Exception, e:
                print >> sys.stderr, "FAILED to generate for %s/%s: %s"%(p, s, str(e))




        
        


        
        
