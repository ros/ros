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

from __future__ import with_statement

from subprocess import Popen, PIPE
import tempfile

import roslib.msgs
import roslib.srvs
import roslib.scriptutil as scriptutil

from rosdoc.rdcore import *

doxy_template = load_tmpl('doxy.template')
external_template = load_tmpl('external.html')
header_template = load_tmpl('header.html')
footer_template = load_tmpl('footer.html')
manifest_template = load_tmpl('manifest.html')
wiki_header_template = load_tmpl('wiki-header.html')

## @param ext str: extension (msg or srv)
## @param type str: type name
## @param text str: full text definition of type
def _msg_srv_tmpl(ext, type, text):
    tmpl = '<h2>%s.%s</h2><div style="border: 1px solid #333;"><p style="font-family: monospace;">'%(type, ext)
    for line in text.split('\n'):
        parts = line.split('#')
        if len(parts) > 1:
            tmpl = tmpl + parts[0]+'<font color="blue">#%s</font><br/>'%('#'.join(parts[1:]))
        else:
            tmpl = tmpl + "%s<br />"%parts[0]
    return tmpl+"</p></div>"

## create include files for messages and services
def generate_msg_srv_includes(package, tmp, to_delete):
    for ext, list_types, spec_file in [('msg', roslib.msgs.list_msg_types, roslib.msgs.msg_file),\
                                       ('srv', roslib.srvs.list_srv_types, roslib.srvs.srv_file)]:
        for type_ in list_types(package, False):
            with open(spec_file(package, type_), 'r') as f:
                p = os.path.join(tmp, '%s.%s.html'%(type_, ext))
                with open(p, 'w') as html_file:
                    to_delete.append(p)
                    html_file.write(_msg_srv_tmpl(ext, type_, f.read()))

## @param docdir str: directory to store documentation in    
def create_package_template(package, path, docdir, header_filename, footer_filename):
    #replace vars in the template file to point to package we are documenting
    if not os.path.exists(docdir):
        os.mkdir(docdir)
        
    #TODO: replace with general purpose key/value parser/substitution to enable <export><doxygen key="foo" val="var"></export> feature
    html_output = html_path(package, docdir)
    vars = { '$INPUT':  path, '$PROJECT_NAME': package, '$HTML_HEADER': header_filename, '$HTML_FOOTER': footer_filename,
             '$OUTPUT_DIRECTORY': os.path.join(docdir, package), '$HTML_OUTPUT': html_output}
    return instantiate_template(doxy_template, vars)

## Processes manifest for package and then generates templates for
## header, footer, and manifest include file
## @param package str: package to create templates for
## @param path str: file path to package
## @param m Manifest: package manifest or None
## @return (str, str, str): header, footer, manifest
def load_manifest_vars(package, path, docdir, m):
    author = license = dependencies = description = usedby = status = notes = li_vc = li_url = brief = ''
    wiki_url = 'http://pr.willowgarage.com/wiki/%s'%package
    project_link = '<a href="%s">%s</a>'%(wiki_url, package)
    if m:
        license = m.license or ''
        author = m.author or ''
        description = m.description or ''
        status = m.status or ''
        notes = m.notes or ''

        if m.brief:
            brief = ": "+m.brief

        li_url = '<li>Homepage: <a href=\"%s\">%s</a></li>'%(m.url, m.url)
        if m.versioncontrol:
            vcurl = m.versioncontrol.url
            li_vc = '<li>Version Control (%s): <a href="%s">%s</a></li>'%(m.versioncontrol.type, vcurl, vcurl)

        if m.depends:
            dependencies = "<ul>\n" + \
                           li_package_links(package, [d.package for d in m.depends], docdir)
        else:
            dependencies = "None<br />"

    dependson1 = scriptutil.rospackexec(['depends-on1', package]).split('\n')
    dependson1 = [d for d in dependson1 if d]
    if dependson1:
        usedby = "<ul>\n"+li_package_links(package, dependson1, docdir)
    else:
        usedby = "None<br />"
    
    return {'$package': package,
            '$projectlink': project_link, '$license': license,
            '$dependencies': dependencies, '$usedby': usedby,
            '$description': description, '$brief': brief,
            '$author': author, '$status':status, 
            '$notes':notes, '$li_vc': li_vc, '$li_url': li_url}

## utility to write string data to files and handle unicode 
def _write_to_file(f, tmpl):
    try:
        if type(tmpl) == str:
            f.write(tmpl)
        else: #iso-8859-1 is the declared encoding of doxygen
            f.write(tmpl.encode('utf-8'))
        f.flush()
    except:
        print "ERROR, f[%s], tmpl[%s]"%(f, tmpl)
        raise
    
def run_doxygen(package, doxygen_file):
    try:
        command = ['doxygen', doxygen_file]
        print "doxygen-ating %s [%s]"%(package, ' '.join(command))
        Popen(command, stdout=PIPE).communicate()
    except OSError, (errno, strerr):
        #fatal        
        print """\nERROR: It appears that you do not have doxygen installed.
If you are on Ubuntu/Debian, you can install doxygen by typing:

   sudo apt-get install doxygen
"""
        sys.exit(1) 

def run_rxdeps(package, dir):
    try:
        command = ['rxdeps', '-s', '--target=%s'%package, '--cluster', '-o', os.path.join(dir, package, 'html', '%s_deps.pdf'%package)]
        print "rxdeping %s [%s]"%(package, ' '.join(command))
        Popen(command, stdout=PIPE).communicate()
    except OSError, (errno, strerr):
        print """\nERROR: It appears that you do not have rxdeps installed. 
Package dependency tree links will not work properly.
"""

## Main entrypoint into creating doxygen files
## @return [str]: list of packages that were successfully generated
def generate_doxygen(ctx):
    # setup temp directory
    tmp = 'tmp'
    if not os.path.exists(tmp):
        os.mkdir(tmp)

    success = []
    
    dir = ctx.docdir
    packages = ctx.packages
    filters = ctx.filters
    external_docs = ctx.external_docs
    manifests = ctx.manifests

    tmpls = [header_template, footer_template, manifest_template, wiki_header_template]
    try:
        for package, path in packages.iteritems():
            if filters and not package in filters:
                continue

            html_dir = os.path.join(dir, package, 'html')
            # have to makedirs for external packages
            if not os.path.exists(html_dir):
                os.makedirs(html_dir)
                
            files = []
            try:
                #TODO: cleanup the temporary files

                header_file = tempfile.NamedTemporaryFile('w+')
                footer_file = tempfile.NamedTemporaryFile('w+')
                doxygen_file = tempfile.NamedTemporaryFile('w+')
                manifest_file = open(os.path.join(tmp, 'manifest.html'), 'w')
                files = [header_file, footer_file, manifest_file, doxygen_file]
                to_delete = [manifest_file]

                # create the doxygen templates and wiki header

                generate_msg_srv_includes(package, tmp, to_delete)

                # - instantiate the templates
                manifest_ = manifests[package] if package in manifests else None
                vars = load_manifest_vars(package, path, dir, manifest_)
                header, footer, manifest_html, wiki_header = [instantiate_template(t, vars) for t in tmpls]

                run_rxdeps(package, dir)
                if package not in external_docs:
                    doxy = create_package_template(package, path, dir, header_file.name, footer_file.name)
                    for f, tmpl in zip(files, [header, footer, manifest_html, doxy]):
                        _write_to_file(f, tmpl)
                    # doxygenate
                    run_doxygen(package, doxygen_file.name)
                else:
                    # for external packages, we generate a landing page that is
                    # similar to the doxygen, but we don't actually run doxygen as
                    # it is time consuming for packages that provide their own docs

                    external_link = ctx.external_docs[package]
                    vars = { '$package': package, '$external_link': external_link,
                             '$header': header, '$footer': footer,
                             '$manifest': manifest_html,
                             # doxygen vars
                             '$relpath$': '../../',
                             '$title': package+': Main Page',
                             }
                    with open(os.path.join(html_dir, 'index.html'), 'w') as ext_html_file:
                        _write_to_file(ext_html_file, instantiate_template(external_template, vars))

                with open(os.path.join(html_dir, 'wiki_header.html'), 'w') as wiki_header_file:
                    _write_to_file(wiki_header_file, wiki_header)

                success.append(package)
            finally:
                for f in files:
                    f.close()
    finally:
        pass
    return success
