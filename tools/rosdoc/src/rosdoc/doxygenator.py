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
import roslib.rospack 

from rosdoc.rdcore import *

doxy_template = load_tmpl('doxy.template')
external_template = load_tmpl('external.html')
header_template = load_tmpl('header.html')
footer_template = load_tmpl('footer.html')
manifest_template = load_tmpl('manifest.html')

# other templates

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

## @param package str: package name
## @param rd_config dict: rosdoc configuration parameters for this doxygen build
## @param m Manifest : package manifest
## @param html_dir str: directory to store doxygen files                    
def create_package_template(package, rd_config, m, path, html_dir,
                            header_filename, footer_filename):
    # TODO: allow rd_config to specify excludes and whatnot
    
    # TODO: replace with general purpose key/value parser/substitution to enable <export><doxygen key="foo" val="var"></export> feature

    # determine the value of overridable keys
    file_patterns = '*.c *.cpp *.h *.cc *.hh *.hpp *.py *.dox'
    excludes = '%s/build/'%path
    # last one wins
    for e in m.get_export('doxygen', 'excludes'):
        # prepend the packages path
        excludes = '%s/%s'%(path, e)
    # rd_config wins
    if rd_config and 'excludes' in rd_config:
        excludes = rd_config['excludes']

    # last one wins        
    for e in m.get_export('doxygen', 'file-patterns'):
        file_patterns = e
    # rd_config wins
    if rd_config and 'file_patterns' in rd_config:
        file_patterns = rd_config['file_patterns']
        
    vars = { '$INPUT':  path, '$PROJECT_NAME': package,
             '$EXCLUDE_PROP': excludes, '$FILE_PATTERNS': file_patterns,
             '$HTML_OUTPUT': os.path.abspath(html_dir),
             '$HTML_HEADER': header_filename, '$HTML_FOOTER': footer_filename,
             '$OUTPUT_DIRECTORY': html_dir}
    return instantiate_template(doxy_template, vars)

## Processes manifest for package and then generates templates for
## header, footer, and manifest include file
## @param package: package to create templates for
## @type  package: str
## @param rd_config: rosdoc configuration dictionary
## @type  rd_config: dict
## @param path: file path to package
## @type  path: str
## @param m: package manifest or None
## @type  m: manifest.Manifest
## @return: header, footer, manifest
## @rtype: (str, str, str)
def load_manifest_vars(ctx, rd_config, package, path, docdir, package_htmldir, m):
    author = license = dependencies = description = usedby = status = notes = li_vc = li_url = brief = ''
    
    # by default, assume that packages are on wiki
    home_url = 'http://ros.org/wiki/%s'%package

    if rd_config:
        if 'homepage' in rd_config:
            home_url = rd_config['homepage']
            print "HOMEPAGE", home_url
            
    project_link = '<a href="%s">%s</a>'%(home_url, package)
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
                           li_package_links(ctx, package, [d.package for d in m.depends], docdir, package_htmldir)
        else:
            dependencies = "None<br />"

    dependson1 = roslib.rospack.rospackexec(['depends-on1', package]).split('\n')
    # filter depends by what we're actually documenting
    dependson1 = [d for d in dependson1 if d and ctx.should_document(d)]
    if dependson1:
        usedby = "<ul>\n"+li_package_links(ctx, package, dependson1, docdir, package_htmldir)
    else:
        usedby = "None<br />"

    # include links to msgs/srvs
    msgs = roslib.msgs.list_msg_types(package, False)
    srvs = roslib.srvs.list_srv_types(package, False)
        
    return {'$package': package,
            '$projectlink': project_link, '$license': license,
            '$dependencies': dependencies, '$usedby': usedby,
            '$description': description, '$brief': brief,
            '$author': author, '$status':status, 
            '$notes':notes, '$li_vc': li_vc, '$li_url': li_url,
            }

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
    
def run_doxygen(package, doxygen_file, quiet=False):
    try:
        command = ['doxygen', doxygen_file]
        print "doxygen-ating %s [%s]"%(package, ' '.join(command))
        if quiet:
            Popen(command, stdout=PIPE, stderr=PIPE).communicate()
        else:
            Popen(command, stdout=PIPE).communicate()            
    except OSError, (errno, strerr):
        #fatal        
        print """\nERROR: It appears that you do not have doxygen installed.
If you are on Ubuntu/Debian, you can install doxygen by typing:

   sudo apt-get install doxygen
"""
        sys.exit(1) 

#TODO: move elsewhere
def run_rxdeps(package, pkg_doc_dir):
    try:
        command = ['rxdeps', '-s', '--target=%s'%package, '--cluster', '-o', os.path.join(pkg_doc_dir, '%s_deps.pdf'%package)]
        print "rxdeping %s [%s]"%(package, ' '.join(command))
        Popen(command, stdout=PIPE).communicate()
    except OSError, (errno, strerr):
        print >> sys.stderr, """\nERROR: It appears that you do not have rxdeps installed. 
Package dependency tree links will not work properly.
"""
    except:
        print >> sys.stderr, "ERROR: rxdeps failed"

## Main entrypoint into creating doxygen files
## @param disable_rxdeps: if True, don't generate rxdeps documenation (note: this parameter is volatile as rxdeps generation will be moved outside of doxygenator)
## @type  disable_rxdeps: bool        
## @param quiet: suppress most stdout output
## @type  quiet: bool        
## @return [str]: list of packages that were successfully generated
def generate_doxygen(ctx, quiet=False, disable_rxdeps=False):

    #TODO: move external generator into its own generator
    #TODO: move rxdeps into its own generator    
    
    # setup temp directory
    tmp = 'tmp'
    if not os.path.exists(tmp):
        os.mkdir(tmp)

    success = []
    
    dir = ctx.docdir
    # dictionary mapping packages to paths
    packages = ctx.packages
    # list of packages that we are documenting
    doc_packages = ctx.doc_packages
    external_docs = ctx.external_docs
    rd_configs = ctx.rd_configs
    manifests = ctx.manifests

    tmpls = [header_template, footer_template, manifest_template]
    try:
        for package, path in packages.iteritems():
            if not package in doc_packages or \
                   not ctx.has_builder(package, 'doxygen'):
                continue

            print "doxygenate", package

            # the logic for the doxygen builder is different from
            # others as doxygen is the default builder if no config is
            # declared
            rd_config = rd_configs.get(package, None)
            if rd_config:
                # currently only allow one doxygen build per package. This is not inherent, it
                # just requires rewriting higher-level logic
                rd_config = [d for d in ctx.rd_configs[package] if d['builder'] == 'doxygen'][0]

            # Configuration (all are optional)
            #
            # name: Documentation set name (e.g. C++ API)
            # output_dir: Directory to store files (default '.')
            # file_patterns: override FILE_PATTERNS
            # excludes: override EXCLUDES
            
            # doxygenator currently does some non-doxygen work.
            # pkg_doc_dir is the pointer to the directory for these non-doxygen
            # tools. html_dir is the path for doxygen
            pkg_doc_dir = html_path(package, ctx.docdir)
                
            # compute the html directory for doxygen
            html_dir = html_path(package, ctx.docdir)
            if rd_config and 'output_dir' in rd_config:
                html_dir = os.path.join(html_dir, rd_config['output_dir'])

            # have to makedirs for external packages
            if not os.path.exists(pkg_doc_dir):
                os.makedirs(pkg_doc_dir)
                    
            files = []
            try:
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
                vars = load_manifest_vars(ctx, rd_config, package, path, dir, html_dir, manifest_)
                header, footer, manifest_html = [instantiate_template(t, vars) for t in tmpls]

                if not disable_rxdeps:
                    run_rxdeps(package, pkg_doc_dir)
                if package not in external_docs:
                    doxy = \
                        create_package_template(package, rd_config, manifest_,
                                                path, html_dir,
                                                header_file.name, footer_file.name)
                    for f, tmpl in zip(files, [header, footer, manifest_html, doxy]):
                        _write_to_file(f, tmpl)
                    # doxygenate
                    run_doxygen(package, doxygen_file.name, quiet=quiet)
                else:
                    # for external packages, we generate a landing page that is
                    # similar to the doxygen, but we don't actually run doxygen as
                    # it is time consuming for packages that provide their own docs

                    external_link = ctx.external_docs[package]

                    # Override mainpage title if 'name' is in config
                    title = 'Main Page'
                    if rd_config:
                        title = rd_config.get('name', title)
                    vars = { '$package': package, '$external_link': external_link,
                             '$header': header, '$footer': footer,
                             '$manifest': manifest_html,
                             # doxygen vars
                             '$relpath$': '../../',
                             '$title': package+': '+title,
                             }

                    with open(os.path.join(pkg_doc_dir, 'index.html'), 'w') as ext_html_file:
                        _write_to_file(ext_html_file, instantiate_template(external_template, vars))
                        
                # support files (stylesheets)
                import shutil
                dstyles_in = os.path.join(ctx.template_dir, 'doxygen.css')
                dstyles_css = os.path.join(html_dir, 'doxygen.css')
                shutil.copyfile(dstyles_in, dstyles_css)

                success.append(package)
            finally:
                for f in files:
                    f.close()
    finally:
        pass
    return success
