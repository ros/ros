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

import cStringIO
import time

import roslib.rospack

from rosdoc.rdcore import *

canonicalize = {
    'lgpl': 'LGPL',
    'unknown': 'Unknown',
    'BSD and GPL': 'BSD/GPL',
    'Commercial': 'Proprietary',
    'Closed': 'Commercial',
    'BSD.': 'BSD',
    'new BSD': 'BSD (new)', #for better alphabetizing
    'GPL v3': "GPLv3",
    'GNU GPL V3': 'GPLv3',
    'GPL 3.0': 'GPLv3',
    '?': 'Unknown',
    }

bsdstyle = ['bsd', 'bsd-style', 'new bsd', 'bsd (new)', 'zlib', 'zlib-style', 'mit', 'wxwindows', 'lgpl', 'free', 'python', 'python-style', 'bsl1.0', 'boost', 'apache', 'apache license, version 2.0', 'boost software license', 'public domain', 'apache license']
contaminated = ['gpl', 'creativecommons-by-nc-sa-2.0', 'commercial', 'proprietary']
# keep track of licenses we've reported as unknown
_already_unknown = []

contam_suffix = " (contaminated)"

## all keys must be lower case
license_urls = {
    'boost': 'http://www.boost.org/users/license.html',
    'bsl1.0': 'http://www.boost.org/LICENSE_1_0.txt',
    'boost software license': 'http://www.boost.org/users/license.html',
    'mit': 'http://www.opensource.org/licenses/mit-license.php',
    'new bsd': 'http://www.opensource.org/licenses/bsd-license.php',
    'bsd': 'http://www.opensource.org/licenses/bsd-license.php',
    'zlib': 'http://www.gzip.org/zlib/zlib_license.html',
    'lgpl': 'http://www.opensource.org/licenses/lgpl-2.1.php',
    'lgplv3': 'http://www.opensource.org/licenses/lgpl-3.0.html',
    'gpl': 'http://www.opensource.org/licenses/gpl-2.0.php',
    'gplv3': 'http://www.opensource.org/licenses/gpl-3.0.html',
    'nosa': 'http://www.opensource.org/licenses/nasa1.3.php',
    'creativecommons-by-nc-sa-2.0': 'http://creativecommons.org/licenses/by-nc-sa/2.0/',
    }

license_template = load_tmpl('license-index.html')

################################################################################
# License logic

## check to see if bsd package is contaminated by GPL
## @return str, [str]: license, blamelist
def _check_contaminated(package, license, manifests):
    depslist = roslib.rospack.rospackexec(['deps', package])
    blame = []
    for dep in [d for d in depslist.split('\n') if d]:
        m = manifests[dep]
        dl = m.license or 'unknown'
        dl = dl.lower()
        if dl not in bsdstyle: #anything we don't whitelist is contaminated
            blame.append(dep)
        if 0:
            if dl in contaminated:
                blame.append(dep)
            elif not dl in bsdstyle:
                if not dl in _already_unknown:
                    print "UNKNOWN", dl
                    _already_unknown.append(dl)
    if blame:
        return license + contam_suffix, blame
    else:
        return license, blame
    
## build up map of packages by license
## @return license_map, blame_list: {license: [packages]}, {package: package}
def _generate_licenses_map(ctx):
    blamelist = {}
    licenses = {}
    licenses['unknown'] = []
    
    manifests = ctx.manifests

    for package in ctx.doc_packages:
        
        if not package in manifests:
            licenses['unknown'].append(package)
            continue
        
        manifest = manifests[package]
        license = manifest.license or 'unknown'
        license = canonicalize.get(license, license)
        # reduce pesky 'licenses' that are just long copyright statements
        if 'Copyright' in license:
            license = 'unknown'
        license_url = manifest.license_url
        if license_url and not license in license_urls:
            license_urls[license.lower()] = license_url
        if license.lower() in bsdstyle:
            license, blame = _check_contaminated(package, license, manifests)
            if blame:
                blamelist[package] = blame
        if license in licenses:
            licenses[license].append(package)
        else:
            licenses[license] = [package]
    return licenses, blamelist

################################################################################
# HTML Generation

## @param packages [str]: list of packages to generate 'li' html links to
def li_license_links(packages, blamelist, docdir):
    list = []
    for p in packages:
        path = html_path(p, docdir)
        blame = blamelist.get(p, None)
        blamestr = ': (%s)'%(', '.join(blame)) if blame else ''
        list.append('  <li><a href="%s/index.html">%s</a>%s</li>'%(path, p, blamestr))
    return '<ul>\n'+'\n'.join(list)+'</ul>\n'


## Generate index of packages by license
def generate_license_index(ctx, license_index):
    docdir = ctx.docdir
    
    licenses, blamelist = _generate_licenses_map(ctx)
    
    buff = cStringIO.StringIO()
    keys = licenses.keys()
    keys.sort()
    
    strs = ['<a href="#%s">%s</a> (%s)'%(k, k, len(licenses[k])) for k in keys if licenses[k]]
    contents = ', '.join(strs)

    for license in keys:
        list = licenses[license]
        if not list:
            continue
        #license_urls breaks in our current way of overloading the
        #name to mark contaminated
        
        real_license = license[:-len(contam_suffix)] if license.endswith(contam_suffix) else license
        real_license = real_license.lower()
        if real_license in license_urls:
            buff.write('<h3><a name="%s"></a><a href="%s">%s</a></h2>\n'%\
                       (license, license_urls[real_license], license))
        else:
            buff.write('<h3><a name="%s"></a>%s</h2>\n'%(license, license))
        list.sort()
        buff.write(li_license_links(list, blamelist, docdir))
    
    vars = {'$name': ctx.name, '$licenselist' : buff.getvalue(),
            '$date': time.strftime("%a, %d %b %Y %H:%M:%S"), '$toc': contents }
    with open(license_index, 'w') as f:
        f.write(instantiate_template(license_template, vars))
        

    
    
