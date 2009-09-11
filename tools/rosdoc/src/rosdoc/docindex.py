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

import os
import cStringIO
import time

from rosdoc.rdcore import *

doc_index_template = load_tmpl('doc-index.html')

################################################################################
# Generate HTML

# utility for sorting
def caseless_comp(x, y):
    if x.lower() < y.lower():
        return -1
    return 1

def render_tree(key, tree, ctx, success, depth=0):
    keys = tree.keys()
    if keys:
        keys.sort(caseless_comp)
        
        subkeys = [k for k in keys if tree[k]]
        leafkeys = [k for k in keys if k not in subkeys]
        
        list_ = [render_tree(k, tree[k], ctx, success, depth+1).strip() for k in leafkeys]
        list_.extend([render_tree(k, tree[k], ctx, success, depth+1).strip() for k in subkeys])
        list_ = [l for l in list_ if l]
        if key:
            if not list_:
                return ''
            if depth <= 1:
                return '<h3 class="group"><a name="%s"></a><a href="#%s"><b>%s</b></a></h3>\n<ul>'%(key, key, key)+'\n'.join(list_)+'</ul>'
            else:
                return '<h4 class="subgroup">%s</h4>\n<ul>'%key+'\n'.join(list_)+'</ul>'
        else:
            return '\n'.join(list_)
    elif key in success: #leaf package node
        return '<li class="package"><a href="%s/index.html">%s</a></li>'%(compute_relative(ctx.docdir, html_path(key, ctx.docdir)), key)
    return ''

def _tree_size(tree):
    if tree.keys():
        return sum([_tree_size(tree[k]) for k in tree.keys()])
    else:
        return 1
    
def generate_doc_index(ctx, success, path):
    tree = generate_package_tree(ctx)
    success.sort(caseless_comp)
    keys = tree.keys()
    keys.sort(caseless_comp)

    contents = ', '.join(['<a href="#%s">%s</a> (%s)'%(k,k, _tree_size(tree[k])) for k in keys if tree[k]])
    package_list = render_tree('', tree, ctx, success)

    date = str(time.strftime('%a, %d %b %Y %H:%M:%S'))
    vars = {'$name': ctx.name, '$packagelist' : package_list, '$date': date, '$toc': contents, '$package_count' : str(_tree_size(tree)) }
    if not os.path.isdir(os.path.dirname(path)):
        os.makedirs(os.path.dirname(path))
    with open(path, 'w') as f:
        f.write(instantiate_template(doc_index_template, vars))
