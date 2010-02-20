#!/usr/bin/env python
from __future__ import with_statement
import os
import sys
from subprocess import call, check_call, Popen

from xml.dom.minidom import getDOMImplementation, parse
import roslib.packages
import roslib.vcs
from roslib.manifest import MANIFEST_FILE

with open('repos','r') as rf:
    reps = [l.strip().split() for l in rf.readlines() if l and l[0] != '#']

for name, uri in reps:
  path = 'checkouts'+os.sep+name
  fresh_install = not os.path.exists(path)
  if fresh_install:
      print "checking out %(name)s from %(uri)s"%locals()
  else:
      print "updating %(name)s from %(uri)s"%locals()
      
  if uri.startswith('git:'):
      if fresh_install:
        cmd = 'cd checkouts && git clone %(uri)s %(name)s'%locals()
      else:
        cmd = "cd %s && git pull"%path
  elif uri.startswith('bzr:'):
      if fresh_install:
        cmd = 'cd checkouts && bzr checkout %(uri)s %(name)s'%locals()
      else:
        cmd = "cd %s && bzr up"%path
  else:
      if fresh_install:
          os.makedirs(path)
      cmd = 'cd checkouts && svn co %(uri)s %(name)s'%locals()

  check_call(cmd, shell=True)

all_pkgs = { }
for name, uri in reps:
    print "snarfing manifests in local copy of %s"%name
    rel_path = 'checkouts'+os.sep+name+os.sep
    # cache paths into repo_pkgs
    repos_pkgs = {}
    for pkg in roslib.packages.list_pkgs_by_path(rel_path, cache=repos_pkgs):
        if pkg not in all_pkgs:
            # de-resolve externals #2456. This logic assumes that the
            # vcs is SVN and that they URLs are consistently
            # specified. It will probably take a lot more logic to get
            # this perfectly right
            override_path = None
            try:
                pkg_path = repos_pkgs[pkg][0]
                vcs, true_uri = roslib.vcs.guess_vcs_uri(pkg_path)
                # rel_path is incorrect if true_uri doesn't
                # match. recompute rel_path by getting URL of package
                # and slicing.
                if vcs == 'svn' and true_uri != uri:
                    url_path = roslib.vcs.get_svn_url(pkg_path)
                    override_path = url_path[len(true_uri):]
            except:
                true_uri = uri
            all_pkgs[pkg] = [pkg, name, true_uri, pkg_path, override_path]

pkgs_dom = getDOMImplementation().createDocument(None, 'pkgs', None)
pkgs_node = pkgs_dom.documentElement

for k, v in all_pkgs.iteritems():
    pkg, repo, uri, rel_path, override_path = v
    manifest_p = rel_path+os.sep+MANIFEST_FILE
    try:
        manifest = parse(manifest_p)
        manifest_node = manifest.documentElement
        manifest_node.setAttribute("name", k)
        manifest_node.setAttribute("repo", repo)
        manifest_node.setAttribute("repo_host", uri)
        rel_path = rel_path[len('checkouts/'+repo):]
        if override_path:
            manifest_node.setAttribute("path", override_path)
        else:
            manifest_node.setAttribute("path", rel_path)            
        pkgs_node.appendChild(manifest_node)
    except:
        import traceback
        traceback.print_exc()
        print "error parsing %s"%manifest_p
    
print "writing megamanifest..."
with open('megamanifest.xml', 'w') as f:
    f.write(pkgs_node.toxml(encoding='utf-8'))
