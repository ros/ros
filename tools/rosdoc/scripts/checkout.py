#!/usr/bin/env python

USAGE = 'checkout.py <rosbrowse_repos_list> <rosdoc_repos_list>'

import fileinput
import sys
import os
import subprocess


def load_rosbrowse_list(fn):
  f_rosbrowse = fileinput.input(fn)
  all_repos = {}
  for l in f_rosbrowse:
    if l.startswith('#'):
      continue
    lsplit = l.split()
    if len(lsplit) != 2:
      continue
    key, uri = lsplit
    all_repos[key] = uri
  return all_repos

def load_rosdoc_list(fn, all_repos):
  f_rosdoc = fileinput.input(fn)
  repos = {}
  for l in f_rosdoc:
    if l.startswith('#'):
      continue
    lsplit = l.split()
    key = lsplit[0].strip()
    if key == 'ros':
      continue
    if key not in all_repos:
      continue
    repos[key] = all_repos[key]
  return repos

def checkout_repos(repos):
  for key in repos:
    url = repos[key]
    fresh_install = os.path.exists(key)
    cmd = None 
    if url.startswith('git:'):
      if fresh_install:
        cmd = 'cd %s && git pull'%key
      else:
        cmd = ['git', 'clone', url, key]
    elif url.startswith('bzr:'):
      url = url[4:]      
      if fresh_install:
        cmd = ['bzr', 'checkout', url, key]
      else:
        cmd = "cd %s && bzr up"%key
    else:
      cmd = ['svn', 'co', url, key]

    if cmd:
      print 'Checking out %s to %s...'%(url, key)
      subprocess.call(cmd, shell=True)
    else:
      print >> sys.stderr, "Unable to checkout %s"%key

def write_setup_file(repos):
  str = 'export ROS_PACKAGE_PATH='
  for key in repos:
    str += os.path.abspath(key) + ':'
  f = open(os.path.abspath('setup.bash'), 'w')
  f.write('%s\n'%str)
  f.close()

def main(argv):
  if len(argv) != 3:
    print USAGE
    sys.exit(1)
  rosbrowse = argv[1]
  rosdoc = argv[2]

  all_repos = load_rosbrowse_list(rosbrowse)
  repos = load_rosdoc_list(rosdoc, all_repos)
  if repos:
    checkout_repos(repos)
    write_setup_file(repos)
  
if __name__ == "__main__":
  main(sys.argv)
