#!/usr/bin/env python

USAGE = 'checkout.py <rosbrowse_repos_list> <rosdoc_repos_list>'

import fileinput
import sys
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
    if key not in all_repos:
      continue
    repos[key] = all_repos[key]
  return repos

def checkout_repos(repos):
  for key in repos:
    print 'Checking out %s to %s...'%(repos[key], key)
    cmd = ['svn', 'co', repos[key], key]
    subprocess.call(cmd)

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
  

if __name__ == "__main__":
  main(sys.argv)
