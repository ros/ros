#! /usr/bin/env python

"""
usage: %prog [args]
"""

import os, sys, string
from optparse import OptionParser

import re
import tarfile

TAR_IGNORE_TOP=['build']
TAR_IGNORE_ALL=['.svn']

def tar_exclude(name):
  if name.split('/')[-1] in TAR_IGNORE_ALL:
    return True
  else:
    return False

def package_source(path):
  """
  Create a source tarball from a stack at a particular path.
  
  @param path: the path of the stack to package up
  @return: the path of the resulting tarball, or else None
  """

  # Verify that the stack has both a stack.xml and CMakeLists.txt file
  stack_xml_path = os.path.join(path,'stack.xml')
  cmake_lists_path = os.path.join(path, 'CMakeLists.txt')

  if not os.path.exists(stack_xml_path):
    print >> sys.stderr, "Did not find: [%s]."%stack_xml_path
    return None

  if not os.path.exists(cmake_lists_path):
    print >> sys.stderr, "Did not find: [%s]."%cmake_lists_path
    return None

  # Get the name of the stack
  stack_name = os.path.split(os.path.abspath(path))[-1]

  # Parse the version number from CMakeLists.txt
  with open(cmake_lists_path, 'r') as f:
    m = re.search('rosbuild_make_distribution\((.*)\)',f.read())
    if m is not None:
      stack_version = m.group(1)
    else:
      print >> sys.stderr, "Could not find version number in CMakeLists.txt for stack"
      return None

  # Create the build directory
  build_dir = os.path.join(path, 'build')
  if not os.path.exists(build_dir):
    os.makedirs(build_dir)

  tarfile_name = os.path.join('build','%s-%s.tar.bz2'%(stack_name, stack_version))
  archive_dir = '%s-%s'%(stack_name, stack_version)

  tar = tarfile.open(tarfile_name, 'w:bz2')
  
  for x in os.listdir(path):
    if x not in TAR_IGNORE_TOP + TAR_IGNORE_ALL:
      # path of item
      p = os.path.join(path,x)
      # tar path of item
      tp = os.path.join(archive_dir, x)

      tar.add(p, tp, exclude=tar_exclude)

  tar.close()

  return tarfile_name

def main(argv, stdout, environ):

  parser = OptionParser(__doc__.strip())

  (options, args) = parser.parse_args()

  if (args == 1):
    parser.error("Must provide a path")
    
  tar_file = package_source(args[0])

  if tar_file is None:
    print >> sys.stderr, "Failed to package source."
    sys.exit(1)

  print "Created: %s"%tar_file
  sys.exit(0)

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
