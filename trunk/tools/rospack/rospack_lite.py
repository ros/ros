#!/usr/bin/env python
# copyright 2009 morgan quigley, bsd license blah blah

import sys, os

if len(sys.argv) < 2:
  print >> sys.stderr, "this version only supports 'find', 'list', and 'list-names'"
  exit(1);

command = sys.argv[1]

if not 'ROS_ROOT' in os.environ:
  print >> sys.stderr, "ROS_ROOT is not defined in the environment"
  exit(1)
s = [ os.environ['ROS_ROOT'] ]  # stack of paths to crawl
if 'ROS_PACKAGE_PATH' in os.environ:
  s += os.environ['ROS_PACKAGE_PATH'].split(':')

pkgs = { }

while len(s) > 0:
  p = s.pop()
  if os.path.exists(os.path.join(p, 'manifest.xml')): # see if p is a package
    pkgs[os.path.basename(p)] = p
  else: # crawl it
    for child in os.listdir(p):
      c = os.path.join(p, child)
      if os.path.isfile(c) or os.path.islink(c) or os.path.basename(c)[0] == '.':
        continue # forget this guy
      if os.path.exists(os.path.join(c, 'rospack_nosubdirs')):
        continue # bail, as requested
      # if we get here, it's safe to descend. push this child onto the stack
      s.append(c)

if command == 'list':
  for k, v in pkgs.iteritems():
    print "%s %s" % (k, v)
elif command == 'list-names':
  for k, v in pkgs.iteritems():
    print k
elif command == 'find':
  if len(sys.argv) < 3:
    print >> sys.stderr, "syntax: rospack find PACKAGE"
    exit(1)
  if sys.argv[2] in pkgs:
    print pkgs[sys.argv[2]]
else:
  print >> sys.stderr, "rospack lite, python style!\nsupported commands:\nrospack find PACKAGE\nrospack list\nrospack list-names\n"
  
