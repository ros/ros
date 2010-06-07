#!/usr/bin/env python

# Simple script to check whether two directories are the same.  I'm doing
# it in this script because the following command-line invocation produces
# a syntax error for reasons that I don't understand:
#
#  python -c 'import os; if os.path.realpath("/u/gerkey/code/ros/ros/core/rosconsole") != os.path.realpath("/u/gerkey/code/ros/ros/core/rosconsole"): raise Exception'

import sys, os

if __name__ == '__main__':
  if len(sys.argv) != 3:
    raise Exception
  if os.path.realpath(sys.argv[1]) != os.path.realpath(sys.argv[2]):
    raise Exception
