#!/usr/bin/env python

NAME="download_checkmd5.py"

import urllib, md5, os, sys
from optparse import OptionParser

def main():
  parser = OptionParser(usage="usage: %prog URI dest [md5sum]", prog=NAME)
  options, args = parser.parse_args()
  md5sum = None
  if len(args) == 2:
    uri, dest = args
  elif len(args) == 3:
    uri, dest, md5sum = args
  else:
    parser.error("wrong number of arguments")

  urllib.urlretrieve(uri, dest)

  if md5sum:
    m = md5.new(open(dest).read())
    d = m.hexdigest()
  
    if d != md5sum:
      print 'md5sum mismatch (%s != %s); removing file %s'%(d, md5sum, dest)
      os.remove(dest)
      return 1

  return 0


if __name__ == '__main__':
  sys.exit(main())
