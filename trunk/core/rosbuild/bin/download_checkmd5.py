#!/usr/bin/env python

NAME="download_checkmd5.py"

import urllib, hashlib, os, sys
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

  # Create intermediate directories as necessary, #2970
  d = os.path.dirname(dest)
  if len(d) and not os.path.exists(d):
    os.makedirs(d)

  fresh = False
  if not os.path.exists(dest):
    sys.stdout.write('[rosbuild] Downloading %s to %s...'%(uri, dest))
    sys.stdout.flush()
    urllib.urlretrieve(uri, dest)
    sys.stdout.write('Done\n')
    fresh = True

  if md5sum:
    m = hashlib.md5(open(dest).read())
    d = m.hexdigest()

    print '[rosbuild] Checking md5sum on %s'%(dest)
  
    if d != md5sum:
      if not fresh:
        print '[rosbuild] WARNING: md5sum mismatch (%s != %s); re-downloading file %s'%(d, md5sum, dest)
        os.remove(dest)

        # Try one more time
        urllib.urlretrieve(uri, dest)
        m = hashlib.md5(open(dest).read())
        d = m.hexdigest()
    
      if d != md5sum:
        print '[rosbuild] ERROR: md5sum mismatch (%s != %s) on %s; aborting'%(d, md5sum, dest)
        return 1

  return 0


if __name__ == '__main__':
  sys.exit(main())
